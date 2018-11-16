#include "Task_IntersectionHandling.h"
#include "../../Helpers/cariosity_structs.h"
#include "stdafx.h"


#define DEBUG_MODE (false)

#define SHOW_DEBUG_MESSAGES (true)
#define SHOW_STATE_PROGRESS (false)
#define CONSOLE_LOG_INFO(...) if (SHOW_DEBUG_MESSAGES) LOG_ADD_ENTRY(A_UTILS_NS::log::tLogLevel::Info, __VA_ARGS__)
#define CONSOLE_LOG_PROGRESS(...) if (SHOW_DEBUG_MESSAGES && SHOW_STATE_PROGRESS) LOG_ADD_ENTRY(A_UTILS_NS::log::tLogLevel::Info, __VA_ARGS__)

#define DEG_TO_RAD(x) (tFloat32(((x) * M_PI / 180.0)))
#define RAD_TO_DEG(x) (tFloat32(((x) / M_PI * 180.0)))


#define FRONT_AXLE_OFFSET (0.365f)
#define LASER_SCANNER_OFFSET (0.45f)
#define ROAD_LANE_WIDTH (0.5f)
#define BASE_SPEED (0.5f)


// APPROACH AND STOP
#define APPROACH_SPEED (0.1f)
#define STOP_LINE_OFFSET_FROM_JUNCTION (0.5f)
#define SLOW_DOWN_DISTANCE_PRIOR_TO_JUNCTION (0.5f)

#define MAXIMUM_TIME_TO_WAIT_FOR_OBSTACLES (10e6)
#define TIME_TO_WAIT_AT_STOP_SIGN (1e6) // in micro seconds
#define TIME_TO_WAIT_FOR_PEOPLE   (3e6) // in micro seconds
#define TIME_TO_WAIT_AFTER_EMERGENCY (2e6) // in micro seconds

// CHECK RIGHT OF WAY
#define MIN_DISTANCE_TO_OBSTACLE (2.0f)

// ENTER_INTERSECTION
#define ENTER_SPEED (BASE_SPEED * 0.8f)


// STEERING
#define TURN_SPEED (BASE_SPEED)
#define RADIUS_AT_FULL_LOCK (0.35f)
// use relative turn angle instead of absolute exitPoint heading (uncomment to use absolute)
//#define USE_RELATIVE_TURN_ANGLE
//#define ESTIMATE_HEADING
#define TURN_ANGLE_REDUCTION (DEG_TO_RAD(10.0f))
#define TURN_ANGLE (DEG_TO_RAD(90.0f) - TURN_ANGLE_REDUCTION)

#define LANE_DETECTION_OVERTAKE_ANGLE (DEG_TO_RAD(10.0f))
#define LANE_DETECTION_MIN_CONFIDENCE_THRESHOLD (12)

#define BEGIN_EASE_OUT_ANGLE LANE_DETECTION_OVERTAKE_ANGLE

// EXIT
#define EXIT_SPEED (BASE_SPEED)



//SIMULATION
#define SIMULATE_EMERGENCY_CAR (DEBUG_MODE && false)
#define SIMULATE_PERSON (DEBUG_MODE && false)
#define SIMULATE_OBSTACLE_STRAIGHT (DEBUG_MODE && false)
#define SIMULATE_OBSTACLE_RIGHT (DEBUG_MODE && false)
#define SIMULATE_OBSTACLE_LEFT (DEBUG_MODE && false)




/*! Calls parent constructor with default values */
cTask_IntersectionHandling::cTask_IntersectionHandling(tBool isFirstTaskOfManeuver, aadc::jury::maneuver direction) : cTask(isFirstTaskOfManeuver), m_direction(direction) {}

tResult cTask_IntersectionHandling::execute(tTaskResult &taskResult) {

    if (!cEgoState::isInitialized()) return cResult(ERR_NOT_INITIALIZED);

    if (m_state == SEARCHING_FOR_ENTRY) {
        LOG_WARNING("Searching for intersection entry!");
        if(!canTakeOverControl()) {
            taskResult.f32steeringAngle = cEgoState::singleton->getRelativeLanePositioning();
            taskResult.f32speed = APPROACH_SPEED;
            RETURN_NOERROR;
        }
    }


    if (m_state == INITIALIZE) {
        CONSOLE_LOG_INFO("Initializing Intersection Handling!");

        vector<Pose3D> poses;

        switch (m_direction) {
            case aadc::jury::maneuver::maneuver_right:
                CONSOLE_LOG_INFO("GOING RIGHT");
                poses = m_junctionEntry.rightTurn;
                break;
            case aadc::jury::maneuver::maneuver_left:
                CONSOLE_LOG_INFO("GOING LEFT");
                poses = m_junctionEntry.leftTurn;
                break;
            default:
                CONSOLE_LOG_INFO("GOING STRAIGHT");
                poses = m_junctionEntry.straight;
                break;
        }


        if (poses.empty()) {
            LOG_WARNING("Selection invalid -> trying '%d' but size is ( %u | %u | %u ) at ( %f | %f )", m_direction, m_junctionEntry.leftTurn.size(), m_junctionEntry.straight.size(), m_junctionEntry.rightTurn.size(), m_junctionEntry.entryPoint.p.x, m_junctionEntry.entryPoint.p.y);

            // selection invalid -> there is no road in that direction
            taskResult.lights.bHazardLight = tTrue;
            taskResult.f32speed = 0.0f;
            taskResult.bEmergencyBrake = tTrue;

            m_taskState = taskState::error;

            RETURN_NOERROR;
        }


        m_target = cOpenDriveMapAnalyzer::convertPoseToVehiclePosition(poses.back());
        //m_target.f32heading = normalizeAngle(egoStateStruct.tPosVehiclePosition.f32heading) + M_PI_2;
        m_state = APPROACH_INTERSECTION;
    }

    // PROCESSING
    // Headlights
    taskResult.lights.bHeadLight = tTrue;

    // Turn indicator
    switch (m_direction) {
        case aadc::jury::maneuver::maneuver_right:
            taskResult.lights.bTurnSignalRight = tTrue;
            break;
        case aadc::jury::maneuver::maneuver_left:
            taskResult.lights.bTurnSignalLeft = tTrue;
            break;
        default: break;
    }

    //TODO: Headlights / brakelights

    tVehiclePosition frontAxlePos = getFrontAxlePos(cEgoState::singleton->getVehiclePosition());

    Point targetStrich = transform(frontAxlePos, m_target);

    CONSOLE_LOG_PROGRESS(
            "state: %d || target heading %f || current heading %f || pos: ( %f | %f )",
             m_state, RAD_TO_DEG(m_target.f32heading), RAD_TO_DEG(frontAxlePos.f32heading), targetStrich.x, targetStrich.y
    );

    switch (m_state) {
        case APPROACH_INTERSECTION: {
            cEgoState::singleton->setYOLOState(true);

            m_personDetected |= SIMULATE_PERSON || !cEgoState::singleton->getPersons().empty() || !cEgoState::singleton->getChilds().empty();
            if (SIMULATE_EMERGENCY_CAR || cEgoState::singleton->isEmergencyCarPresent()) {
                m_timeLastEmergencyVehicle = cEgoState::singleton->getLastUpdated();
            }

            tFloat32 distanceToStopLine = 0.0f;

            if (!m_junctionEntry.rightTurn.empty()) { // we can turn to the right
                Point rightExitPoint = transform(frontAxlePos, cOpenDriveMapAnalyzer::convertPoseToVehiclePosition(m_junctionEntry.rightTurn.back())); // exit point
                distanceToStopLine += rightExitPoint.x - (ROAD_LANE_WIDTH * 0.5f);
            }

            if (!m_junctionEntry.leftTurn.empty()) { // we can turn to the left
                Point leftExitPoint = transform(frontAxlePos, cOpenDriveMapAnalyzer::convertPoseToVehiclePosition(m_junctionEntry.leftTurn.back())); // exit point
                distanceToStopLine += leftExitPoint.x - (ROAD_LANE_WIDTH * 1.5f);
            }

            if (!m_junctionEntry.leftTurn.empty() && !m_junctionEntry.rightTurn.empty()) {
                distanceToStopLine /= 2.0f;
            }

            distanceToStopLine -= STOP_LINE_OFFSET_FROM_JUNCTION;

            if (distanceToStopLine > 0) {
                // we've not yet reached the stop line

                // if there is no straight option, dont use lane detection
                taskResult.f32steeringAngle = m_junctionEntry.straight.empty() ? 0.0f : cEgoState::singleton->getRelativeLanePositioning();

                if (distanceToStopLine > SLOW_DOWN_DISTANCE_PRIOR_TO_JUNCTION || (m_junctionEntry.roadSign == RAKETE && m_direction != aadc::jury::maneuver::maneuver_left)) {
                    taskResult.f32speed = BASE_SPEED;
                } else {
                    taskResult.f32speed = APPROACH_SPEED;
                }

                //TODO: STOP IF EMERGENCY?
                RETURN_NOERROR;

            } else {
                m_state = WAIT_AT_STOP_SIGN;
                m_timeStartedWaitingAtJunctionEntry = cEgoState::singleton->getLastUpdated();
            }
        }
        case WAIT_AT_STOP_SIGN: {
            cEgoState::singleton->setYOLOState(true);

            tTimeStamp currentTime = cEgoState::singleton->getLastUpdated();

            if (cEgoState::singleton->isEmergencyCarPresent()) {
                m_timeLastEmergencyVehicle = currentTime;
            }

            tTimeStamp timeWaitingAtJunctionEntry = currentTime - m_timeStartedWaitingAtJunctionEntry;
            tTimeStamp timeWaitingSinceEmergencyVehicle = currentTime - m_timeLastEmergencyVehicle;

            //TODO: way of right -> which signs were really at the junction?
            // TODO check for people here and emergency cars

            if ((m_junctionEntry.roadSign == STOP && timeWaitingAtJunctionEntry < TIME_TO_WAIT_AT_STOP_SIGN)
                || (!cEgoState::singleton->getChilds().empty() && timeWaitingAtJunctionEntry < TIME_TO_WAIT_FOR_PEOPLE)
                || (!cEgoState::singleton->getPersons().empty() && timeWaitingAtJunctionEntry < TIME_TO_WAIT_FOR_PEOPLE)
                || (timeWaitingSinceEmergencyVehicle < TIME_TO_WAIT_AFTER_EMERGENCY)
                || (timeWaitingAtJunctionEntry < MAXIMUM_TIME_TO_WAIT_FOR_OBSTACLES && !checkRightOfWay())
            ) { //TODO: NEEDS TESTING

                taskResult.f32steeringAngle = 0.0f;
                taskResult.f32speed = 0.0f;
                taskResult.bEmergencyBrake = tTrue; // just to make sure

                CONSOLE_LOG_PROGRESS("In State CHECK_RIGHT_OF_WAY: Coast is NOT clear. Already waiting for %.2f seconds.", (timeWaitingAtJunctionEntry * 1.0f / 1e6));
                RETURN_NOERROR;
            } else {
                m_state = ENTER_INTERSECTION;
                CONSOLE_LOG_INFO("In State CHECK_RIGHT_OF_WAY: Coast is clear. Proceeding with ENTER_INTERSECTION ...");
            }
        }

        case ENTER_INTERSECTION:

            cEgoState::singleton->setYOLOState(false); // we need the precision and samples here

            CONSOLE_LOG_PROGRESS("ENTER_INTERSECTION: approaching (%f > %f)  target: ( %f | %f )", targetStrich.x, RADIUS_AT_FULL_LOCK, targetStrich.x, targetStrich.y);
            if (targetStrich.x > RADIUS_AT_FULL_LOCK) {
                // if there is no straight option, dont use lane detection
                taskResult.f32steeringAngle = m_junctionEntry.straight.empty() ? 0.0f : cEgoState::singleton->getRelativeLanePositioning();
                taskResult.f32speed = ENTER_SPEED;
                RETURN_NOERROR;
            }

            {
                // set target heading to be 90 degrees offset of current heading
                tFloat32 relativeTargetHeading = frontAxlePos.f32heading;
                tFloat32 absoluteTargetHeading = m_target.f32heading;
                switch (m_direction) {
                    case aadc::jury::maneuver::maneuver_right:
                        relativeTargetHeading -= TURN_ANGLE;
                        absoluteTargetHeading += TURN_ANGLE_REDUCTION;
                        break;
                    case aadc::jury::maneuver::maneuver_left:
                        relativeTargetHeading += TURN_ANGLE;
                        absoluteTargetHeading -= TURN_ANGLE_REDUCTION;
                        break;
                    default:
                        break;
                }

                relativeTargetHeading = normalizeAngle(relativeTargetHeading);
                absoluteTargetHeading = normalizeAngle(absoluteTargetHeading);

#ifdef USE_RELATIVE_TURN_ANGLE
                CONSOLE_LOG_INFO("USING RELATIVE ANGLE %f instead of absolute %f. (current: %f)", RAD_TO_DEG(relativeTargetHeading), RAD_TO_DEG(absoluteTargetHeading), RAD_TO_DEG(frontAxlePos.f32heading));
                m_target.f32heading = relativeTargetHeading;
#else
                CONSOLE_LOG_INFO("USING ABSOLUTE ANGLE %f instead of relative %f. (current: %f)", RAD_TO_DEG(absoluteTargetHeading), RAD_TO_DEG(relativeTargetHeading), RAD_TO_DEG(frontAxlePos.f32heading));
                m_target.f32heading = absoluteTargetHeading;
#endif
            }

            m_state = STEERING;
            //fallthrough intended!
            CONSOLE_LOG_INFO("ENTERING State STEERING (%d)", STEERING);

        case STEERING: {
            cEgoState::singleton->setYOLOState(false);

            tFloat32 remainingHeading = getRelativeAngle(frontAxlePos.f32heading, m_target.f32heading);
            cEgoState::singleton->setLaneDetectionAngleOfFieldOfView(remainingHeading);
            cEgoState::singleton->setOverrideLaneDetectionAngleOfFielOfView(true);

#ifdef ESTIMATE_HEADING
            tTimeStamp currentTime = cEgoState::singleton->getLastUpdated();
            tTimeStamp timeElapsed = currentTime - m_timeRemainingHeadingLastUpdated;
            if (remainingHeading != m_lastRemainingHeading) {
                if (m_timeRemainingHeadingLastUpdated != 0) {
                    m_angleVelocity = (remainingHeading - m_lastRemainingHeading) / (timeElapsed);
                }
                m_lastRemainingHeading = remainingHeading;
                m_timeRemainingHeadingLastUpdated = currentTime;
            } else {
                tFloat32 estimatedHeading = remainingHeading + m_angleVelocity * timeElapsed;
                CONSOLE_LOG_INFO("remainingHeading: %.2f | estimatedHeading: %.2f", RAD_TO_DEG(remainingHeading), RAD_TO_DEG(estimatedHeading));
                remainingHeading = estimatedHeading;
            }
#endif


            switch (m_direction) {
                case aadc::jury::maneuver::maneuver_right:
                    CONSOLE_LOG_PROGRESS("STEERING: right %f (%f > %f)", RAD_TO_DEG(remainingHeading), RAD_TO_DEG(frontAxlePos.f32heading), RAD_TO_DEG(m_target.f32heading));

                    // wenn wir uns mindestens bis zu einem bestimmten winkel gedreht haben, check if lane detection can take over
                    if ((cEgoState::singleton->getLaneDetectionConfidence() > LANE_DETECTION_MIN_CONFIDENCE_THRESHOLD) && (remainingHeading < LANE_DETECTION_OVERTAKE_ANGLE)) {
                        // yes, lets shift control to lane detection -> exit intersection
                        CONSOLE_LOG_INFO("LANE DETECTION CAN CONTINUE! -> found %d lines. (remainingHeading: %f)", cEgoState::singleton->getLaneDetectionConfidence(), RAD_TO_DEG(remainingHeading));
                    } else if (remainingHeading > 0) {

                        taskResult.f32speed = TURN_SPEED;
                        taskResult.f32steeringAngle = 100.0f;


                        // do not steer that much the last 20 degrees (see BEGIN_EASE_OUT_ANGLE)
                        if (remainingHeading < BEGIN_EASE_OUT_ANGLE) {
                            // 20 -> 100  | 0 -> 0
                            taskResult.f32steeringAngle = (remainingHeading * 5.0f);
                        }

                        RETURN_NOERROR;
                    }
                    break;
                case aadc::jury::maneuver::maneuver_left:
                    CONSOLE_LOG_PROGRESS("STEERING: left %f (%f < %f)", RAD_TO_DEG(remainingHeading), RAD_TO_DEG(frontAxlePos.f32heading), RAD_TO_DEG(m_target.f32heading));
                    // wenn wir uns mindestens bis zu einem bestimmten winkel gedreht haben, check if lane detection can take over
                    if ((cEgoState::singleton->getLaneDetectionConfidence() > LANE_DETECTION_MIN_CONFIDENCE_THRESHOLD) && (remainingHeading > -LANE_DETECTION_OVERTAKE_ANGLE)) {
                        // yes, lets shift control to lane detection -> exit intersection
                        CONSOLE_LOG_INFO("LANE DETECTION CAN CONTINUE! -> found %d lines. (remainingHeading: %f)", cEgoState::singleton->getLaneDetectionConfidence(), RAD_TO_DEG(remainingHeading));
                    } else if (remainingHeading < 0) {

                        taskResult.f32speed = TURN_SPEED;
                        taskResult.f32steeringAngle = -100.0f;

                        // do not steer that much the last 20 degrees (see BEGIN_EASE_OUT_ANGLE)
                        if (remainingHeading > -BEGIN_EASE_OUT_ANGLE) {
                            // -20 -> -100  | 0 -> 0
                            taskResult.f32steeringAngle = (remainingHeading * 5.0f);
                        }


                        RETURN_NOERROR;
                    }
                    break;
                default:
                    CONSOLE_LOG_INFO("STEERING: straight = fallthrough");
                    break;
            }

            m_state = EXIT_INTERSECTION;
            cEgoState::singleton->setOverrideLaneDetectionAngleOfFielOfView(false);
            //fallthrough intended!
            CONSOLE_LOG_INFO("ENTERING State EXIT_INTERSECTION (%d)", EXIT_INTERSECTION);
        }

        case EXIT_INTERSECTION:

            cEgoState::singleton->setYOLOState(true);
            CONSOLE_LOG_PROGRESS("EXIT_INTERSECTION: (%f > %d) target: ( %f | %f )", targetStrich.x, 0, targetStrich.x, targetStrich.y);

            if (targetStrich.x > 0.0f) {
                taskResult.f32steeringAngle = cEgoState::singleton->getRelativeLanePositioning();
                taskResult.f32speed = EXIT_SPEED;
                RETURN_NOERROR;
            }

            // as we would do lane detection as well in the next task, just finsh the task here.
            CONSOLE_LOG_INFO("FINISHED");
            m_taskState = taskState::finished;

        default: break;
    }


    RETURN_NOERROR;
}

tBool cTask_IntersectionHandling::canTakeOverControl() {
    if (!cOpenDriveMapAnalyzer::isInitialized() || !cEgoState::isInitialized()) return tFalse;

    tJunctionEntry *entry = cOpenDriveMapAnalyzer::singleton->inRangeOfJunctionEntry(getFrontAxlePos(cEgoState::singleton->getVehiclePosition()));
    if (entry == nullptr) return tFalse;

    tUInt32 numExitPoints = 0;
    if (!entry->leftTurn.empty()) numExitPoints++;
    if (!entry->rightTurn.empty()) numExitPoints++;
    if (!entry->straight.empty()) numExitPoints++;
    // this is not an intersection -> maybe its an merge scenario?
    if (numExitPoints <= 1) {
        LOG_WARNING("Found intersection but discarding because only one exit point was discovered. (Merge Scenario?)");
        return tFalse;
    }

    m_junctionEntry = *entry;
    if (m_state == SEARCHING_FOR_ENTRY) {
        m_state = INITIALIZE;
    }

    return tTrue;
}

/**
 * X = The direction the car is facing
 * Y = The perpendicular axis (sideways)
 * Z = 0
 * @param origin
 * @param pointToTransform
 * @return
 */
Point cTask_IntersectionHandling::transform(const tVehiclePosition &origin, const tVehiclePosition &pointToTransform) {

    tFloat32 dx = pointToTransform.f32x - origin.f32x;
    tFloat32 dy = pointToTransform.f32y - origin.f32y;
    //tFloat32 dPhi = target.f32heading - currentPos.f32heading;

    // coordinates relative to our current heading
    tFloat32 dxStrich = dx * cos(origin.f32heading) + dy * sin(origin.f32heading);
    tFloat32 dyStrich = dx * -sin(origin.f32heading) + dy * cos(origin.f32heading);

    return { dxStrich, dyStrich, 0.0f };
}



tBool cTask_IntersectionHandling::checkRightOfWay() {
    tBool coastIsClear = tFalse;

    switch (m_junctionEntry.roadSign) {
        case STOP:
        case GIVE_WAY:
            CONSOLE_LOG_INFO("In State CHECK_RIGHT_OF_WAY: STOP/GIVE_WAY");
            // Treat give way as stop sign
            switch (m_direction) {
                case aadc::jury::maneuver::maneuver_right:
                    coastIsClear = checkIntersectionLeft();
                    break;

                case aadc::jury::maneuver::maneuver_left:
                    coastIsClear = checkIntersectionRight()
                                   && checkIntersectionLeft()
                                   && checkIntersectionStraight();
                    break;

                default:
                    coastIsClear = checkIntersectionRight()
                                   && checkIntersectionLeft();
                    break;
            }
            break; // STOP / GIVE_WAY

        case RAKETE:
            CONSOLE_LOG_INFO("In State CHECK_RIGHT_OF_WAY: RAKETE");
            coastIsClear = tTrue;
            break; // RAKETE


        default:
            if (m_junctionEntry.roadSign == RECHTS_VOR_LINKS) {
                CONSOLE_LOG_INFO("In State CHECK_RIGHT_OF_WAY: RECHTS_VOR_LINKS");
            } else {
                CONSOLE_LOG_INFO("In State CHECK_RIGHT_OF_WAY: Found invalid sign -> interpreted as RECHTS_VOR_LINKS");
            }

            switch (m_direction) {
                case aadc::jury::maneuver::maneuver_right:
                    // In our case, drive anyway
                    coastIsClear = tTrue;
                    break;

                case aadc::jury::maneuver::maneuver_left:
                    coastIsClear = checkIntersectionRight()
                                   && checkIntersectionStraight();
                    break;
                default:
                    coastIsClear = checkIntersectionRight();
                    break;
            }

            break; //  RECHTS_VOR_LINKS
    }

    return coastIsClear;
}

tBool cTask_IntersectionHandling::checkIntersectionRight() {
    if (!cEgoState::isInitialized()) return tFalse;

    // If we don't have a right intersection, the right section is free
    if (m_junctionEntry.rightTurn.empty()) return tTrue;
    CONSOLE_LOG_INFO("Checking right ...");
    if (SIMULATE_OBSTACLE_RIGHT) return tFalse;

    auto laserScannerData = cEgoState::singleton->getLaserScannerData();

    for (const auto &polarCoordinate : laserScannerData) {
        if (polarCoordinate.f32Angle > 0 && polarCoordinate.f32Angle < 90) {
            Point p = toCartesian(polarCoordinate);

            if (p.y > ROAD_LANE_WIDTH && p.y < 2 * ROAD_LANE_WIDTH) {
                // point is on opponents lane

                if (p.x < MIN_DISTANCE_TO_OBSTACLE + ROAD_LANE_WIDTH / 2) {
                    CONSOLE_LOG_INFO("Right is NOT clear (%.2f < %.2f)\t| (%f,%f) (angle: %f, radius: %f).", p.x, MIN_DISTANCE_TO_OBSTACLE + ROAD_LANE_WIDTH / 2, p.x, p.y, polarCoordinate.f32Angle, polarCoordinate.f32Radius);
                    return tFalse;
                } else {
                    //CONSOLE_LOG_INFO("Ignoring obstacle at (%f,%f) (angle: %f, radius: %f) -- second if", p.x, p.y, polarCoordinate.f32Angle, polarCoordinate.f32Radius);
                }
            } else {
                //CONSOLE_LOG_INFO("Ignoring obstacle at (%f,%f) (angle: %f, radius: %f) -- first if", p.x, p.y, polarCoordinate.f32Angle, polarCoordinate.f32Radius);
            }
        }
    }
    CONSOLE_LOG_INFO("Right is clear.");

    return tTrue;
}

tBool cTask_IntersectionHandling::checkIntersectionStraight() {
    if (!cEgoState::isInitialized()) return tFalse;

    // If we don't have a straight intersection, the straight section is free
    if (m_junctionEntry.straight.empty()) return tTrue;
    CONSOLE_LOG_INFO("Checking straight ...");
    if (SIMULATE_OBSTACLE_STRAIGHT) return tFalse;

    auto laserScannerData = cEgoState::singleton->getLaserScannerData();

    for (const auto &polarCoordinate : laserScannerData) {
        if (polarCoordinate.f32Angle > 270 && polarCoordinate.f32Angle < 360) {
            Point p = toCartesian(polarCoordinate);

            if (p.x < -(ROAD_LANE_WIDTH / 2) && p.x > -(1.5 * ROAD_LANE_WIDTH)) {
                // point is on opponents lane

                if (p.y < MIN_DISTANCE_TO_OBSTACLE + 2 * ROAD_LANE_WIDTH) {
                    CONSOLE_LOG_INFO("Straight is NOT clear (%.2f < %.2f)\t| (%.2f,%.2f) (angle: %.2f, radius: %.2f).", p.y, MIN_DISTANCE_TO_OBSTACLE + 2 * ROAD_LANE_WIDTH, p.x, p.y, polarCoordinate.f32Angle, polarCoordinate.f32Radius);
                    return tFalse;
                } else {
                    //CONSOLE_LOG_INFO("Ignoring obstacle at (%.2f, %.2f) -- too far", p.x, p.y);
                }
            } else {
                //CONSOLE_LOG_INFO("Ignoring obstacle at (%.2f, %.2f) -- not on other lane", p.x, p.y);
            }
        }
    }
    CONSOLE_LOG_INFO("Straight is clear.");

    return tTrue;
}

tBool cTask_IntersectionHandling::checkIntersectionLeft() {
    if (!cEgoState::isInitialized()) return tFalse;

    // If we don't have a left intersection, the left section is free
    if (m_junctionEntry.leftTurn.empty()) return tTrue;
    CONSOLE_LOG_INFO("Checking left ...");
    if (SIMULATE_OBSTACLE_LEFT) return tFalse;

    auto laserScannerData = cEgoState::singleton->getLaserScannerData();

    for (const auto &polarCoordinate : laserScannerData) {
        if (polarCoordinate.f32Angle > 270 && polarCoordinate.f32Angle < 360) {
            Point p = toCartesian(polarCoordinate);

            if (p.y > 0 && p.y < ROAD_LANE_WIDTH) {
                // point is on opponents lane

                if (abs(p.x) < MIN_DISTANCE_TO_OBSTACLE + 1.5 * ROAD_LANE_WIDTH) {
                    CONSOLE_LOG_INFO("Left is NOT clear (%.2f < %.2f)\t| (%f,%f) (angle: %f, radius: %f).", abs(p.x), MIN_DISTANCE_TO_OBSTACLE + 1.5 * ROAD_LANE_WIDTH, p.x, p.y, polarCoordinate.f32Angle, polarCoordinate.f32Radius);
                    return tFalse;
                } else {
                    //CONSOLE_LOG_INFO("Ignoring obstacle at (%f,%f) (angle: %f, radius: %f) -- second if", p.x, p.y, polarCoordinate.f32Angle, polarCoordinate.f32Radius);
                }
            } else {
                //CONSOLE_LOG_INFO("Ignoring obstacle at (%f,%f) (angle: %f, radius: %f) -- first if", p.x, p.y, polarCoordinate.f32Angle, polarCoordinate.f32Radius);
            }
        }
    }
    CONSOLE_LOG_INFO("Left is clear.");

    return tTrue;
}


Point cTask_IntersectionHandling::toCartesian(const tPolarCoordiante &polar) {
    Point cartesian = {};

    cartesian.x = sin(DEG_TO_RAD(polar.f32Angle)) * (polar.f32Radius / 1000.0f);
    cartesian.y = cos(DEG_TO_RAD(polar.f32Angle)) * (polar.f32Radius / 1000.0f);

    return cartesian;
}

tVehiclePosition cTask_IntersectionHandling::getLaserScannerPos(tVehiclePosition origin) {

    origin.f32x += LASER_SCANNER_OFFSET * cos(origin.f32heading);
    origin.f32y += LASER_SCANNER_OFFSET * sin(origin.f32heading);
    origin.f32heading = normalizeAngle(origin.f32heading);

    return origin;
}

tVehiclePosition cTask_IntersectionHandling::getFrontAxlePos(tVehiclePosition origin) {

    origin.f32x += FRONT_AXLE_OFFSET * cos(origin.f32heading);
    origin.f32y += FRONT_AXLE_OFFSET * sin(origin.f32heading);
    origin.f32heading = normalizeAngle(origin.f32heading);

    return origin;
}

tFloat32 cTask_IntersectionHandling::normalizeAngle(tFloat32 angle) {
    return tFloat32(fmod(angle + 2.0f * M_PI, 2.0f * M_PI));
}


/**
 * Returns the relative angle of an angle 'ofAngle' in respect to a fixed angle 'toAngle'.
 *
 * @param ofAngle the absolute angle which should get 'relative'
 * @param toAngle the new absolute 'origin' angle -> (toAngle, toAngle) would yield 0
 * @return relative angle [-179.9...180]
 */
tFloat32 cTask_IntersectionHandling::getRelativeAngle(tFloat32 ofAngle, tFloat32 toAngle) {
    tFloat32 of = normalizeAngle(ofAngle);
    tFloat32 to = normalizeAngle(toAngle);

    tFloat32 rel = of - to;
    if (rel > 180.0f * M_PI / 180.0f) {
        rel -= 2.0f * M_PI;
    } else if (rel <= -180.0f * M_PI / 180.0f) {
        rel += 2.0f * M_PI;
    }

    return rel;
}

