#include "Task_MergeScenario.h"
#include "../../Helpers/cariosity_structs.h"
#include <aadc_geometrics.h>


#define RADIUS_AT_FULL_LOCK (0.36f)//(0.63f)
#define ROAD_LANE_WIDTH (0.5f)
#define MIN_DISTANCE_TO_OBSTACLE (40.0f)

#define FRONT_AXLE_OFFSET (0.365f)
#define LASER_SCANNER_OFFSET (0.45f)

/*! Calls parent constructor with default values */
cTask_MergeScenario::cTask_MergeScenario(tBool isFirstTaskOfManeuver, aadc::jury::maneuver direction) : cTask(isFirstTaskOfManeuver), m_direction(direction) {}

tResult cTask_MergeScenario::execute(tTaskResult &taskResult) {
    if (m_state == SEARCHING_FOR_ENTRY) {
        LOG_WARNING("Searching for intersection entry!");
        if(!canTakeOverControl()) {
            taskResult.f32steeringAngle = cEgoState::singleton->getRelativeLanePositioning();
            taskResult.f32speed = 0.2f;
            RETURN_NOERROR;
        }
    }

    tVehiclePosition frontAxlePos = getFrontAxlePos(cEgoState::singleton->getVehiclePosition());

    if (m_state == INITIALIZE) {
        LOG_INFO("Initializing Intersection Handling!");
        vector<Pose3D> poses;

        switch (m_direction) {
            case aadc::jury::maneuver::maneuver_merge_left:
                LOG_INFO("MERGING LEFT");
                poses = m_junctionEntry.leftTurn.empty() ? m_junctionEntry.straight : m_junctionEntry.leftTurn;
                break;
            default: break;
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
        m_intermediateHeading = tFloat32(fmod(frontAxlePos.f32heading + M_PI * 10.0f / 180.0f + 2.0f * M_PI, 2.0f * M_PI));
        m_state = CHECK_RIGHT_OF_WAY;
    }

    // PROCESSING
    // Headlights
    taskResult.lights.bHeadLight = tTrue;

    // Turn indicator
    switch (m_direction) {
        case aadc::jury::maneuver::maneuver_merge_right:
            taskResult.lights.bTurnSignalRight = tTrue;
            break;
        case aadc::jury::maneuver::maneuver_merge_left:
            taskResult.lights.bTurnSignalLeft = tTrue;
            break;
        default: break;
    }

    // CHECK_RIGHT_OF_WAY
    if (m_state == CHECK_RIGHT_OF_WAY) {

        tBool coastIsClear = tTrue;// TODO: Check left ultrasonic?



        if (coastIsClear) {
            m_state = ENTER_MERGE;
            LOG_INFO("In State CHECK_RIGHT_OF_WAY: Coast is clear. Proceeding with MERGE ...");
        } else {
            LOG_INFO("In State CHECK_RIGHT_OF_WAY: Coast is NOT clear. Waiting ...");
            RETURN_NOERROR;
        }
    }

    Point targetStrich = transform(frontAxlePos, m_target);

    LOG_INFO(
            "state: %d || target heading %f || current heading %f ||  ( %f | %f )",
             m_state, m_target.f32heading, frontAxlePos.f32heading, targetStrich.x, targetStrich.y
    );

    switch (m_state) {
        case ENTER_MERGE:

            LOG_INFO("ENTER_MERGE: turning (%f < %f)  target: ( %f | %f )", frontAxlePos.f32heading, m_intermediateHeading, targetStrich.x, targetStrich.y);

            // set target heading to be 90 degrees offset of current heading


            if (frontAxlePos.f32heading < m_intermediateHeading) {
                taskResult.f32steeringAngle = -100.0f;
                taskResult.f32speed = 0.5f;
                RETURN_NOERROR;
            }

            m_state = ON_MERGE;
            //fallthrough intended!
            LOG_INFO("ENTERING State ON_MERGE (%d)", ON_MERGE);

        case ON_MERGE:
            LOG_INFO("ON_MERGE: straight (%f > %f)  target: ( %f | %f )", targetStrich.x, 10.0f, targetStrich.x, targetStrich.y);

            if (targetStrich.x > 0.2f) {
                taskResult.f32steeringAngle = 0.0f;
                taskResult.f32speed = 0.5f;
                RETURN_NOERROR;
            }

            m_state = EXIT_MERGE;
            //fallthrough intended!
            LOG_INFO("ENTERING State EXIT_MERGE (%d)", EXIT_MERGE);

        case EXIT_MERGE:
            LOG_INFO("EXIT_MERGE: turning (%f < %f)  target: ( %f | %f )", frontAxlePos.f32heading, m_target.f32heading, targetStrich.x, targetStrich.y);

            if (getRelativeAngle(frontAxlePos.f32heading, m_target.f32heading) > 0) {
                taskResult.f32steeringAngle = 100.0f;
                taskResult.f32speed = 0.5f;
                RETURN_NOERROR;
            }

            // as we would do lane detection as well in the next task, just finsh the task here.
            LOG_INFO("FINISHED");
            m_taskState = taskState::finished;

        default: break;
    }


    RETURN_NOERROR;
}

tBool cTask_MergeScenario::canTakeOverControl() {
    if (!cOpenDriveMapAnalyzer::isInitialized()) return tFalse;

    tJunctionEntry *entry = cOpenDriveMapAnalyzer::singleton->inRangeOfJunctionEntry(getFrontAxlePos(cEgoState::singleton->getVehiclePosition()));
    if (entry == nullptr) return tFalse;

    m_junctionEntry = *entry;
    if (m_state == SEARCHING_FOR_ENTRY) {
        m_state = INITIALIZE;
    }

    return tTrue;
}

Point cTask_MergeScenario::transform(const tVehiclePosition &origin, const tVehiclePosition &pointToTransform) {

    tFloat32 dx = pointToTransform.f32x - origin.f32x;
    tFloat32 dy = pointToTransform.f32y - origin.f32y;
    //tFloat32 dPhi = target.f32heading - currentPos.f32heading;

    // coordinates relative to our current heading
    tFloat32 dxStrich = dx * cos(origin.f32heading) + dy * sin(origin.f32heading);
    tFloat32 dyStrich = dx * -sin(origin.f32heading) + dy * cos(origin.f32heading);

    return { dxStrich, dyStrich, 0.0f };
}

tVehiclePosition cTask_MergeScenario::getFrontAxlePos(tVehiclePosition origin) {

    origin.f32x += FRONT_AXLE_OFFSET * cos(origin.f32heading);
    origin.f32y += FRONT_AXLE_OFFSET * sin(origin.f32heading);

    return origin;
}

tFloat32 cTask_MergeScenario::normalizeAngle(tFloat32 angle) {
    return tFloat32(fmod(angle + 2.0f * M_PI, 2.0f * M_PI));
}

/**
* Returns the relative angle of an angle 'ofAngle' in respect to a fixed angle 'toAngle'.
*
* @param ofAngle the absolute angle which should get 'relative'
* @param toAngle the new absolute 'origin' angle -> (toAngle, toAngle) would yield 0
* @return relative angle [-179.9...180]
*/
tFloat32 cTask_MergeScenario::getRelativeAngle(tFloat32 ofAngle, tFloat32 toAngle) {
    tFloat32 of = normalizeAngle(ofAngle);
    tFloat32 to = normalizeAngle(toAngle);

    tFloat32 rel = of - to;
    if (rel > 180.0f * M_PI / 180.0f) {
        rel -= 2.0f * M_PI;
    }
    else if (rel <= -180.0f * M_PI / 180.0f) {
        rel += 2.0f * M_PI;
    }

    return rel;
}