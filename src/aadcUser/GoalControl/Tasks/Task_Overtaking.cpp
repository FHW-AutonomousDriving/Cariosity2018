#include "stdafx.h"
#include "Task_Overtaking.h"
#include "../../Helpers/cariosity_structs.h"

#define DEG_TO_RAD(x) (tFloat32(((x) * M_PI / 180.0)))
#define RAD_TO_DEG(x) (tFloat32(((x) / M_PI * 180.0)))


#define CONSOLE_LOG_INFO(...) if (m_enableConsoleOutput) LOG_ADD_ENTRY(A_UTILS_NS::log::tLogLevel::Info, __VA_ARGS__)
#define CONSOLE_LOG_STATE_CHANGE(...) if (m_enableStateChangeOutput) LOG_ADD_ENTRY(A_UTILS_NS::log::tLogLevel::Info, __VA_ARGS__)



/*! Calls parent constructor with default values: is never first of maneuver (extra one) */
cTask_Overtaking::cTask_Overtaking(tBool isFirstTaskOfManeuver) : cTask(tFalse) {
    CONSOLE_LOG_INFO("Constructor of Task_Overtaking");
    m_state = INIT;
}

tResult cTask_Overtaking::execute(tBool emergencyBrakeFlag, tTaskResult &taskResult) {

    // TODO: react to emergency vehicle inmid overtake?

    if (!cEgoState::isInitialized()) return cResult(ERR_NOT_INITIALIZED);

    if (emergencyBrakeFlag
        && m_state != eState::INIT
        && m_state != eState::DRIVING_BACKWARDS) {
        CONSOLE_LOG_INFO("Emergency Brake but not INIT or BACKWARDS");

        // not on left lane yet? (not managed overtaking) -> retry!
        if(m_state == SWITCHING_TO_LEFT_LANE || m_state == OVERTAKING_ON_LEFT_LANE) {
            CONSOLE_LOG_INFO("Did not manage lane switch -- restarting from INIT state");
            m_state = INIT;
            RETURN_NOERROR;
        }

        taskResult.bEmergencyBrake = tTrue;
        taskResult.lights.bHazardLight = tTrue;
        taskResult.f32steeringAngle = cEgoState::singleton->getRelativeLanePositioning();
        RETURN_NOERROR;
    }

    static tFloat32 initialDistance;
    static tTimeStamp timeOfStateStart;

    switch(m_state) {
        case eState::INIT : {
            //TODO: check that the situation is clear, lane is straight, etc.

            tUltrasonicStruct lUSS = cEgoState::singleton->getUltrasonicStruct();
            if (lUSS.tRearCenter.f32Value < 50) {
                CONSOLE_LOG_INFO("Obstacle behind within 50cm! Remaining in place.");
                break; // not yet done
            }
            // initialisation done, start driving backwards
            initialDistance = cEgoState::singleton->getTotalDistance();
            //timeOfStateStart = cEgoState::singleton->getLastUpdated();

            m_state = DRIVING_BACKWARDS;
            CONSOLE_LOG_STATE_CHANGE("Task_Overtaking -- DRIVING_BACKWARDS");
        }

        case eState::DRIVING_BACKWARDS : {
            cEgoState::singleton->setYOLOState(false);

            CONSOLE_LOG_INFO("Backwards distance: %f", (cEgoState::singleton->getTotalDistance() - initialDistance));
            //CONSOLE_LOG_INFO("Backwards driving time: %f", ((timeOfStateStart + tTimeStamp(m_timeDrivingBackwards * 1e6))/1e6));

            tUltrasonicStruct lUSS = cEgoState::singleton->getUltrasonicStruct();
            if (lUSS.tRearCenter.f32Value < 50) {
                CONSOLE_LOG_INFO("Obstacle behind within 50cm! Remaining in place.");
                break; // not yet done
            }

            if((cEgoState::singleton->getTotalDistance() - initialDistance) < m_targetDistanceBackwards) {
            // if((timeOfStateStart + tTimeStamp(m_timeDrivingBackwards * 1e6)) > cEgoState::singleton->getLastUpdated()) {
                // still some more backwards driving to do for target (in distance or time)

                taskResult.f32steeringAngle = 0.0f; //-cEgoState::singleton->getRelativeLanePositioning();
                taskResult.f32speed = -0.2f;

                //TODO: already enable left turn signal?
                taskResult.lights.bHeadLight = tTrue;
                taskResult.lights.bReverseLight = tTrue;
                break; // not yet done
            }

            // backwards driving is done, start switching to left lane
            timeOfStateStart = cEgoState::singleton->getLastUpdated();

            m_state = STOP;
            CONSOLE_LOG_STATE_CHANGE("Task_Overtaking -- STOP before forwards");
        }

        case eState::STOP : {
            cEgoState::singleton->setYOLOState(true);
            taskResult.lights.bHeadLight = tTrue;

            if(cEgoState::singleton->isEmergencyCarPresent()){
                timeOfStateStart = cEgoState::singleton->getLastUpdated();
            }

            if((cEgoState::singleton->getTotalDistance() - initialDistance) > 0.01f) {
                // sensor says still moving
                taskResult.bEmergencyBrake = tTrue;
                taskResult.f32speed = 0.0f;
                initialDistance = cEgoState::singleton->getTotalDistance();
                timeOfStateStart = cEgoState::singleton->getLastUpdated();
                break;
            }

            if((timeOfStateStart + tTimeStamp(2 * 1e6)) > cEgoState::singleton->getLastUpdated()) {
                // not yet two seconds standing: move on
                break;
            }

            initialDistance = cEgoState::singleton->getTotalDistance();
            m_state = SWITCHING_TO_LEFT_LANE;
            CONSOLE_LOG_STATE_CHANGE("Task_Overtaking -- SWITCHING to left lane");
        }

        case eState::SWITCHING_TO_LEFT_LANE : {
            cEgoState::singleton->setYOLOState(false);

            // steering angle hard left for first 5 cm
            if((cEgoState::singleton->getTotalDistance() - initialDistance) < (0.3f)) {
                taskResult.f32speed = 0.3f;
                taskResult.f32steeringAngle = -80.0f;
                taskResult.lights.bHeadLight = tTrue;
                taskResult.lights.bTurnSignalLeft = tTrue;
                break; // not yet done
            }
            // while high probability of lane to the left: steer left
            if(cEgoState::singleton->getLanes().fHasLaneToTheLeft > 65) {
                taskResult.f32speed = 0.3f;
                taskResult.f32steeringAngle = -50.0f;
                taskResult.lights.bHeadLight = tTrue;
                taskResult.lights.bTurnSignalLeft = tTrue;
                break; // not yet done
            }

            if((cEgoState::singleton->getTotalDistance() - initialDistance) < (0.4f)) {
                taskResult.f32speed = 0.3f;
                taskResult.f32steeringAngle = cEgoState::singleton->getRelativeLanePositioning();
                taskResult.lights.bHeadLight = tTrue;
                taskResult.lights.bTurnSignalLeft = tTrue;
                break; // not yet done
            }

            // switching to left lane is done, start overtaking
            initialDistance = cEgoState::singleton->getTotalDistance();
            //timeOfStateStart = cEgoState::singleton->getLastUpdated();

            m_state = OVERTAKING_ON_LEFT_LANE;
            CONSOLE_LOG_STATE_CHANGE("Task_Overtaking -- OVERTAKING on left lane");
        }

        case eState::OVERTAKING_ON_LEFT_LANE : {
            cEgoState::singleton->setYOLOState(false);

            tUltrasonicStruct lUSS = cEgoState::singleton->getUltrasonicStruct();
            tBool lDetectedObstacleToTheRight = lUSS.tSideRight.f32Value < 10
                                            || lUSS.tRearRight.f32Value < 10;

            tBool fDroveCarMinLength = (cEgoState::singleton->getTotalDistance() - initialDistance) > (tFloat32(CAR_HEIGHT)*2.0f/100.0f);

            CONSOLE_LOG_INFO("Overtaking distance -- drove length: %f;  detectedObstacleToRight: %b", (cEgoState::singleton->getTotalDistance() - initialDistance), lDetectedObstacleToTheRight);

            // driving straight on left lane if obstacle to the right or not yet driven car length
            if(lDetectedObstacleToTheRight
               || !fDroveCarMinLength) {
                taskResult.f32speed = 0.3f;
                taskResult.f32steeringAngle = cEgoState::singleton->getRelativeLanePositioning();
                taskResult.lights.bHeadLight = tTrue;
                break; // not yet done
            }

            // overtaking on left lane is done, start switching back to right lane
            initialDistance = cEgoState::singleton->getTotalDistance();
            //timeOfStateStart = cEgoState::singleton->getLastUpdated();

            //m_state = SWITCHING_TO_RIGHT_LANE;
            //CONSOLE_LOG_STATE_CHANGE("Task_Overtaking -- SWITCHING to right lane");

            // immediately switch to state DONE: let laneswitch handle getting back to right lane!
            m_state = DONE;
            CONSOLE_LOG_STATE_CHANGE("Task_Overtaking -- DONE");
        }
/*
        case eState::SWITCHING_TO_RIGHT_LANE : {
            cEgoState::singleton->setYOLOState(false);

            if(cEgoState::singleton->getLanes().fHasLaneToTheRight > 65) {
                taskResult.f32speed = 0.3f;
                taskResult.f32steeringAngle = 50.0f;
                taskResult.lights.bHeadLight = tTrue;
                taskResult.lights.bTurnSignalLeft = tTrue;

                cEgoState::singleton->setLaneDetectionAngleOfFieldOfView(0.0f);
                cEgoState::singleton->setOverrideLaneDetectionAngleOfFielOfView(tTrue);

                break; // not yet done
            }

            if((cEgoState::singleton->getTotalDistance() - initialDistance) < 0.5f) {
                // if((timeOfStateStart + tTimeStamp(m_timeLaneSwitch * 1e6)) > cEgoState::singleton->getLastUpdated()) {
                // still some more driving to do for target distance over to left lane

                taskResult.f32speed = 0.3f;
                taskResult.f32steeringAngle = -70.0f;
                taskResult.lights.bHeadLight = tTrue;

                cEgoState::singleton->setLaneDetectionAngleOfFieldOfView(0.0f);
                cEgoState::singleton->setOverrideLaneDetectionAngleOfFielOfView(tTrue);

                break; // not yet done
            }

            m_state = DONE;
            CONSOLE_LOG_STATE_CHANGE("Task_Overtaking -- DONE");
        }
*/
        case eState::DONE : {
            cEgoState::singleton->setYOLOState(true);
            cEgoState::singleton->setOverrideLaneDetectionAngleOfFielOfView(tFalse);

            m_taskState = fhw::taskState::finished;
            break;
        }

        default :
            LOG_ERROR("Task_Overtaking: Entered non-existent state -- something went massively wrong. Go hunt down that bug!");
            m_taskState = fhw::taskState::error;
            break;
    }

    RETURN_NOERROR;
}