#include "Task_EmergencyCar.h"
#include "../../Helpers/cariosity_structs.h"

#define LANE_DETECTION_MIN_CONFIDENCE_THRESHOLD (9)

cTask_EmergencyCar::cTask_EmergencyCar(tBool firstOfManeuver) : cTask(firstOfManeuver)
{
    LOG_INFO("Task for emergency car taking over control");
    state = DETECTED;
}

tResult cTask_EmergencyCar::execute(tTaskResult &taskResult)
{
    taskResult.lights.bHeadLight = tTrue;

    switch(state) {
        case DETECTED:
            LOG_INFO("Started task -- stopping");

            state = DRIVE_RIGHT;
            taskResult.lights.bBrakeLight = tTrue;
            taskResult.f32speed = 0.0f;
            taskResult.f32steeringAngle = 0.0f;
            taskResult.bEmergencyBrake = tTrue;
            break;

        case DRIVE_RIGHT:
            LOG_INFO("Driving right");
            taskResult.lights.bTurnSignalRight = tTrue;
            taskResult.lights.bBrakeLight = tTrue;

            if(targetDistance == 0.0f) {
                targetDistance = cEgoState::singleton->getTotalDistance() + 0.3f;
            }

            taskResult.f32speed = 0.3f;
            taskResult.f32steeringAngle = 100.0f;

            if (targetDistance > cEgoState::singleton->getTotalDistance()) {
                break;
            }
            targetDistance = 0.0f; //reset
            state = PULL_STRAIGHT;
            //fall thru intended

        case PULL_STRAIGHT:
            LOG_INFO("Driving left");
            taskResult.lights.bTurnSignalRight = tTrue;
            taskResult.lights.bBrakeLight = tTrue;


            if(targetDistance == 0.0f) {
                targetDistance = cEgoState::singleton->getTotalDistance() + 0.3f;
            }
            taskResult.f32speed = 0.3f;
            taskResult.f32steeringAngle = -100.0f;

            if (targetDistance > cEgoState::singleton->getTotalDistance()) {
                break;
            }

            state = STOP;
            //fall thru intended
        case STOP:
            LOG_INFO("stopping");

            taskResult.bEmergencyBrake = tTrue;
            //wait atleast 1s
            taskResult.lights.bBrakeLight = tTrue;
            taskResult.f32speed = 0.0f;
            taskResult.f32steeringAngle = 0.0f;

            if (!cEgoState::singleton->isEmergencyCarPresent()) {
                if (m_firstStop == 0 ) {
                    m_firstStop = cEgoState::singleton->getLastUpdated();
                } else if (tTimeStamp(m_firstStop + tTimeStamp(2 * 1e6)) <= cEgoState::singleton->getLastUpdated()) {
                    LOG_INFO("Haven't seen the cops, should be safe to continue now");

                    state = FINDWAYBACKONTOTHESTREET;
                }
            } else { //reset when the cops are seen again, start waiting again.
                m_firstStop = 0;
            }
            break;

        case FINDWAYBACKONTOTHESTREET:
            taskResult.lights.bTurnSignalLeft = tTrue;

            cEgoState::singleton->setLaneDetectionAngleOfFieldOfView(0.0);
            cEgoState::singleton->setOverrideLaneDetectionAngleOfFielOfView(tTrue);

            taskResult.f32speed = 0.3f;
            taskResult.f32steeringAngle = -50.0f;

            if(cEgoState::singleton->getLaneDetectionConfidence() > LANE_DETECTION_MIN_CONFIDENCE_THRESHOLD ) {
                taskResult.f32speed = 0.5f;
                taskResult.f32steeringAngle = cEgoState::singleton->getRelativeLanePositioning();
                cEgoState::singleton->setOverrideLaneDetectionAngleOfFielOfView(tFalse);
                m_taskState = fhw::taskState::finished;
            }
            break;
        default:
            break;
    }

    RETURN_NOERROR;
}
