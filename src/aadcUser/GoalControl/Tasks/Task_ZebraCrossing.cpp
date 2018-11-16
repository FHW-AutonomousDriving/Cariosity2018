#include "Task_ZebraCrossing.h"
#include "../../Helpers/cariosity_structs.h"

cTask_ZebraCrossing::cTask_ZebraCrossing(tBool firstOfManeuver) : cTask(firstOfManeuver)
{
    LOG_INFO("Zebra detected, task taking over control");
    state = DETECTED;
}

tResult cTask_ZebraCrossing::execute(tTaskResult &taskResult) {

    taskResult.f32steeringAngle = cEgoState::singleton->getRelativeLanePositioning();

    detectedPersonOnce = detectedPersonOnce || !cEgoState::singleton->getPersons().empty() || !cEgoState::singleton->getChilds().empty();

    switch(state){
        case DETECTED:
            taskResult.lights.bBrakeLight = tTrue;
            taskResult.f32speed = 0.2f;
            if(detectedPersonOnce && m_firstStop == 0) {
                LOG_INFO("Saw person");
                state = STOP; //fall thru intended
            } else {
                break;
            }
        case STOP:
            taskResult.bEmergencyBrake = tTrue;
            taskResult.f32speed = 0.0f;
            if (m_firstStop == 0) {
                m_firstStop = cEgoState::singleton->getLastUpdated();
            } else if (tTimeStamp(m_firstStop + tTimeStamp(6 * 1e6)) <= cEgoState::singleton->getLastUpdated()) {
                LOG_INFO("Zebra waiting time over");
                taskResult.lights.bBrakeLight = tFalse;
                state = DETECTED;
            }
            break;
        case DONE:
            LOG_INFO("Zebra handled, finished.");
            m_taskState = fhw::taskState::finished;
            break;

    }

    RETURN_NOERROR;
}
