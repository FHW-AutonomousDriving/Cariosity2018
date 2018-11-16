#include "Task.h"
#include "../../Helpers/cariosity_structs.h"

cTask::cTask(tBool isFirstTaskOfManeuver) {
    m_isFirstTaskOfManeuver = isFirstTaskOfManeuver;
}

tBool cTask::isFirstTaskOfManeuver() {
    return m_isFirstTaskOfManeuver;
}

fhw::taskState cTask::getTaskState() {
    return m_taskState;
}

void cTask::setTaskState(fhw::taskState theTaskState) {
    m_taskState = theTaskState;
}

tBool cTask::isInterruptable() {
    return tFalse;
}

tBool cTask::canTakeOverControl() {
    return tFalse;
}

tResult cTask::execute(tBool emergencyBrakeFlag, tTaskResult &taskResult) {
    if (emergencyBrakeFlag) {
        taskResult.bEmergencyBrake = tTrue;
        taskResult.lights.bHazardLight = tTrue;
        taskResult.f32steeringAngle = cEgoState::singleton->getRelativeLanePositioning();
        RETURN_NOERROR;
    } else {
        return execute(taskResult);
    }
}

tResult cTask::execute(tTaskResult &taskResult) {
    RETURN_NOERROR;
}
