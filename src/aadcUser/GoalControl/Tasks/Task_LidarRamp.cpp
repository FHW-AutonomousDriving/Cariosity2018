#include "Task_LidarRamp.h"
#include "../../Helpers/cariosity_structs.h"
#include "stdafx.h"


#define ROAD_LANE_WIDTH (500.0f)
#define ANGLE_TOLLERANCE (5.0f)
#define TEST_ANGLE (90.0f)


/*! Calls parent constructor with default values */
cTask_LidarRamp::cTask_LidarRamp(tBool isFirstTaskOfManeuver) : cTask(isFirstTaskOfManeuver) {}

tResult cTask_LidarRamp::execute(tBool emergencyBrakeFlag, tTaskResult &taskResult) {
    cEgoState::singleton->setYOLOState(tFalse);
    return execute(taskResult);
}


tResult cTask_LidarRamp::execute(tTaskResult &taskResult) {

    tPolarCoordiante coordinate = findCoordinate();

    if (coordinate.f32Radius > 650.0f) {
        m_taskState = fhw::taskState::finished;
        LOG_INFO("Leaving Ramp Task: %f", coordinate.f32Radius);
        cEgoState::singleton->setYOLOState(tTrue);
        RETURN_NOERROR;
    }

    tFloat32 targetDistance = ROAD_LANE_WIDTH / 2;

    taskResult.f32speed = 0.3f;
    taskResult.f32steeringAngle = coordinate.f32Radius - targetDistance;
    LOG_INFO("Dist %f\tTarget %f\tSteering %f", coordinate.f32Radius, targetDistance, taskResult.f32steeringAngle);

    RETURN_NOERROR;
}

tBool cTask_LidarRamp::canTakeOverControl() {
    tPolarCoordiante coordinate = findCoordinate();
    tBool result = coordinate.f32Radius < ROAD_LANE_WIDTH && (coordinate.f32Angle < TEST_ANGLE + ANGLE_TOLLERANCE) && (coordinate.f32Angle > TEST_ANGLE - ANGLE_TOLLERANCE);
    if (result) {
        LOG_INFO("Entering Ramp Task %f", coordinate.f32Radius);

    }
    return result;
}

tPolarCoordiante cTask_LidarRamp::findCoordinate() {
    tPolarCoordiante bestCandidate;
    tFloat32 minAngleError = FLT_MAX;
    for (tPolarCoordiante coordinate : cEgoState::singleton->getLaserScannerData()) {
        tFloat32 angleError = fabs(TEST_ANGLE - coordinate.f32Angle);
        if (angleError < minAngleError) {
            minAngleError = angleError;
            bestCandidate = coordinate;
        }
    }
    return bestCandidate;
}
