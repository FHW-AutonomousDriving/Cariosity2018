//
// Created by aadc on 20.09.18.
//

#include "stdafx.h"
#include "Task_LaneFollowing.h"
#include "../../Helpers/cariosity_structs.h"

#define BASE_SPEED (0.5f)
#define MAX_SPEED (0.8f)
#define LANE_DETECTION_MIN_CONFIDENCE_THRESHOLD (10)
#define LANE_DETECTION_MAX_CONFIDENCE_THRESHOLD (18)

#define LANE_DETECTION_MAX_STEERING_ANGLE (20.0f)


#define CONSOLE_LOG_INFO(...) if (m_enableConsoleOutput) LOG_ADD_ENTRY(A_UTILS_NS::log::tLogLevel::Info, __VA_ARGS__)



cTask_LaneFollowing::cTask_LaneFollowing(tBool isFirstTaskOfManeuver) : cTask(isFirstTaskOfManeuver) {}

tBool cTask_LaneFollowing::isInterruptable() {
    return tTrue;
}

tResult cTask_LaneFollowing::execute(tTaskResult &taskResult) {

    taskResult.lights.bHeadLight = tTrue;
    taskResult.f32speed = BASE_SPEED;

// ------ steering angle adjusting based on sidelane confidence; possible laneswitch

    tFloat32 leftLaneConf = cEgoState::singleton->getLanes().fHasLaneToTheLeft;
    tFloat32 rightLaneConf = cEgoState::singleton->getLanes().fHasLaneToTheRight;

    if(leftLaneConf < 65) { // links keine Spur

        if(rightLaneConf >= 80.0f) {
            cEgoState::singleton->setLaneDetectionAngleOfFieldOfView(0.0f);
            cEgoState::singleton->setOverrideLaneDetectionAngleOfFielOfView(tTrue);

            taskResult.f32steeringAngle = putValueInRange(cEgoState::singleton->getRelativeLanePositioning() + 60.0f, 0.0f, 100.0f);
            CONSOLE_LOG_INFO("Added +60 percent to steering angle: --> %f", taskResult.f32steeringAngle);
            taskResult.lights.bTurnSignalRight = tTrue;
        }
        else if(rightLaneConf >= 60.0f) {
            cEgoState::singleton->setLaneDetectionAngleOfFieldOfView(0.0f);
            cEgoState::singleton->setOverrideLaneDetectionAngleOfFielOfView(tTrue);

            taskResult.f32steeringAngle = putValueInRange(cEgoState::singleton->getRelativeLanePositioning() + 20.0f, 0.0f, 100.0f);
            CONSOLE_LOG_INFO("Added +20 percent to steering angle: --> %f", taskResult.f32steeringAngle);
            taskResult.lights.bTurnSignalRight = tTrue;
        }
        else {
            cEgoState::singleton->setOverrideLaneDetectionAngleOfFielOfView(tFalse);
            taskResult.f32steeringAngle = cEgoState::singleton->getRelativeLanePositioning();
        }
    }
    else {
        cEgoState::singleton->setOverrideLaneDetectionAngleOfFielOfView(tFalse);
        taskResult.f32steeringAngle = cEgoState::singleton->getRelativeLanePositioning();

    }

// ----- speed adjustements

    //TODO: use future steering angle for weighting so we can in die kurve abbremsen and aus der kurve herausbeschleunigen
    tFloat32 confidence = tFloat32(cEgoState::singleton->getLaneDetectionConfidence());
    if (abs(taskResult.f32steeringAngle) <= LANE_DETECTION_MAX_STEERING_ANGLE && confidence >= LANE_DETECTION_MIN_CONFIDENCE_THRESHOLD) {

        tFloat32 curvatureFactor = (confidence - LANE_DETECTION_MIN_CONFIDENCE_THRESHOLD) / (LANE_DETECTION_MAX_CONFIDENCE_THRESHOLD - LANE_DETECTION_MIN_CONFIDENCE_THRESHOLD);
        tFloat32 steeringAngleFactor = (LANE_DETECTION_MAX_STEERING_ANGLE - abs(taskResult.f32steeringAngle)) / LANE_DETECTION_MAX_STEERING_ANGLE;
        taskResult.f32speed = BASE_SPEED + (MAX_SPEED - BASE_SPEED) * min(steeringAngleFactor * curvatureFactor, 1.0f);
    } else {
        taskResult.f32speed = BASE_SPEED;
    }

    if(!cEgoState::singleton->getChilds().empty()) {
        CONSOLE_LOG_INFO("Found child: Going slow!");

        taskResult.lights.bBrakeLight = tTrue;
    }

    RETURN_NOERROR;
}

tFloat32 cTask_LaneFollowing::putValueInRange(tFloat32 value, tFloat32 min, tFloat32 max) {
    return fmax(min, fmin(max, value));
}
