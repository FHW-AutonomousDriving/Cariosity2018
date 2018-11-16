//
// Created by aadc on 20.09.18.
//

#include "Task_PullOutLeft.h"
#include "../../Helpers/cariosity_structs.h"
#include "stdafx.h"

#define LANE_DETECTION_MIN_CONFIDENCE_THRESHOLD (9)
#define DEGREES_TO_TURN (75)
#define LANEDETECTION_OVERTAKE_ANGLE (15 * M_PI / 180)

cTask_PullOutLeft::cTask_PullOutLeft(tBool isFirstTaskOfManeuver) : cTask( isFirstTaskOfManeuver) {
    state = INIT;
}

tResult cTask_PullOutLeft::execute(tBool emergencyBrakeFlag, tTaskResult &taskResult) {
    cEgoState::singleton->setYOLOState(tFalse);
    return execute(taskResult);
}

tResult cTask_PullOutLeft::execute(tTaskResult &taskResult) {
    cEgoState::singleton->setYOLOState(tFalse);
    LOG_INFO("Total Distance %f\t Target %f\t Heading %f\t Target %f",
             cEgoState::singleton->getTotalDistance(), targetDistance, cEgoState::singleton->getVehiclePosition().f32heading, targetHeading);
    taskResult.lights.bHeadLight = tTrue;

    if (cEgoState::singleton->getVehiclePosition().f32heading == 0.0f) {
        RETURN_NOERROR;
    }

    switch (state) {
        case INIT: {
            LOG_INFO("INIT");
            auto nextParking = nextParkingSpot();
            targetDistance = cEgoState::singleton->getTotalDistance() + nextParking.first - 0.05f;
            targetHeading = normalizeAngle((cEgoState::singleton->getParkingSpaces().at(nextParking.second).f32Direction * M_PI / 180) + DEGREES_TO_TURN * M_PI / 180);
            state = STRAIGHT;
            // No break because we now want to start driving
        }

        case STRAIGHT:
            LOG_INFO("STRAIGHT");
            if (targetDistance > cEgoState::singleton->getTotalDistance()) {

                taskResult.f32steeringAngle = 0.0f;
                taskResult.lights.bTurnSignalLeft = tTrue;
                taskResult.f32speed = 0.3f;
                break;
            }
            state = STEERING;
            // We are done going straight, we now want to give in

        case STEERING: {
            LOG_INFO("STEERING");
            tFloat64 remainingHeading = getRelativeAngle(cEgoState::singleton->getVehiclePosition().f32heading, targetHeading); // 50 - 90 = -40
            cEgoState::singleton->setLaneDetectionAngleOfFieldOfView(tFloat32(remainingHeading));
            cEgoState::singleton->setOverrideLaneDetectionAngleOfFielOfView(true);

            if (remainingHeading < 0
                && !(remainingHeading > -LANEDETECTION_OVERTAKE_ANGLE &&  cEgoState::singleton->getLaneDetectionConfidence() > LANE_DETECTION_MIN_CONFIDENCE_THRESHOLD)) {

                taskResult.f32steeringAngle = -100.0f;
                taskResult.lights.bTurnSignalRight = tTrue;
                taskResult.f32speed = 0.3f;

                break;
            }
            state = DONE;
        }


        case DONE:
            LOG_INFO("DONE");

            cEgoState::singleton->setOverrideLaneDetectionAngleOfFielOfView(false);

            taskResult.f32steeringAngle = 0.0f;
            taskResult.f32speed = 0.0f;
            taskResult.bEmergencyBrake = tTrue;

            m_taskState = fhw::taskState::finished;
            cEgoState::singleton->setYOLOState(tTrue);
    }



    RETURN_NOERROR;
}

tFloat64 cTask_PullOutLeft::normalizeAngle(tFloat64 value) {
    tFloat64 result = value + 2 * M_PI;
    if (result > 2 * M_PI) {
        result -= 2 * M_PI;
    }
    return result;
}

/**
 * Returns the relative angle of an angle 'ofAngle' in respect to a fixed angle 'toAngle'.
 *
 * @param ofAngle the absolute angle which should get 'relative'
 * @param toAngle the new absolute 'origin' angle -> (toAngle, toAngle) would yield 0
 * @return relative angle [-179.9...180]
 */
tFloat64 cTask_PullOutLeft::getRelativeAngle(tFloat64 ofAngle, tFloat64 toAngle) {
    tFloat64 of = normalizeAngle(ofAngle);
    tFloat64 to = normalizeAngle(toAngle);

    tFloat64 rel = of - to;
    if (rel > 180.0f * M_PI / 180.0f) {
        rel -= 2.0f * M_PI;
    } else if (rel <= -180.0f * M_PI / 180.0f) {
        rel += 2.0f * M_PI;
    }

    return rel;
}

std::pair<tFloat32, int> cTask_PullOutLeft::nextParkingSpot() {
    std::pair<tFloat32, int> result;
    result.first = std::numeric_limits<float>::max();
    for(auto const &parkingSpace : cEgoState::singleton->getParkingSpaces()) {
        tFloat64 distance = sqrt(pow(parkingSpace.second.f32X - cEgoState::singleton->getVehiclePosition().f32x, 2) +
                                 pow(parkingSpace.second.f32Y - cEgoState::singleton->getVehiclePosition().f32y, 2));
        if (distance < result.first) {
            result.first = tFloat32(distance);
            result.second = parkingSpace.first;
        }
    }
    return result;
}