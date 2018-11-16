//
// Created by aadc on 20.09.18.
//

#include "Task_PullInRight.h"
#include "../../Helpers/cariosity_structs.h"
#include "stdafx.h"

#define SPEED 0.25f

cTask_PullInRight::cTask_PullInRight(tBool isFirstTaskOfManeuver, int parkingSpaceID) : cTask( isFirstTaskOfManeuver) {
    LOG_INFO("Initialized PARKING at %d", parkingSpaceID);
    space = cEgoState::singleton->getParkingSpaces().at(parkingSpaceID);
    state = INIT;
}

tBool cTask_PullInRight::canTakeOverControl() {
    return sqrt(pow(space.f32X - cEgoState::singleton->getVehiclePosition().f32x, 2) +
                pow(space.f32Y - cEgoState::singleton->getVehiclePosition().f32y, 2)) < 1;

}

tResult cTask_PullInRight::execute(tBool emergencyBrakeFlag, tTaskResult &taskResult) {
    cEgoState::singleton->setYOLOState(tFalse);
    return execute(taskResult);
}

tResult cTask_PullInRight::execute(tTaskResult &taskResult) {
    cEgoState::singleton->setYOLOState(tFalse);
    taskResult.lights.bHeadLight = tTrue;

    if (cEgoState::singleton->getVehiclePosition().f32heading == 0.0f) {
        RETURN_NOERROR;
    }

    switch (state) {
        case INIT:
            LOG_INFO("INIT");

            if (space.f32Direction == 0.0f) {
                targetDistance = cEgoState::singleton->getVehiclePosition().f32y - space.f32Y;
            } else if (space.f32Direction == 90.0f) {
                targetDistance = space.f32X - cEgoState::singleton->getVehiclePosition().f32x;
            } else if (space.f32Direction == 180.0f) {
                targetDistance = space.f32Y - cEgoState::singleton->getVehiclePosition().f32y;
            } else if (space.f32Direction == -90.0f) {
                targetDistance = cEgoState::singleton->getVehiclePosition().f32x - space.f32X;
            }

            targetDistance += cEgoState::singleton->getTotalDistance() - 0.2;

            LOG_INFO("Target distance: %f\t Current distance: %f", targetDistance, cEgoState::singleton->getTotalDistance());

            state = STRAIGHT;
            LOG_INFO("STRAIGHT");
            // No break because we now want to start driving

        case STRAIGHT:
            if (targetDistance > cEgoState::singleton->getTotalDistance()) {

                taskResult.f32steeringAngle = 0.0f;
                taskResult.lights.bTurnSignalRight = targetDistance - cEgoState::singleton->getTotalDistance() > 0.5f;
                taskResult.lights.bTurnSignalRight = tTrue;
                taskResult.f32speed = SPEED;
                break;
            }
            targetHeading = normalizeAngle(cEgoState::singleton->getVehiclePosition().f32heading + 0.7);
            LOG_INFO("Target heading: %f\t Current heading: %f", targetHeading, cEgoState::singleton->getVehiclePosition().f32heading);
            state = LEFT_STEERING;
            LOG_INFO("LEFT_STEERING");
            // We are done going straight, we now want to give in

        case LEFT_STEERING: {
            tFloat64 remainingHeading = getRelativeAngle(cEgoState::singleton->getVehiclePosition().f32heading, targetHeading); // 0 - 45 = -45 || 0 - 45 = -45

            if (remainingHeading < 0) {

                taskResult.f32steeringAngle = -100.0f;

                taskResult.lights.bTurnSignalRight = tTrue;
                taskResult.f32speed = SPEED;

                break;
            }
            targetHeading = normalizeAngle(cEgoState::singleton->getVehiclePosition().f32heading + 0.75);
            LOG_INFO("Target heading: %f\t Current heading: %f", targetHeading, cEgoState::singleton->getVehiclePosition().f32heading);

            // Reset Controller to full brake!
            taskResult.bEmergencyBrake = tTrue;
            state = RIGHT_STEERING;
            LOG_INFO("RIGHT_STEERING");
        }

        case RIGHT_STEERING: {
            tFloat64 remainingHeading = getRelativeAngle(cEgoState::singleton->getVehiclePosition().f32heading, targetHeading); // 45 - 90 = -45

            if (remainingHeading < 0) {

                taskResult.f32steeringAngle = 100.0f;

                taskResult.lights.bTurnSignalRight = tTrue;
                taskResult.lights.bReverseLight = tTrue;
                taskResult.f32speed = -SPEED;

                break;
            }
            targetDistance = cEgoState::singleton->getTotalDistance() + 0.45;

            LOG_INFO("Target distance: %d\t Current distance: %d", targetDistance, cEgoState::singleton->getTotalDistance());
            state = BACK_IN;
        }

        case BACK_IN:
            if (targetDistance > cEgoState::singleton->getTotalDistance()) {

                taskResult.f32steeringAngle = 0.0f;
                taskResult.lights.bTurnSignalRight = tTrue;
                taskResult.lights.bReverseLight = tTrue;
                taskResult.f32speed = -SPEED;
                break;
            }
            state = DONE;
            taskResult.lights.bHazardLight = tTrue;
            taskResult.lights.bBrakeLight = tTrue;
            taskResult.bEmergencyBrake = tTrue;
            waitEndTime = cEgoState::singleton->getLastUpdated() + tTimeStamp(5 * 1e6);
            break;

        case DONE:
            taskResult.lights.bHazardLight = tTrue;
            taskResult.lights.bBrakeLight = tTrue;
            taskResult.bEmergencyBrake = tTrue;
            LOG_INFO("Current: %d\t Target: %d", cEgoState::singleton->getLastUpdated(), waitEndTime);
            if (waitEndTime < cEgoState::singleton->getLastUpdated()) {
                LOG_INFO("DONE");
                taskResult.lights.bHazardLight = tFalse;
                taskResult.lights.bBrakeLight = tFalse;
                m_taskState = fhw::taskState::finished;
                cEgoState::singleton->setYOLOState(tTrue);
            }

    }



    RETURN_NOERROR;
}

tFloat64 cTask_PullInRight::normalizeAngle(tFloat64 value) {
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
tFloat64 cTask_PullInRight::getRelativeAngle(tFloat64 ofAngle, tFloat64 toAngle) {
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