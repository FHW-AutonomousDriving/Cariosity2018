#pragma once

#include "stdafx.h"
#include "Task.h"

using namespace fhw;



class cTask_PullOutLeft : public cTask
{
private:

    enum eState {
        INIT,
        STRAIGHT,
        STEERING,
        DONE
    };

    tFloat32 targetDistance;
    tFloat64 targetHeading;
    eState state;

public:
    /*! Default constructor. Sets parent Task. */
    cTask_PullOutLeft(tBool isFirstTaskOfManeuver = tFalse);

    /*! Destructor. */
    virtual ~cTask_PullOutLeft() = default;

    /*! Calculates actuator steering info from egoState and map
    * --> needs to be overridden by specific Task implementations! */
    tResult execute(tBool emergencyBrakeFlag, tTaskResult &taskResult);
    tResult execute(tTaskResult &taskResult) override;

    tFloat64 normalizeAngle(tFloat64 value);
    tFloat64 getRelativeAngle(tFloat64 ofAngle, tFloat64 toAngle);

    std::pair<tFloat32, int> nextParkingSpot();
};
