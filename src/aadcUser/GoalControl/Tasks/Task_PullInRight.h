#pragma once

#include "stdafx.h"
#include "Task.h"

using namespace fhw;



class cTask_PullInRight : public cTask
{
private:

    enum eState {
        INIT,
        STRAIGHT,
        LEFT_STEERING,
        RIGHT_STEERING,
        BACK_IN,
        DONE
    };

    tFloat64 targetDistance;
    tFloat64 targetHeading;
    tRoadSign space;
    eState state;
    tTimeStamp waitEndTime;

public:
    /*! Default constructor. Sets parent Task. */
    cTask_PullInRight(tBool isFirstTaskOfManeuver = tFalse, int parkingSpaceID = 0);

    /*! Destructor. */
    virtual ~cTask_PullInRight() = default;

    /*! Indicates that the task should be started for its correct execution
     * For tasks that can be started in any state this defaults false and the
     * task will be started when the previous is done */
    tBool canTakeOverControl() override;


    /*! Calculates actuator steering info from egoState and map
    * --> needs to be overridden by specific Task implementations! */
    tResult execute(tBool emergencyBrakeFlag, tTaskResult &taskResult);
    tResult execute(tTaskResult &taskResult) override;

    tFloat64 normalizeAngle(tFloat64 value);
    tFloat64 getRelativeAngle(tFloat64 ofAngle, tFloat64 toAngle);

    };
