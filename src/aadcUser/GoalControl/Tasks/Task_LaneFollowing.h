#pragma once

#include "stdafx.h"
#include "Task.h"

using namespace fhw;

class cTask_LaneFollowing : public cTask
{
private:
    tBool m_enableConsoleOutput = tTrue;

    tFloat32 putValueInRange(tFloat32 value, tFloat32 min, tFloat32 max);

public:
    /*! Default constructor. Sets parent Task. */
    cTask_LaneFollowing(tBool isFirstTaskOfManeuver = tFalse);

    /*! Destructor. */
    virtual ~cTask_LaneFollowing() = default;

    /*! Calculates actuator steering info from egoState and map
    * --> needs to be overridden by specific Task implementations! */
    tResult execute(tTaskResult &taskResult) override;

    tBool isInterruptable() override;

};
