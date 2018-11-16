#pragma once

#include "stdafx.h"
#include "Task.h"

/*! Dummy class for testing, possibly standing with blinking lights etc. */
class cTask_DoNothing : public cTask
{
public:
    /*! Default constructor. Sets parent Task. */
    cTask_DoNothing(tBool firstTaskOfManeuver = tFalse);

    /*! Destructor. */
    virtual ~cTask_DoNothing() = default;

// ------------ methods ------------

    /*! Calculates actuator steering info from egoState and map
    * --> needs to be overridden by specific Task implementations! */
    tResult execute(tTaskResult &taskResult) override;


};