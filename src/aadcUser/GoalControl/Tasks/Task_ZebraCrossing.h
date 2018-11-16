#pragma once

#include "stdafx.h"
#include "Task.h"

/*! Dummy class for testing, possibly standing with blinking lights etc. */
class cTask_ZebraCrossing : public cTask
{

private:
    enum eState {
        DETECTED,
        STOP,
        DONE,
    };
    eState state;
    tBool detectedPersonOnce = tFalse;

public:
    /*! Default constructor. Sets parent Task. */
    cTask_ZebraCrossing(tBool firstTaskOfManeuver = tFalse);

    /*! Destructor. */
    virtual ~cTask_ZebraCrossing() = default;

// ------------ methods ------------

    /*! Calculates actuator steering info from egoState and map
    * --> needs to be overridden by specific Task implementations! */
    tResult execute(tTaskResult &taskResult) override;

    tTimeStamp m_firstStop = 0;


};