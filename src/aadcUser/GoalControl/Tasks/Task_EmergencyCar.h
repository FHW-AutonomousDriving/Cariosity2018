#pragma once

#include "stdafx.h"
#include "Task.h"

/*! Dummy class for testing, possibly standing with blinking lights etc. */
class cTask_EmergencyCar : public cTask
{

private:
    enum eState {
        DETECTED,
        DRIVE_RIGHT,
        PULL_STRAIGHT,
        STOP,
        FINDWAYBACKONTOTHESTREET
    };
    eState state;
    tFloat64 targetDistance = 0.0f;

    tTimeStamp m_firstStop = 0;

public:
    /*! Default constructor. Sets parent Task. */
    cTask_EmergencyCar(tBool firstTaskOfManeuver = tFalse);

    /*! Destructor. */
    virtual ~cTask_EmergencyCar() = default;

// ------------ methods ------------

    /*! Calculates actuator steering info from egoState and map
    * --> needs to be overridden by specific Task implementations! */
    tResult execute(tTaskResult &taskResult) override;


};