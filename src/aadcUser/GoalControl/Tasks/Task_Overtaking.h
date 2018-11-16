#pragma once

#include "stdafx.h"
#include "Task.h"
#include "../../Helpers/OpenDriveAnalyzer/OpenDriveMapAnalyzer.h"


using namespace fhw;

class cTask_Overtaking : public cTask
{
private:

// -------------- Definition of states of state machine --------------

    enum eState {
        INIT,                           // 0
        DRIVING_BACKWARDS,              // 1
        STOP,                           // 2
        SWITCHING_TO_LEFT_LANE,         // 3
        OVERTAKING_ON_LEFT_LANE,        // 4
        SWITCHING_TO_RIGHT_LANE,        // 5
        DONE                            // 6
    };

// -------------- Member variables --------------

    tBool m_enableConsoleOutput = tFalse;
    tBool m_enableStateChangeOutput = tTrue;

    eState m_state = INIT;
    tFloat32 m_targetDistanceBackwards = 0.6f; // in m

public:
    /*! Default constructor. Sets parent Task. */
    cTask_Overtaking(tBool isFirstTaskOfManeuver = tFalse);

    /*! Destructor. */
    virtual ~cTask_Overtaking() = default;


    // ------------ methods ------------

    /*! Calculates actuator steering info from egoState and map */
    tResult execute(tBool emergencyBrakeFlag, tTaskResult &taskResult) override;

};