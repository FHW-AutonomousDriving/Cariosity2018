#pragma once

#include "stdafx.h"

#include "../../EnvironmentModel/Models/EgoState.h"
#include "../../EnvironmentModel/Models/Map.h"
#include "../../Helpers/cariosity_structs.h"

using namespace fhw;

/**
 * Parent class for all types. Implements default behavior and methods for generating actuator steering information.
 */
class cTask {

private:

    /*! What state the task is in */
    tBool m_isFirstTaskOfManeuver;

protected:

    /*! The state that the task is in: used for  */
    fhw::taskState m_taskState;


public:
    /*! Default constructor. Sets parent Task. */
    cTask(tBool isFirstTaskOfManeuver);

    /*! Destructor. */
    virtual ~cTask() = default;


// ------------ methods ------------

    /*! Calculates actuator steering info from egoState and map
    * --> needs to be overridden by specific Task implementations! */
    virtual tResult execute(tBool emergencyBrakeFlag, tTaskResult &taskResult);
    virtual tResult execute(tTaskResult &taskResult);

    virtual tBool isInterruptable();
    virtual tBool canTakeOverControl();

// ------- task states ------------

// ------ unchanged properties

    tBool isFirstTaskOfManeuver();

// ------ task state handling
    fhw::taskState getTaskState();
    void setTaskState(fhw::taskState theTaskState);

};