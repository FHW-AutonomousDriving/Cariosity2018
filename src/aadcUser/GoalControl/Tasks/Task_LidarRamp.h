#pragma once

#include "stdafx.h"
#include <aadc_geometrics.h>
#include "Task.h"
#include "../../EnvironmentModel/Models/EgoState.h"



class cTask_LidarRamp : public cTask {
private:

    tPolarCoordiante findCoordinate();

public:
    /*! Default constructor. */
    cTask_LidarRamp(tBool isFirstTaskOfManeuver);

    /*! Destructor. */
    virtual ~cTask_LidarRamp() = default;

// ------------ methods ------------

    /*! Calculates actuator steering info from egoState and map */
    tResult execute(tBool emergencyBrakeFlag, tTaskResult &taskResult);
    tResult execute(tTaskResult &taskResult) override;

    tBool canTakeOverControl() override;

};