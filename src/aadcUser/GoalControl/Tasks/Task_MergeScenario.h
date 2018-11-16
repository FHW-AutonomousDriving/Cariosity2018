#pragma once

#include "stdafx.h"
#include "Task.h"
#include "../../Helpers/OpenDriveAnalyzer/OpenDriveMapAnalyzer.h"



class cTask_MergeScenario : public cTask {
private:
    aadc::jury::maneuver m_direction;
    tJunctionEntry m_junctionEntry;

    tVehiclePosition m_target;
    tFloat32 m_intermediateHeading;


    enum State {
        SEARCHING_FOR_ENTRY, // 0
        INITIALIZE,          // 1
        CHECK_RIGHT_OF_WAY,  // 2
        ENTER_MERGE,  // 3
        ON_MERGE,     // 4
        EXIT_MERGE    // 5
    };

    State m_state = SEARCHING_FOR_ENTRY;

    tFloat32 getRelativeAngle(tFloat32 ofAngle, tFloat32 toAngle);

public:
    /*! Default constructor. */
    cTask_MergeScenario(tBool isFirstTaskOfManeuver, aadc::jury::maneuver direction);

    /*! Destructor. */
    virtual ~cTask_MergeScenario() = default;

// ------------ methods ------------

    /*! Calculates actuator steering info from egoState and map */
    tResult execute(tTaskResult &taskResult) override;

    tBool canTakeOverControl() override;

private:
    Point transform(const tVehiclePosition &origin, const tVehiclePosition &pointToTransform);

    tVehiclePosition getFrontAxlePos(tVehiclePosition origin);

    tFloat32 normalizeAngle(tFloat32 angle);

};