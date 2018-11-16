#pragma once

#include "stdafx.h"
#include "Task.h"
#include "../../Helpers/OpenDriveAnalyzer/OpenDriveMapAnalyzer.h"



class cTask_IntersectionHandling : public cTask {
private:
    aadc::jury::maneuver m_direction;
    tJunctionEntry m_junctionEntry;

    tVehiclePosition m_target;

    enum State {
        SEARCHING_FOR_ENTRY,    // 0
        INITIALIZE,             // 1
        APPROACH_INTERSECTION,  // 2
        CHECK_RIGHT_OF_WAY,     // 3
        WAIT_AT_STOP_SIGN,      // 4
        ENTER_INTERSECTION,     // 5
        STEERING,               // 6
        EXIT_INTERSECTION       // 7
    };

    State m_state = SEARCHING_FOR_ENTRY;

    tBool m_personDetected = tFalse;
    tTimeStamp m_timeStartedWaitingAtJunctionEntry;
    tTimeStamp m_timeStartedWaitingForPeople;
    tTimeStamp m_timeRemainingHeadingLastUpdated = 0;

    tTimeStamp m_timeLastEmergencyVehicle = 0;
    tFloat32 m_lastRemainingHeading = 0;
    tFloat32 m_angleVelocity = 0;

public:
    /*! Default constructor. */
    cTask_IntersectionHandling(tBool isFirstTaskOfManeuver, aadc::jury::maneuver direction);

    /*! Destructor. */
    virtual ~cTask_IntersectionHandling() = default;

// ------------ methods ------------

    /*! Calculates actuator steering info from egoState and map */
    tResult execute(tTaskResult &taskResult) override;

    tBool canTakeOverControl() override;

    Point transform(const tVehiclePosition &origin, const tVehiclePosition &pointToTransform);

    tBool checkRightOfWay();

    tBool checkIntersectionRight();

    tBool checkIntersectionStraight();

    tBool checkIntersectionLeft();

    Point toCartesian(const tPolarCoordiante &polar);

    tVehiclePosition getLaserScannerPos(tVehiclePosition origin);
    tVehiclePosition getFrontAxlePos(tVehiclePosition origin);

    tFloat32 normalizeAngle(tFloat32 angle);


    tFloat32 getRelativeAngle(tFloat32 ofAngle, tFloat32 toAngle);

};