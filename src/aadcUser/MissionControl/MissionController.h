#pragma once

#include <aadc_structs.h>

#include "../Helpers/StdFilter.h"
#include "../Helpers/cariosity_structs.h"

#include "../GoalControl/GoalController/GoalController.h"
#include "../EnvironmentModel/Models/EgoState.h"
#include "../EnvironmentModel/Models/Map.h"
#include "../GoalControl/Tasks/tasks.h"


#define CID_CARIOSITY_DATA_TRIGGERED_FILTER "fc_mission_controller.filter.user.aadc.cid"

using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;
using namespace aadc::jury;

/**
 * Evaluates the actions to take.
 * If energencyBrake or juryModule info comes in, they take precedence.
 * Otherwise passes evironment-info and roadPositioning-info to the current task reference, which returns actuator steering information.
 * If task is finished or new one should be inserted, it passes that to the GoalController and fetches the newest Task.
 *
 */
class fcMissionController : public cStdFilter
{

private:

    enum eCarState {
        STARTUP,
        READY,
        RUNNING,
        STOPPED,
        FINISHED
    };

// ------------ properties ------------

    /*! Enabling console output */
    adtf::base::property_variable<tBool> m_propEnableConsoleOutput = tTrue;
    adtf::base::property_variable<tUInt32> m_propMinimumStartupTime = 5;


// ------------ pins ------------

// ------ input pins

    // jury input
    cPinReader m_oInputJuryStruct;
    cPinReader m_oInputManeuverList;

    // environment, state and sensor input
    cPinReader m_oInputEgoState;
    cPinReader m_oInputEmergencyBrake;

    cPinReader m_oReaderOpenDriveMapAnalyzerInstanceSynchronization;


// ------ output

// ------ output pins

    // generated actuator output
    cPinWriter m_oOutputEmergencyBrake;
    cPinWriter m_oOutputSpeed;
    cPinWriter m_oOutputSteeringAngle;

    // lights output
    cPinWriter m_oOutputLight_Head;
    cPinWriter m_oOutputLight_TurnLeft;
    cPinWriter m_oOutputLight_TurnRight;
    cPinWriter m_oOutputLight_Brake;
    cPinWriter m_oOutputLight_Hazard;
    cPinWriter m_oOutputLight_Reverse;

    // generated jury output
    cPinWriter m_oOutputDriverState;



// ------------ current result fields ------------

    tJuryStruct m_currentJuryInstruction;
    tDriverStruct m_currentDriverState;

    tTaskResult m_currentTaskResult;

    tSignalValue m_emergencyBrakeSignal;

    eCarState m_currentCarState = STARTUP;

    tBool m_bLight_Head;
    tBool m_bLight_TurnLeft;
    tBool m_bLight_TurnRight;
    tBool m_bLight_Brake;
    tBool m_bLight_Hazard;
    tBool m_bLight_Reverse;


// ------------ startup requirements ------------
    tBool m_recievedManeuver = tFalse;
    tBool m_recievedMap = tFalse;
    tBool m_hasHeading = tFalse;
    tBool m_timePassed = tFalse;

    tTimeStamp m_startTime = 0;

private:
// ------------ methods ------------

    tResult handleTask();

    /*! Checks for need to add immediate action to goal controller queue
     * Sends actuator info from struct to output pins */
    tResult transmitTaskResult(tTaskResult &taskResult, tTimeStamp timeOfTrigger);

// ------ jury communication

    tResult receiveManeuverList();
    tResult receiveJuryInstruction();

    tResult sendDriverStateToJury(tDriverStruct &theDriverState);

    tResult receiveOpenDriveMapAnalyzerInstance();
    tResult receiveEgoStateInstance();

public:
    /*! Default constructor. */
    fcMissionController();

    /*! Destructor. */
    virtual ~fcMissionController() = default;

    tResult Configure() override;

    /*! Executed on data-trigger from pins
     * Checks for new jury instruction, that needs to take priority
     * Checks task-state and possibly fetches new task
     *
     * */
    tResult Process(tTimeStamp tmTimeOfTrigger) override;

};