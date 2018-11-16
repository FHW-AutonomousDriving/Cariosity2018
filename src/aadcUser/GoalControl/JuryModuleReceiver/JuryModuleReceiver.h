#pragma once
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

#include "stdafx.h"

//*************************************************************************************************
using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;

using namespace aadc::jury;

using namespace std;


#define CID_CARIOSITY_DATA_TRIGGERED_FILTER "jurymodule_receiver.filter.user.aadc.cid"

//*************************************************************************************************


/*!
 * Filter function of a time-triggered filter.
 * Allows receiving of maneuvers and immediate instructions via TCP from jury module and sending back the car status.
 * JuryStructs + CarState sent via pins
 * ManeuverList + OpenDriveMap + TrafficSignMap sent via method calls into singleton instances
 */
class fcJuryModuleReceiver : public cStdFilter

{
private:
// ------------------ properties ------------------

    /*! Enabling console output of jury communication */
    adtf::base::property_variable<tBool> m_propEnableConsoleOutput = tTrue;


// ------------------ pins ------------------
// ------ input pins
    cPinReader     m_oJMInput_JuryStruct;
    cPinReader     m_oJMInput_ManeuverList;
    cPinReader     m_oJMInput_OpenDriveMap;
    cPinReader     m_oJMInput_TrafficSignMap;

// ------ output pins
    cPinWriter m_oCarOutput_JuryStruct;
    cPinWriter m_oCarOutput_ManeuverList;
    cPinWriter m_oCarOutput_OpenDriveMap;
    cPinWriter m_oCarOutput_TrafficSignMap;


// ------------------ maneuver member items ------------------

    /*! The maneuver file string */
    cString     m_strManeuverFileString;
    cString     m_strOpenDriveMapFileString;
    cString     m_strTrafficSignMapFileString;

// ------------------ reference objects ------------------

    /*! The reference clock */
    object_ptr<adtf::services::IReferenceClock> m_pClock;

    /*! The mutex */
    std::mutex m_oMutex;

public:
    /*! Default constructor. */
    fcJuryModuleReceiver();

    /*! Destructor. */
    virtual ~fcJuryModuleReceiver();

    /*!
    * Override: reads and configures Properties
    */
    tResult Configure() override;

    /*!
     * Override: called on time-trigger
     * Checks the TCP port for new Jury data to send on to the GoalController.
     * Checks the GoalController instance for new carState messages to send on to the JM.
     * Triggers the fetching, processing and sending on on of the received data.
     *
     * \param   tmTimeOfTrigger The time time of trigger.
     * \return  Standard Result Code.
     */
    tResult Process(tTimeStamp tmTimeOfTrigger) override;

private:

// ------ Receiving data
    tResult receiveJuryInstruction();
    tResult receiveManeuverList();
    tResult receiveOpenDriveMap();
    tResult receiveTrafficSignMap();

// ------ Sending data
    tResult sendJuryInstruction(tJuryStruct &juryInstruction);
    tResult sendManeuverList(); //TODO: send pointer/ compact data type
    tResult sendOpenDriveMap(); //TODO: send pointer/ compact data type
    tResult sendTrafficSignMap(); //TODO: send pointer/ compact data type

// ------ General helper methods
    std::string juryActionToString(aadc::jury::juryAction theJuryAction);

};

