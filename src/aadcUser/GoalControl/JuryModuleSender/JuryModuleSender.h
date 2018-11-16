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


#define CID_CARIOSITY_DATA_TRIGGERED_FILTER "jurymodule_sender.filter.user.aadc.cid"

//*************************************************************************************************


/*!
 * Filter function of a time-triggered filter.
 * Allows receiving of maneuvers and immediate instructions via TCP from jury module and sending back the car status.
 * JuryStructs + CarState sent via pins
 * ManeuverList + OpenDriveMap + TrafficSignMap sent via method calls into singleton instances
 */
class fcJuryModuleSender : public cStdFilter

{
private:
// ------------------ properties ------------------

    /*! Enabling console output of jury communication */
    adtf::base::property_variable<tBool> m_propEnableConsoleOutput = tTrue;


// ------------------ pins ------------------
// ------ input pins
    cPinReader m_oCarInput_DriverStruct;

// ------ output pins
    cPinWriter     m_oJMOutput_DriverStruct;

// ------------------ maneuver member items ------------------

// ------------------ reference objects ------------------

    /*! The reference clock */
    object_ptr<adtf::services::IReferenceClock> m_pClock;

    /*! The mutex */
    std::mutex m_oMutex;

public:
    /*! Default constructor. */
    fcJuryModuleSender();

    /*! Destructor. */
    virtual ~fcJuryModuleSender();

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
    tResult receiveDriverState();

// ------ Sending data
    tResult sendDriverState(tDriverStruct &driverState);

};

