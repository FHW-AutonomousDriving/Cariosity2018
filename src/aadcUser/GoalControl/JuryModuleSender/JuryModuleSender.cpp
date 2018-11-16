#include "stdafx.h"

#include "JuryModuleSender.h"

//*************************************************************************************************

ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_CARIOSITY_DATA_TRIGGERED_FILTER,
                                    "JuryModuleSender",
                                    fcJuryModuleSender,
                                    adtf::filter::pin_trigger({ "in_carstate" }));


#define CONSOLE_LOG(_text, _log_level) if (m_propEnableConsoleOutput) { LOG_ADD_ENTRY(_log_level, _text); }    //!< enables log function if console output is activated
#define CONSOLE_LOG_INFO(_text)      CONSOLE_LOG(_text, A_UTILS_NS::log::tLogLevel::Info)                        //!< log info messages

//*************************************************************************************************

// ------ General ------

// Constructor
fcJuryModuleSender::fcJuryModuleSender() : cStdFilter()
{
    // Register Properties

    RegisterPropertyVariable("Enable console log", m_propEnableConsoleOutput);

    // Register pins

    // ------input pins

    Register(m_oCarInput_DriverStruct, "in_carstate" , getStreamType(DRIVER_STATE));

    // ------ output pins

    Register(m_oJMOutput_DriverStruct, "out_carstate" , getStreamType(DRIVER_STATE));
}

// Destructor
fcJuryModuleSender::~fcJuryModuleSender() {
}

tResult fcJuryModuleSender::Configure()
{
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    RETURN_NOERROR;
}

tResult fcJuryModuleSender::Process(tTimeStamp tmTimeOfTrigger)
{
    //if (m_propEnableConsoleOutput) { printf("\n"); }
    // TODO: bug!! called all the time; triggered without trigger
    //CONSOLE_LOG_INFO(adtf_util::cString::Format("JuryModuleSender on data triggered ..."));

    //std::lock_guard<std::mutex> oGuard(m_oMutex);

    receiveDriverState();

    RETURN_NOERROR;
}


// ------------ Receiving data ------------

tResult fcJuryModuleSender::receiveDriverState() {
    tDriverStruct driverStruct;
    if(IS_OK(readDriverStateData(m_oCarInput_DriverStruct, driverStruct))) {
        CONSOLE_LOG_INFO(cString::Format("Received new driver state data. %s (%d. maneuverlist entry)", stateCarToString(stateCar(driverStruct.i16StateID)).c_str(), driverStruct.i16ManeuverEntry));
        // send received driver state to jury module
        return sendDriverState(driverStruct);
    }
    RETURN_NOERROR;
}

// ------------ Sending data ------------

tResult fcJuryModuleSender::sendDriverState(tDriverStruct &driverState) {
    return transmitDriverStateData(driverState, m_oJMOutput_DriverStruct);
}