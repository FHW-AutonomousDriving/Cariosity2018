#include "stdafx.h"

#include "JuryModuleReceiver.h"

//*************************************************************************************************

ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_CARIOSITY_DATA_TRIGGERED_FILTER,
                                    "JuryModuleReceiver",
                                    fcJuryModuleReceiver,
                                    adtf::filter::pin_trigger({"in_jurystruct",
                                                               "in_maneuver_list",
                                                               "in_opendrive_map",
                                                               "in_trafficsign_map" }));


#define CONSOLE_LOG(_text, _log_level) if (m_propEnableConsoleOutput) { LOG_ADD_ENTRY(_log_level, _text); }    //!< enables log function if console output is activated
#define CONSOLE_LOG_INFO(_text)      CONSOLE_LOG(_text, A_UTILS_NS::log::tLogLevel::Info)                        //!< log info messages

//*************************************************************************************************

// ------ General ------

// Constructor
fcJuryModuleReceiver::fcJuryModuleReceiver() : cStdFilter()
{
    // Register Properties

    RegisterPropertyVariable("Enable console log", m_propEnableConsoleOutput);

    // Register pins

    // general pintype for large unspecific data: maneuverList, openDriveMap, trafficSignMap
    object_ptr<IStreamType> pTypeDefault = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_anonymous());

    // ------input pins

    Register(m_oJMInput_JuryStruct, "in_jurystruct" , getStreamType(JURY_INSTRUCTION));
    Register(m_oJMInput_ManeuverList, "in_maneuver_list", pTypeDefault);
    Register(m_oJMInput_OpenDriveMap, "in_opendrive_map", pTypeDefault);
    Register(m_oJMInput_TrafficSignMap, "in_trafficsign_map", pTypeDefault);


    // ------ output pins

    Register(m_oCarOutput_JuryStruct, "out_jurystruct" , getStreamType(JURY_INSTRUCTION));
    Register(m_oCarOutput_ManeuverList, "out_maneuver_list", pTypeDefault);
    Register(m_oCarOutput_OpenDriveMap, "out_opendrive_map", pTypeDefault);
    Register(m_oCarOutput_TrafficSignMap, "out_trafficsign_map", pTypeDefault);
}

// Destructor
fcJuryModuleReceiver::~fcJuryModuleReceiver() {
}

tResult fcJuryModuleReceiver::Configure()
{
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    RETURN_NOERROR;
}

tResult fcJuryModuleReceiver::Process(tTimeStamp tmTimeOfTrigger)
{
    if (m_propEnableConsoleOutput) { printf("\n"); }
    CONSOLE_LOG_INFO(adtf_util::cString::Format("JuryModuleReceiver on data triggered ..."));

    std::lock_guard<std::mutex> oGuard(m_oMutex);

    receiveJuryInstruction();
    receiveManeuverList();
    receiveOpenDriveMap();
    receiveTrafficSignMap();

    RETURN_NOERROR;
}


// ------------ Receiving data ------------

tResult fcJuryModuleReceiver::receiveJuryInstruction() {
    CONSOLE_LOG_INFO(cString::Format("Checking for new jury instruction data"));

    tJuryStruct juryStruct;
    if(IS_OK(readJuryInstructionData(m_oJMInput_JuryStruct, juryStruct))) {
        CONSOLE_LOG_INFO(cString::Format("Received new jury instruction data. %s (%d. maneuverlist entry)", juryActionToString(juryAction(juryStruct.i16ActionID)).c_str(), juryStruct.i16ManeuverEntry));
        // send received jury instruction data to car
        sendJuryInstruction(juryStruct);
    }

    RETURN_NOERROR;
}

tResult fcJuryModuleReceiver::receiveManeuverList() {
    CONSOLE_LOG_INFO(cString::Format("Checking for new maneuver list data"));

    object_ptr<const ISample> pManeuverListData;
    if(IS_OK(m_oJMInput_ManeuverList.GetNextSample(pManeuverListData)))
    {
        vector<tChar> data;
        object_ptr_shared_locked<const ISampleBuffer> pSampleBuffer;
        RETURN_IF_FAILED(pManeuverListData->Lock(pSampleBuffer));
        data.resize(pSampleBuffer->GetSize());
        memcpy(data.data(), pSampleBuffer->GetPtr(), pSampleBuffer->GetSize());
        if (data.size() > 0)
        {//maneuverlist
            CONSOLE_LOG_INFO(cString::Format("Received ManeuverList data :-)"));
            m_strManeuverFileString.Set(data.data(), data.size());
            // sending on maneuver list
            sendManeuverList();
        }
    }

    RETURN_NOERROR;
}

tResult fcJuryModuleReceiver::receiveOpenDriveMap() {
    CONSOLE_LOG_INFO(cString::Format("Checking for new open drive map data"));

    object_ptr<const ISample> pOpenDriveMapData;
    if(IS_OK(m_oJMInput_OpenDriveMap.GetNextSample(pOpenDriveMapData)))
    {
        vector<tChar> data;
        object_ptr_shared_locked<const ISampleBuffer> pSampleBuffer;
        RETURN_IF_FAILED(pOpenDriveMapData->Lock(pSampleBuffer));
        data.resize(pSampleBuffer->GetSize());
        memcpy(data.data(), pSampleBuffer->GetPtr(), pSampleBuffer->GetSize());
        if (data.size() > 0)
        {//open drive map
            CONSOLE_LOG_INFO(cString::Format("Received open drive map data :-)"));
            m_strOpenDriveMapFileString.Set(data.data(), data.size());
            //TODO: send on to EnvironmentalModel
            sendOpenDriveMap();
        }
    }

    RETURN_NOERROR;
}

tResult fcJuryModuleReceiver::receiveTrafficSignMap() {
    CONSOLE_LOG_INFO(cString::Format("Checking for new traffic sign map data"));

    object_ptr<const ISample> pTrafficSignMapData;
    if(IS_OK(m_oJMInput_TrafficSignMap.GetNextSample(pTrafficSignMapData)))
    {
        vector<tChar> data;
        object_ptr_shared_locked<const ISampleBuffer> pSampleBuffer;
        RETURN_IF_FAILED(pTrafficSignMapData->Lock(pSampleBuffer));
        data.resize(pSampleBuffer->GetSize());
        memcpy(data.data(), pSampleBuffer->GetPtr(), pSampleBuffer->GetSize());
        if (data.size() > 0)
        {//traffic sign map
            CONSOLE_LOG_INFO(cString::Format("Received traffic sign map data :-)"));
            m_strTrafficSignMapFileString.Set(data.data(), data.size());
            //TODO: send on to EnvironmentalModel
            sendTrafficSignMap();
        }
    }

    RETURN_NOERROR;
}


// ------------ Sending data ------------

tResult fcJuryModuleReceiver::sendJuryInstruction(tJuryStruct &juryInstruction) {
    transmitJuryInstructionData(juryInstruction, m_oCarOutput_JuryStruct);
    RETURN_NOERROR;
}

tResult fcJuryModuleReceiver::sendManeuverList() {
    LOG_INFO("Sending maneuverList to MissionController (-> GoalController)...");

    object_ptr<ISample> pSample;
    RETURN_IF_FAILED(alloc_sample(pSample));
    pSample->SetTime(m_pClock->GetStreamTime());
    {
        object_ptr_locked<ISampleBuffer> pBuffer;
        RETURN_IF_FAILED(pSample->WriteLock(pBuffer, m_strManeuverFileString.GetLength()));
        RETURN_IF_FAILED(pBuffer->Write(adtf_memory_buffer<const tVoid>(m_strManeuverFileString, m_strManeuverFileString.GetLength())));
    }
    m_oCarOutput_ManeuverList << pSample << flush << trigger;
    LOG_INFO("Sent maneuverList to MissionController (-> GoalController).");

    RETURN_NOERROR;
}

tResult fcJuryModuleReceiver::sendOpenDriveMap() {
    LOG_INFO("Sending OpenDrive map to EnvironmentalModel...");

    object_ptr<ISample> pSample;
    RETURN_IF_FAILED(alloc_sample(pSample));
    pSample->SetTime(m_pClock->GetStreamTime());
    {
        object_ptr_locked<ISampleBuffer> pBuffer;
        RETURN_IF_FAILED(pSample->WriteLock(pBuffer, m_strOpenDriveMapFileString.GetLength()));
        RETURN_IF_FAILED(pBuffer->Write(adtf_memory_buffer<const tVoid>(m_strOpenDriveMapFileString, m_strOpenDriveMapFileString.GetLength())));
    }
    m_oCarOutput_OpenDriveMap << pSample << flush << trigger;
    LOG_INFO("Sent OpenDrive map to EnvironmentalModel...");

    RETURN_NOERROR;
}

tResult fcJuryModuleReceiver::sendTrafficSignMap() {
    LOG_INFO("Sending traffic sign map to EnvironmentalModel...");

    object_ptr<ISample> pSample;
    RETURN_IF_FAILED(alloc_sample(pSample));
    pSample->SetTime(m_pClock->GetStreamTime());
    {
        object_ptr_locked<ISampleBuffer> pBuffer;
        RETURN_IF_FAILED(pSample->WriteLock(pBuffer, m_strTrafficSignMapFileString.GetLength()));
        RETURN_IF_FAILED(pBuffer->Write(adtf_memory_buffer<const tVoid>(m_strTrafficSignMapFileString, m_strTrafficSignMapFileString.GetLength())));
    }
    m_oCarOutput_TrafficSignMap << pSample << flush << trigger;
    LOG_INFO("Sent traffic sign map to EnvironmentalModel...");

    RETURN_NOERROR;
}



// ------------ General data processing

// ------ General helper methods

std::string fcJuryModuleReceiver::juryActionToString(aadc::jury::juryAction theJuryAction) {
    std::string juryActionName = std::string("");
    switch(theJuryAction){
        case action_stop :
            return std::string("STOP");
        case action_getready :
            return std::string("GET READY");
        case action_start :
            return std::string("START");
    }
    return juryActionName;
}