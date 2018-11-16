#include "EnvironmentalModel.h"
#include "ADTF3_OpenCV_helper.h"
#include "stdafx.h"
#include "../Helpers/cariosity_structs.h"
#include "../Helpers/Median.h"

#define egoState cEgoState::singleton

using namespace fhw;

//*************************************************************************************************

ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_CARIOSITY_DATA_TRIGGERED_FILTER,
                                    "EnvironmentalModel",
                                    fcEnvironmentalModel,
                                    adtf::filter::pin_trigger({"laserScanner", "in_opendrive_map", "in_trafficsign_map"}));

#define DISTANCE_LASER_SCANNER_REAR_AXEL 0.395f

#define CONSOLE_LOG(_text, _log_level) if (m_propEnableConsoleOutput) { LOG_ADD_ENTRY(_log_level, _text); }    //!< enables log function if console output is activated
#define CONSOLE_LOG_INFO(...) if (m_propEnableConsoleOutput) LOG_ADD_ENTRY(A_UTILS_NS::log::tLogLevel::Info, __VA_ARGS__)

//*************************************************************************************************


fcEnvironmentalModel::fcEnvironmentalModel() : cStdFilter()
{
// ------ Register Properties

    RegisterPropertyVariable("Enable console log", m_propEnableConsoleOutput);



// ------ Register Pins

    //the position struct
    object_ptr<IStreamType> pTypePositionData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tPosition", pTypePositionData, m_PositionSampleFactory))
    {
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32x", m_ddlPositionIndex.x);
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32y", m_ddlPositionIndex.y);
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32radius", m_ddlPositionIndex.radius);
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32speed", m_ddlPositionIndex.speed);
        adtf_ddl::access_element::find_index(m_PositionSampleFactory, "f32heading", m_ddlPositionIndex.heading);
    }
    else
    {
        LOG_WARNING("No mediadescription for tPosition found!");
    }

    object_ptr<const IStreamType> pConstTypePositionData = pTypePositionData;

    //Register output pins
    m_OutPinVideoFormat.m_ui32Width = 42;
    m_OutPinVideoFormat.m_ui32Height = 42;
    m_OutPinVideoFormat.m_szMaxByteSize = 42;
    m_OutPinVideoFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
    adtf::ucom::object_ptr<IStreamType> pTypeOutput = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pTypeOutput, m_OutPinVideoFormat);
    Register(m_oWriterVideo, "output", pTypeOutput);


    Register(m_oReaderLaserScanner, "laserScanner", getStreamType(LASER_SCANNER));
    Register(m_oReaderUSSideRight, "USSideRight", getStreamType(SIGNAL_VALUE));
    Register(m_oReaderUSRearRight, "USRearRight", getStreamType(SIGNAL_VALUE));
    Register(m_oReaderUSRearCenter, "USRearCenter", getStreamType(SIGNAL_VALUE));
    Register(m_oReaderUSRearLeft, "USRearLeft", getStreamType(SIGNAL_VALUE));
    Register(m_oReaderUSSideLeft, "USSideLeft", getStreamType(SIGNAL_VALUE));

    Register(m_oReaderPosition, "Position", pConstTypePositionData);
    Register(m_oReaderTotalDistance, "TotalDistance", getStreamType(SIGNAL_VALUE));

    Register(m_oReaderRelativeLanePositioning, "RelativeLanePositioning", getStreamType(SIGNAL_VALUE));
    Register(m_oReaderLanes, "lanes", getStreamType(LANES));

    Register(m_oSampleWriter, "newStateTrigger", getStreamType(SIGNAL_VALUE));
    Register(m_oWriterEgoState, "egoStateStruct", getStreamType(POINTER_VALUE));

    Register(m_oReaderZebraDetection, "ZebraDetection", getStreamType(BOOL_SIGNAL_VALUE));

    Register(m_oWriterOpenDriveMapAnalyzerInstanceSynchronization, "openDriveMapAnalyzerInstanceSynchronization", getStreamType(POINTER_VALUE));

    Register(m_oReaderEmergencyBrakeSignal, "EmergencyBrakeSignal", getStreamType(SIGNAL_VALUE));


// ------ data types for map data from jury
    // general pintype for large unspecific data: maneuverList, openDriveMap, trafficSignMap
    object_ptr<IStreamType> pTypeDefault = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_anonymous());

    Register(m_oInput_OpenDriveMap, "in_opendrive_map", pTypeDefault);
    m_mapAnalyzer = new cOpenDriveMapAnalyzer();

    Register(m_oInput_TrafficSignMap, "in_trafficsign_map", pTypeDefault);
}


//implement the Configure function to read ALL Properties
tResult fcEnvironmentalModel::Configure()
{
    if (!cEgoState::isInitialized()) {
        cEgoState::singleton = new cEgoState;
    }
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    RETURN_NOERROR;
}

tResult fcEnvironmentalModel::Process(tTimeStamp tmTimeOfTrigger)
{
// ------ receive jury map data: open drive and traffic sign

    receiveOpenDriveMap();
    receiveTrafficSignMap();

    vector<tPolarCoordiante> lLSData;
    if(IS_OK(readLaserScannerData(m_oReaderLaserScanner, lLSData))) {
        egoState->setLaserScannerData(lLSData);
    }

    tSignalValue lUSSideRight;
    if(IS_OK(readSignalValue(m_oReaderUSSideRight, lUSSideRight))) {
        egoState->setUSSideRight(lUSSideRight);
    }

    tSignalValue lUSRearRight;
    if(IS_OK(readSignalValue(m_oReaderUSRearRight, lUSRearRight))) {
        egoState->setUSRearRight(lUSRearRight);
    }

    tSignalValue lUSRearCenter;
    if(IS_OK(readSignalValue(m_oReaderUSRearCenter, lUSRearCenter))) {
        egoState->setUSRearCenter(lUSRearCenter);
    }

    tSignalValue lUSRearLeft;
    if(IS_OK(readSignalValue(m_oReaderUSRearLeft, lUSRearLeft))) {
        egoState->setUSRearLeft(lUSRearLeft);
    }

    tSignalValue lUSSideLeft;
    if(IS_OK(readSignalValue(m_oReaderUSSideLeft, lUSSideLeft))) {
        egoState->setUSSideLeft(lUSSideLeft);
    }

    tVehiclePosition lPosition;
    if(IS_OK(readPosition(m_oReaderPosition, lPosition))) {
        egoState->setVehiclePosition(lPosition);
    }

    tSignalValue lTotalDistance;
    if(IS_OK(readSignalValue(m_oReaderTotalDistance, lTotalDistance))) {
        egoState->setTotalDistance(lTotalDistance.f32Value);
    }

// ------ checking pins for lane information

    tSignalValue lRelativeLanePositioning;
    if(IS_OK(readSignalValue(m_oReaderRelativeLanePositioning, lRelativeLanePositioning))) {
        egoState->setRelativeLanePositioning(lRelativeLanePositioning.f32Value);
        egoState->setLaneDetectionConfidence(lRelativeLanePositioning.ui32ArduinoTimestamp); // dirty but correct!
    }

    tLanes lLanes;
    if(IS_OK(readLanesData(m_oReaderLanes, lLanes))) {
        egoState->setLanes(lLanes);
    }

// ------ checking pins for detection of environment situations

    tBoolSignalValue lZebraDetected;
    if(IS_OK(readBoolSignalValue(m_oReaderZebraDetection, lZebraDetected))) {
        if (lZebraDetected.bValue && egoState->getEnvironmentSituation() == environmentSituation::lanekeeping) {
            egoState->setEnvironmentSituation(environmentSituation::zebra_crossing);
        } else {
            egoState->setEnvironmentSituation(environmentSituation::lanekeeping);
        }
    }




    handleEmergencyBrakeSignal(tmTimeOfTrigger);

    egoState->setLastUpdated(tmTimeOfTrigger);
    transmitEgoStateInstance();

    RETURN_NOERROR;
}

tResult fcEnvironmentalModel::handleEmergencyBrakeSignal(tTimeStamp tmTimeOfTrigger) {
    static tBool lastSignal = tFalse;
    static tBool obstacleAheadDetected;
    static tTimeStamp lastUpdated;
    static cMedian emergencyBrakeSignals(20);

    // we only get a new emergency brake signal value if it has changed!
    tSignalValue lEmergencyBrakeSignal;
    if(IS_OK(readSignalValue(m_oReaderEmergencyBrakeSignal, lEmergencyBrakeSignal))) {
        m_sigvLastEmergencyBrakeSignal = lEmergencyBrakeSignal;

        emergencyBrakeSignals.pushValue(lEmergencyBrakeSignal.f32Value);

        tBool currentSignal = emergencyBrakeSignals.calculateMedian() > 0.5f; // we detected an emergencybrake

        if (currentSignal != lastSignal) {
            CONSOLE_LOG_INFO("Emergency Brake changed: %d. Time: %u", currentSignal, tmTimeOfTrigger);
            lastUpdated = tmTimeOfTrigger;
            lastSignal = currentSignal;
        }
    }

    // check if emergency Brake detected + time has run out for obstacle detection -> set situation obstacle if not zebra
    if(lastSignal && (lastUpdated + tTimeStamp(m_propEmergencyBrakeSecsForObstacleDetection * 1e6)) < tmTimeOfTrigger) {
        if(egoState->getEnvironmentSituation() != fhw::environmentSituation::zebra_crossing
           && !obstacleAheadDetected) {
            // obstacle detected and no zebra: set stituation obstacle ahead
            CONSOLE_LOG_INFO("Obstacle ahead! time since start:%f", ((lastUpdated + tTimeStamp(m_propEmergencyBrakeSecsForObstacleDetection * 1e6))/1e6) );
            obstacleAheadDetected = tTrue;
            egoState->setEnvironmentSituation(environmentSituation::obstacle_ahead);
        }
    }
    else { // obstacle detected before but no longer true: set lanekeeping situation
        obstacleAheadDetected =tFalse;
    }
    RETURN_NOERROR;
}


void fcEnvironmentalModel::processLaserScannerData(vector<tPolarCoordiante> data) {
    tVehiclePosition currentPos = egoState->getVehiclePosition();
    for (tPolarCoordiante coordinate : data) {
        tObstacle obstacle = {
            currentPos.f32x + cos(currentPos.f32heading) * DISTANCE_LASER_SCANNER_REAR_AXEL + cos(-currentPos.f32heading + degToRad(coordinate.f32Angle)) * coordinate.f32Radius *0.001f,
            currentPos.f32y + sin(currentPos.f32heading) * DISTANCE_LASER_SCANNER_REAR_AXEL + sin(+currentPos.f32heading - degToRad(coordinate.f32Angle)) * coordinate.f32Radius *0.001f
        };
        //CONSOLE_LOG_INFO(cString::Format("Radius: %f; Angle: %f; Inserting at %f, %f", coordinate.f32Radius, coordinate.f32Angle, obstacle.f32x, obstacle.f32y));
        //LOG_INFO("Map Position ( %f | %f )", obstacle.f32x, obstacle.f32y);

        cMap::getInstance().set(obstacle.f32x, obstacle.f32y, OBSTACLE);
    }
}

tFloat32 fcEnvironmentalModel::degToRad(tFloat32 angle) {
    return tFloat32(angle * M_PI / 180);
}

tResult fcEnvironmentalModel::readPosition(cPinReader &inputPin, tVehiclePosition &positionData) {
    object_ptr<const ISample> pReadSample;

    RETURN_IF_FAILED(inputPin.GetLastSample(pReadSample));
    auto oDecoder = m_PositionSampleFactory.MakeDecoderFor(*pReadSample);

    RETURN_IF_FAILED(oDecoder.IsValid());

    // retrieve the values (using convenience methods that return a variant)
    RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.x, &(positionData.f32x)));
    RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.y, &(positionData.f32y)));
    RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.radius, &(positionData.f32radius)));
    RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.speed, &(positionData.f32speed)));
    RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlPositionIndex.heading, &(positionData.f32heading)));


    RETURN_NOERROR;
}

tResult fcEnvironmentalModel::receiveOpenDriveMap() {
    //CONSOLE_LOG_INFO(cString::Format("Checking for new open drive map data"));

    // Check for new map data
    adtf::ucom::object_ptr<const adtf::streaming::ISample> pSampleAnonymous;
    while (IS_OK(m_oInput_OpenDriveMap.GetNextSample(pSampleAnonymous))) {

        adtf::ucom::ant::object_ptr_shared_locked<const adtf::streaming::ant::ISampleBuffer> pSampleBuffer;
        RETURN_IF_FAILED(pSampleAnonymous->Lock(pSampleBuffer));

        adtf_util::cString openDriveMapFileString;
        openDriveMapFileString.SetBuffer(pSampleBuffer->GetSize());
        memcpy(openDriveMapFileString.GetBuffer(), pSampleBuffer->GetPtr(), pSampleBuffer->GetSize());

        if (openDriveMapFileString.GetBufferSize() > 0) {
            RETURN_IF_FAILED(m_mapAnalyzer->analyzeOpenDriveXML(openDriveMapFileString.GetBuffer(), 10));
            transmitOpenDriveMapAnalyzerInstance();
        }
    }

    RETURN_NOERROR;
}

tResult fcEnvironmentalModel::receiveTrafficSignMap() {
    //CONSOLE_LOG_INFO(cString::Format("Checking for new traffic sign map data"));

    object_ptr<const ISample> pTrafficSignMapData;
    if(IS_OK(m_oInput_TrafficSignMap.GetNextSample(pTrafficSignMapData))) {
        CONSOLE_LOG_INFO(cString::Format("Received data at traffic sign map pin..."));

        vector<tChar> data;
        object_ptr_shared_locked<const ISampleBuffer> pSampleBuffer;
        RETURN_IF_FAILED(pTrafficSignMapData->Lock(pSampleBuffer));
        data.resize(pSampleBuffer->GetSize());
        memcpy(data.data(), pSampleBuffer->GetPtr(), pSampleBuffer->GetSize());

        if (!data.empty()) { //traffic sign map
            CONSOLE_LOG_INFO(cString::Format("Received traffic sign map data: %s", data.data()));
            m_strTrafficSignMap.Set(data.data(), data.size());
            RETURN_IF_FAILED(m_mapAnalyzer->analyzeRoadSignXML(m_strTrafficSignMap));
            if (!cEgoState::isInitialized())
                LOG_ERROR("EgoState not initialized!");
            egoState->setParkingSpaces(m_mapAnalyzer->parkingSpaces);
            transmitOpenDriveMapAnalyzerInstance();
        }
    }

    RETURN_NOERROR;
}

tResult fcEnvironmentalModel::transmitOpenDriveMapAnalyzerInstance() {

    LOG_INFO("Transmitting %u junction entries and %u casual lanes.", m_mapAnalyzer->junctionEntries.size(), m_mapAnalyzer->casualLanes.size());
    tUInt64 address = reinterpret_cast<tUInt64>(m_mapAnalyzer);
    LOG_INFO("Address is %u", address);
    //read using vector<tJunctionEntry> *entries = reinterpret_cast<vector<tJunctionEntry> *>(intReader);
    transmitPointerData(address, m_oWriterOpenDriveMapAnalyzerInstanceSynchronization);
    RETURN_NOERROR;
}

tResult fcEnvironmentalModel::transmitEgoStateInstance() {
    tUInt64 address = reinterpret_cast<tUInt64>(egoState);
    return transmitPointerData(address, m_oWriterEgoState);
}

