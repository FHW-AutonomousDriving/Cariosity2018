#pragma once

#define CID_CARIOSITY_DATA_TRIGGERED_FILTER "environmental_model.filter.user.aadc.cid"

#include "stdafx.h"
#include "../Helpers/StdFilter.h"
#include "../Helpers/ddlStructs.h"
#include "Models/EgoState.h"
#include "Models/Map.h"
#include "../Helpers/OpenDriveAnalyzer/OpenDriveMapAnalyzer.h"

using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;
using namespace fhw;

// TODO: includes + using namespace

/**
 * Takes cleaned sensor inputs + relativeRoadPositioning from camera imaging to create an ego and environment model, that is passed on.
 */
class fcEnvironmentalModel : public cStdFilter {

private:
// -------------- Data definitions for position pin --------------

    /*! The ddl indices for a tVehiclePosition */
    struct
    {
        tSize x;
        tSize y;
        tSize radius;
        tSize speed;
        tSize heading;
    } m_ddlPositionIndex;

    /*! The position sample factory */
    adtf::mediadescription::cSampleCodecFactory m_PositionSampleFactory;

// ------------------ Properties ------------------

    adtf::base::property_variable<tBool> m_propEnableConsoleOutput = tTrue;
    adtf::base::property_variable<tFloat32> m_propEmergencyBrakeSecsForObstacleDetection = M_PI; //3.141592653589793f;

// -------------- Pins: --------------

// ------ reader of map data
    cPinReader m_oInput_OpenDriveMap;
    cPinReader m_oInput_TrafficSignMap;


// ------ reader of sensor data
    cPinReader m_oReaderLaserScanner;
    cPinReader m_oReaderUSSideRight;
    cPinReader m_oReaderUSRearRight;
    cPinReader m_oReaderUSRearCenter;
    cPinReader m_oReaderUSRearLeft;
    cPinReader m_oReaderUSSideLeft;

    cPinReader m_oReaderPosition;
    cPinReader m_oReaderTotalDistance;

// ------ reader of lane info data
    cPinReader m_oReaderRelativeLanePositioning;
    cPinReader m_oReaderLanes;

// ------ reader of situation fature data
    cPinReader m_oReaderYOLOObjects;
    cPinReader m_oReaderZebraDetection;

    cPinReader m_oReaderEmergencyBrakeSignal;


// ------ writer

    cPinWriter m_oWriterOpenDriveMapAnalyzerInstanceSynchronization;

    cPinWriter m_oWriterEgoState;

    cPinWriter m_oSampleWriter;

    /*! Writer for the map. */
    cPinWriter m_oWriterVideo;

    /*! The output format */
    adtf::streaming::tStreamImageFormat m_OutPinVideoFormat;

    object_ptr<adtf::services::IReferenceClock> m_pClock;


// ------------ Fields ------------
    cString     m_strOpenDriveMap;
    cString     m_strTrafficSignMap;

    cOpenDriveMapAnalyzer *m_mapAnalyzer;

    tSignalValue m_sigvLastEmergencyBrakeSignal;


// -------------- Processing methods: --------------

    void processLaserScannerData(vector<tPolarCoordiante> data);

    tFloat32 degToRad(tFloat32 angle);

    tResult readPosition(cPinReader &inputPin, tVehiclePosition &positionData);

    tResult handleEmergencyBrakeSignal(tTimeStamp tmTimeOfTrigger);

// ------ Receiving map data ------------

    tResult receiveOpenDriveMap();
    tResult receiveTrafficSignMap();
    tResult transmitOpenDriveMapAnalyzerInstance();
    tResult transmitEgoStateInstance();


public:
    /*! Default constructor. */
    fcEnvironmentalModel();

    /*! Destructor. */
    virtual ~fcEnvironmentalModel() = default;

    /**
    * Overwrites the Configure
    * This is to Read Properties prepare your Trigger Function
    */
    tResult Configure() override;
    /**
    * Overwrites the Process
    * You need to implement the Reading and Writing of Samples within this function
    * MIND: Do Reading until the Readers queues are empty or use the IPinReader::GetLastSample()
    * This FUnction will be called if the Run() of the TriggerFunction was called.
    */
    tResult Process(tTimeStamp tmTimeOfTrigger) override;

};