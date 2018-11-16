#pragma once

#define CID_CARIOSITY_DATA_TRIGGERED_FILTER "fc_sensor_lidar.filter.user.aadc.cid"

#include "../../Helpers/StdFilter.h"
#include "../../Helpers/Median.h"

using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;

// TODO: includes + using namespace

class fcSensorLidar : public cStdFilter {


private:

    // Lidar
    cPinReader m_oReaderLaser;
    cPinWriter m_oWriterLaser;

    adtf::mediadescription::cSampleCodecFactory m_LSStructSampleFactory;

    vector<tPolarCoordiante> sanitize(vector<tPolarCoordiante> scan);

    property_variable<tFloat32> m_propEpsilon = tFloat32(0.5);

public:
    /*! Default constructor. */
    fcSensorLidar();

    /*! Destructor. */
    virtual ~fcSensorLidar() = default;

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