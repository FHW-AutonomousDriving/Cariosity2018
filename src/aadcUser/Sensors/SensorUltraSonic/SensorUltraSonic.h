#pragma once

#pragma once

#define CID_CARIOSITY_DATA_TRIGGERED_FILTER "fc_sensor_uss.filter.user.aadc.cid"

using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;
#include "../../Helpers/StdFilter.h"
#include "../../Helpers/Median.h"

// TODO: includes + using namespace

class fcSensorUltraSonic : public cStdFilter {

    // TODO:
    // - datatype definitions:
    // uss struct in cm

    // - fields:

    // - input pins:
    // uss in

    // - output pins:
    // uss out

    // - properties:
    // median filter properties

private:

    /*! Reader of an InPin. */
    cPinReader m_oReaderUltraSonic;

    /*! Writer to an OutPin. */
    cPinWriter m_oWriterUltraSonic;
    
    /*! Writers for individual Scanners */
    cPinWriter m_oWriterSideRight; 
    cPinWriter m_oWriterRearRight;
    cPinWriter m_oWriterRearCenter; 
    cPinWriter m_oWriterRearLeft;
    cPinWriter m_oWriterSideLeft;

    // Medians
    cMedian m_mSideRight;
    cMedian m_mRearRight;
    cMedian m_mRearCenter;
    cMedian m_mRearLeft;
    cMedian m_mSideLeft;

    property_variable<tInt> m_propMedianWindowSize = tInt(3);
    property_variable<tFloat32> m_propMaxValidValue = tFloat32(300.0);

    tUltrasonicStruct sanitize(tUltrasonicStruct USSData);
    tBool isValid(tFloat32 value);


public:
    /*! Default constructor. */
    fcSensorUltraSonic();

    /*! Destructor. */
    virtual ~fcSensorUltraSonic() = default;

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