
#pragma once
#include "stdafx.h"

//*************************************************************************************************
#define CID_CARIOSITY_DATA_TRIGGERED_FILTER "birds_eye_transform.filter.user.aadc.cid"

using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;
using namespace cv;


/*! the main class for the lane detection filter. */
class fcBirdsEyeTransform : public cTriggerFunction
{
private:
    //Properties
    /*! Offset of the ROI in the Stream*/
    adtf::base::property_variable<tFloat32> m_ROIOffsetX = 0.25;
    /*! Offset of the ROI in the Stream*/
    adtf::base::property_variable<tFloat32> m_ROIOffsetY = 0.55;
    /*! Width of the ROI*/
    adtf::base::property_variable<tFloat32> m_ROIWidth = 0.5;
    /*! Height of the ROI*/
    adtf::base::property_variable<tFloat32> m_ROIHeight = 0.25;

    adtf::base::property_variable<tFloat32> m_topSideInset = 0.0f;
    adtf::base::property_variable<tFloat32> m_bottomSideInset = 0.4f;
    adtf::base::property_variable<tUInt64> m_OutputHeight = 400;
    adtf::base::property_variable<tBool> m_showROIDebug = tFalse;

    //Pins
    /*! Reader for the video. */
    cPinReader m_oReaderVideo;
    /*! Writer for the video. */
    cPinWriter m_oWriterVideo;

    //Stream Formats
    /*! The input format */
    adtf::streaming::tStreamImageFormat m_InPinVideoFormat;
    /*! The output format */
    adtf::streaming::tStreamImageFormat m_OutPinVideoFormat;

    /*! The clock */
    object_ptr<adtf::services::IReferenceClock> m_pClock;

    /*! lane detection roi bounding rectangle */
    cv::Rect m_LaneRoi = cv::Rect();


    
public:
    /*! Default constructor. */
    fcBirdsEyeTransform();

    /*! Destructor. */
    virtual ~fcBirdsEyeTransform() = default;

    /**
    * Overwrites the Configure
    * This is to Read Properties prepare your Trigger Function
    */
    tResult Configure() override;

    /*!
     * Overwrites the Process You need to implement the Reading and Writing of Samples within this
     * function MIND: Do Reading until the Readers queues are empty or use the
     * IPinReader::GetLastSample()
     * This FUnction will be called if the Run() of the TriggerFunction was called.
     *
     * \param   tmTimeOfTrigger The time time of trigger.
     *
     * \return  Standard Result Code.
     */
    tResult Process(tTimeStamp tmTimeOfTrigger) override;

private:
    
    /*!
    * Change type.
    *
    * \param [in,out]  inputPin    The input pin.
    * \param           oType       The type.
    *
    * \return  Standard Result Code.
    */
    tResult ChangeType(adtf::streaming::cDynamicSampleReader& inputPin,
                       const adtf::streaming::ant::IStreamType& oType);

    /*!
    * Checks if the ROI is within the Image boundaries
    *
    *
    *
    * \return  Standard Result Code.
    */
    tResult checkRoi(void);
};


//*************************************************************************************************
