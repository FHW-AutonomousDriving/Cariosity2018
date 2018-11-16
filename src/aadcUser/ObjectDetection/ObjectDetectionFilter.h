//
// Created by aadc on 07.09.18.
//
#pragma once

#include "stdafx.h"
#include "YOLOv3SE.h"
#include "../Helpers/StdFilter.h"
#include "../EnvironmentModel/Models/EgoState.h"
#include "../Helpers/Median.h"

using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;
using namespace cv;
using namespace dnn;

#define CID_CARIOSITY_TIME_TRIGGERED_FILTER "object_detection_filter_yolos.filter.user.aadc.cid"

class fcObjectDetectionFilter : public cStdFilter {

private:
    //Pins
    /*! Input Pin for frame data*/
    cPinReader m_oInputFrame;
    cPinReader m_egoState;

    /*! output pin writer for the out frame */
    cPinWriter m_oOutputFrame;


    //Stream Formats
    /*! The input format */
    adtf::streaming::tStreamImageFormat m_InPinVideoFormat;
    /*! The output format */
    adtf::streaming::tStreamImageFormat m_OutPinVideoFormat;


    /*! The clock */
    object_ptr<adtf::services::IReferenceClock> m_pClock;

    /*! Weight file */
    adtf::base::property_variable<cFilename> m_weights = cFilename("/home/aadc/AADC/src/aadcUser/ObjectDetection/yolov3.weights");
    // names file
    adtf::base::property_variable<cFilename> m_nameList = cFilename("/home/aadc/AADC/src/aadcUser/ObjectDetection/coco.names");
    // config file
    adtf::base::property_variable<cFilename> m_config = cFilename("/home/aadc/AADC/src/aadcUser/ObjectDetection/yolov3.cfg");
    // threshold
    adtf::base::property_variable<tFloat32> m_threshold = 0.80F;
    adtf::base::property_variable<tFloat32> m_thresholdCar = 0.80F;
    // field of view
    adtf::base::property_variable<tFloat32> m_fov = 170.0F;
    adtf::base::property_variable<tBool> m_showFPS = tFalse;

    adtf::base::property_variable<tFloat32> m_yoloAngleEpsilon = 2.5F;
    adtf::base::property_variable<tFloat32> m_maxObjectDistance = 1700.0F;

    adtf::base::property_variable<cFilename> m_pattern = cFilename("/home/aadc/AADC/src/aadcUser/ObjectDetection/patternmax2.png");

    adtf::base::property_variable<tBool> m_debugEmergency = tFalse;

    //used in process
    bool createYOLO = true;
    vector<Scalar> m_colors = {Scalar(255, 0, 0), Scalar(0, 0, 0), Scalar(0, 255, 0)};

    cMedian m_medianEmergencyCar;

public:
    fcObjectDetectionFilter();

    virtual ~fcObjectDetectionFilter() = default;

    tResult Configure() override;

    tResult Process(tTimeStamp tmTimeOfTrigger) override;

    private:

    tResult receiveEgoStateInstance();

    tResult ChangeType(adtf::streaming::cDynamicSampleReader &inputPin, const adtf::streaming::ant::IStreamType &oType);

    tFloat32 checkEmergencyCar(BoxSE box, Mat &img);

    tResult handleDetections(vector<BoxSE> boxes, Mat &img, YOLOv3 &detector);

    tBool checkTemplate(Mat &img, Mat &templ, float threshold, Scalar color, double *maxVal);
};

