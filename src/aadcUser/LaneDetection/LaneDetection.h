/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

**********************************************************************/

#pragma once
#include "stdafx.h"
#include "../EnvironmentModel/Models/EgoState.h"

//*************************************************************************************************
#define CID_CARIOSITY_DATA_TRIGGERED_FILTER "lane_detection.filter.user.aadc.cid"

using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;
using namespace cv;

typedef struct {
    Point2f left;
    Point2f right;
    tFloat32 absoluteAngle;
    tFloat32 relativeAngle;
    tFloat32 curvature;
    tFloat32 distanceToPredecessor;
} tLineDetectionEvaluation;

typedef struct {
    Point2f middle;
    tFloat32 angle;
    tFloat32 length;
    tFloat32 width;

    vector<Point> rotatedBox;
    vector<Point> validArea; // the area in the image which is said to be valid (needs to be a convex polygon)
} tCenterLine;


/*! the main class for the lane detection filter. */
class fcLaneDetection : public cStdFilter
{
private:
    //Properties

    /*! Enabling console output */
    adtf::base::property_variable<tBool> m_propEnableConsoleOutput = tFalse;

    /*! number of detection lines searched in ROI */
    adtf::base::property_variable<tUInt32> m_detectionLines = 40;
    adtf::base::property_variable<tUInt32> m_stepsPerDegree = 15;

    /*! Minimum Line Width in Pixel */
    adtf::base::property_variable<tUInt32> m_minLineWidth = 4;
    /*! Maximum Line Width in Pixel */
    adtf::base::property_variable<tUInt32> m_maxLineWidth = 16;

    /*! Minimum Lane Width in Pixel */
    adtf::base::property_variable<tUInt32> m_minLaneWidth = 90;
    /*! Maximum Lane Width in Pixel */
    adtf::base::property_variable<tUInt32> m_maxLaneWidth = 130;

    adtf::base::property_variable<tUInt32> m_minSearchRadius = 100;
    adtf::base::property_variable<tUInt32> m_maxSearchRadius = 300;

    adtf::base::property_variable<tUInt32> m_minCenterStripeLength = 15;
    adtf::base::property_variable<tUInt32> m_maxCenterStripeLength = 40;


    adtf::base::property_variable<tUInt32> m_maxDetectionDistance = 80;

    adtf::base::property_variable<tFloat32> m_fieldOfView = 100;
    adtf::base::property_variable<tFloat32> m_maxAngleDiff = 20;

    adtf::base::property_variable<tBool> m_showCalibInfo = tFalse;

    adtf::base::property_variable<tBool> m_showLineDetection = tFalse;
    adtf::base::property_variable<tBool> m_showZebraDetection = tFalse;

    // steeringAngle
    property_variable<tFloat32> m_maxSteeringAngle = 30.0f;
    property_variable<tFloat32> m_maxCenterLineOffset = 0.15f;

    //zebra
    adtf::base::property_variable<tUInt32> m_zdetectionLines = 50;
    adtf::base::property_variable<tFloat32> m_cutOff= 0.4f;
    adtf::base::property_variable<tUInt32> m_zminLineWidth = 6;
    adtf::base::property_variable<tUInt32> m_zmaxLineWidth = 12;
    adtf::base::property_variable<tUInt32> m_maxDistanceBetweenBars = 25;
    adtf::base::property_variable<tUInt32> m_amountBars = 4;
    adtf::base::property_variable<tUInt32> m_detectionThreshold = 4;
    adtf::base::property_variable<tUInt32> m_marginTop = 40;

    adtf::base::property_variable<tFloat32> m_percentageOfLaneProbReinforcement = 0.5f;

    //Pins
    cPinReader m_oReaderSteeringIn;
    cPinReader m_oReaderEgoState;

    /*! Reader for the video. */
    cPinReader m_oReaderVideo;
    /*! Writer for the video. */
    cPinWriter m_oWriterVideo;
    cPinWriter m_oWriterDebugVideo;
    /*! Writer pin for lane center split */
    cPinWriter m_oWriterLaneCenterSplit;
    /*! Sends the lanes information: if one exists to the left, to the right or if the car is on one */
    cPinWriter m_oWriterLanes;

    /*! Writer pin for the zebra crossing detection */
    cPinWriter m_oWriterZebraCrossing;

    //Stream Formats
    /*! The input format */
    adtf::streaming::tStreamImageFormat m_InPinVideoFormat;
    /*! The output format */
    adtf::streaming::tStreamImageFormat m_OutPinVideoFormat;
    adtf::streaming::tStreamImageFormat m_OutPinDebugVideoFormat;

    /*! The clock */
    object_ptr<adtf::services::IReferenceClock> m_pClock;

    cv::Size m_inputSize;

    /*! The Median filter  */
    cMedian m_medianLastSteeringAngles;


    /*! the contours we found in this image */
    vector<vector<Point>> m_contours;
    vector<Vec4i> m_contour_hierarchy;

    vector<tCenterLine> m_centerLines;


    // ------ member variables

    tLanes m_lanes;

public:
    /*! Default constructor. */
    fcLaneDetection();

    /*! Destructor. */
    virtual ~fcLaneDetection() = default;

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


    static tFloat32 angleOfLine(Point2f left, Point2f right);
    static tFloat32 calcDistance(Point2f pointA, Point2f pointB);
    static Point2f centerPoint(Point2f pointA, Point2f pointB);
    static Point2f getCartesian(Point2f center, tFloat32 radius, tFloat32 angle);

private:


    /*!
    * Change type.
    *
    * \param [in,out]  inputPin    The input pin.
    * \param           oType       The type.
    *
    * \return  Standard Result Code.
    */
    tResult ChangeType(adtf::streaming::cDynamicSampleReader& inputPin, const adtf::streaming::ant::IStreamType& oType);

    void applyThreshold(Mat &src, Mat &dst);


    vector<tLineDetectionEvaluation> wipe(Mat &orig, Mat &thresholded, const Point2f &center, const tFloat32 heading, const tFloat32 curvature);
    tFloat32 calculateCenterLineOffset(const Point2f &origin, const vector<tLineDetectionEvaluation> &selectedLanes, tFloat32 &confidence);
    //tFloat32 calculateSteeringAngle(tFloat32 centerLineOffset);

    tFloat32 calcAccumulatedError(const Point2f &origin, tFloat32 radius, const vector<Point2f> &centerPoints);
    tFloat32 fitCircle(const Point2f &origin, const vector<Point2f> &centerPoints);

    vector<tFloat32> linspace(tFloat32 min, tFloat32 max, size_t N);

    tBool findZebraCrossing(Mat &orig, Mat &thresholded);

    Point2f transform(const Point2f &origin, const tFloat32 heading, const Point2f &pointToTransform);
    tBool isInFieldOfView(const Point2f &leftAnchor, const Point2f &rightAnchor, const Point2f &point, const tFloat32 heading);
    tBool isValid(const Mat &image, const Point &point);

    tFloat32 getAngle(Point2f anchor, Point2f point);

    tBool containsLaneBorder(Mat &orig, Mat &thresholded, Point2f point1, Point2f point2, Point2f originLaneBorder);
    tFloat32 percentageOfWhiteBetweenPoints(Mat &thresholded, Point2f point1, Point2f point2);

    void calculateLanesProbability(tUInt32 centerLaneDetections, tUInt32 leftLaneDetections, tUInt32 rightLaneDetections);

    tResult receiveEgoStateInstance();


};