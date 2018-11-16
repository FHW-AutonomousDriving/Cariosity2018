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

#include "stdafx.h"
#include "LaneDetection.h"
#include "ADTF3_OpenCV_helper.h"
#include "../Helpers/cariosity_structs.h"

#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/cudaimgproc.hpp>

#include <aadc_geometrics.h>

/// This defines a data triggered filter and exposes it via a plugin class factory.
/// The Triggerfunction cSimpleDataStatistics will be embedded to the Filter
/// and called repeatedly (last parameter of this macro)!
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_CARIOSITY_DATA_TRIGGERED_FILTER,
                                    "Lane Detection",
                                    fcLaneDetection,
                                    adtf::filter::pin_trigger({"in"}));

#define CONSOLE_LOG(_text, _log_level) if (m_propEnableConsoleOutput) { LOG_ADD_ENTRY(_log_level, _text); }    //!< enables log function if console output is activated
#define CONSOLE_LOG_INFO(...) if (m_propEnableConsoleOutput) LOG_ADD_ENTRY(A_UTILS_NS::log::tLogLevel::Info, __VA_ARGS__)


#define ANGLE_AT_FULL_LOCK (34.5f * float(M_PI) / 180.0f)
#define RADIUS_AT_FULL_LOCK (0.40f)
#define PIXEL_PER_METER (100.0f)


fcLaneDetection::fcLaneDetection() : cStdFilter(), m_medianLastSteeringAngles(3) {

    //Register Properties
    RegisterPropertyVariable("Enable console log", m_propEnableConsoleOutput);

    RegisterPropertyVariable("Accuracy: Number of Lines", m_detectionLines);
    RegisterPropertyVariable("Accuracy: Steps per Degree", m_stepsPerDegree);

    RegisterPropertyVariable("minLineWidth [Pixel]", m_minLineWidth);
    RegisterPropertyVariable("maxLineWidth [Pixel]", m_maxLineWidth);

    RegisterPropertyVariable("minimumLaneWidth", m_minLaneWidth);
    RegisterPropertyVariable("maximumLaneWidth", m_maxLaneWidth);

    RegisterPropertyVariable("fieldOfView", m_fieldOfView);
    RegisterPropertyVariable("Maximum Angle Difference between two lines[°]", m_maxAngleDiff);


    RegisterPropertyVariable("minimumSearchRadius", m_minSearchRadius);
    RegisterPropertyVariable("maximumSearchRadius", m_maxSearchRadius);
    RegisterPropertyVariable("minimumCenterStripeLength", m_minCenterStripeLength);
    RegisterPropertyVariable("maximumCenterStripeLength", m_maxCenterStripeLength);

    RegisterPropertyVariable("Maximum Detection Distance for lines [Pixel]", m_maxDetectionDistance);

    RegisterPropertyVariable("Steering: Max distance from center line [m]", m_maxCenterLineOffset);
    RegisterPropertyVariable("Steering: Angle at max distance [°]", m_maxSteeringAngle);

    RegisterPropertyVariable("Show calibration Infos: ", m_showCalibInfo);
    RegisterPropertyVariable("Show Lanedetection Lines (slow!)", m_showLineDetection);
    RegisterPropertyVariable("Show Zebra Lines (annoying!)", m_showZebraDetection);

    RegisterPropertyVariable("Added percentage of detected lane confidence per evaluation (if increase:*1, decrease:*2)", m_percentageOfLaneProbReinforcement);

    RegisterPropertyVariable("_Z: Detection Lines", m_zdetectionLines);
    RegisterPropertyVariable("_Z: Cutoff at sides", m_cutOff);
    RegisterPropertyVariable("_Z: Max bar width", m_zmaxLineWidth);
    RegisterPropertyVariable("_Z: Min bar width", m_zminLineWidth);
    RegisterPropertyVariable("_Z: Max Distance in between", m_maxDistanceBetweenBars);
    RegisterPropertyVariable("_Z: Amount of bars", m_amountBars);
    RegisterPropertyVariable("_Z: Min detected", m_detectionThreshold);
    RegisterPropertyVariable("_Z: Margin top", m_marginTop);

    //create and set inital input format type
    m_InPinVideoFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
    adtf::ucom::object_ptr<IStreamType> pTypeInput = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pTypeInput, m_InPinVideoFormat);
    //Register input pin
    Register(m_oReaderVideo, "in", pTypeInput);
    Register(m_oReaderEgoState, "EgoState_in", getStreamType(POINTER_VALUE));

    //Register output pins
    adtf::ucom::object_ptr<IStreamType> pTypeOutput = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pTypeOutput, m_OutPinVideoFormat);
    Register(m_oWriterVideo, "out", pTypeOutput);
    Register(m_oWriterDebugVideo, "debug_out", pTypeOutput);

    Register(m_oWriterZebraCrossing, "ZebraCrossing", getStreamType(BOOL_SIGNAL_VALUE));

    //register callback for type changes
    m_oReaderVideo.SetAcceptTypeCallback([this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType> &pType) -> tResult {
        return ChangeType(m_oReaderVideo, *pType.Get());
    });


    Register(m_oReaderSteeringIn, "SteeringIn", getStreamType(SIGNAL_VALUE));
    Register(m_oWriterLaneCenterSplit, "LaneCenterSplit", getStreamType(SIGNAL_VALUE));
    m_medianLastSteeringAngles.windowSize = 3;


    Register(m_oWriterLanes, "lanes", getStreamType(LANES));

    // initialize lanes information: 0% left, 100% on, 0% right lane
    m_lanes = {50, 100, 50};
}

tResult fcLaneDetection::Configure() {
    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    RETURN_NOERROR;
}


tResult fcLaneDetection::ChangeType(adtf::streaming::cDynamicSampleReader &inputPin,
                                    const adtf::streaming::ant::IStreamType &oType) {
    if (oType == adtf::streaming::stream_meta_type_image()) {
        adtf::ucom::object_ptr<const adtf::streaming::IStreamType> pTypeInput;
        // get pType from input reader
        inputPin >> pTypeInput;
        adtf::streaming::get_stream_type_image_format(m_InPinVideoFormat, *pTypeInput);

        //set also output format
        adtf::streaming::get_stream_type_image_format(m_OutPinVideoFormat, *pTypeInput);
        //we always have a grayscale output image
        m_OutPinVideoFormat.m_strFormatName = ADTF_IMAGE_FORMAT(GREYSCALE_8);
        m_OutPinDebugVideoFormat.m_strFormatName = ADTF_IMAGE_FORMAT(GREYSCALE_8);
        // and set pType also to samplewriter
        m_oWriterVideo << pTypeInput;
    } else {
        RETURN_ERROR(ERR_INVALID_TYPE);
    }

    RETURN_NOERROR;
}

tResult fcLaneDetection::receiveEgoStateInstance() {

    tPointerValue value;
    RETURN_IF_FAILED(cStdFilter::readPointerData(m_oReaderEgoState, value));

    cEgoState::singleton = reinterpret_cast<cEgoState *>(value.address);
    if (!cEgoState::isInitialized()) {
        LOG_ERROR("Yolo: "
                  ": ERR_NOT_INITIALIZED.");
        //TODO: return cError(ERR_NOT_INITIALIZED);
        RETURN_NOERROR;
    }


    RETURN_NOERROR;
}

static void drawTrajectory(Mat &img, tFloat32 drivingAngle, Point2f center) {

    tFloat32 radius = (ANGLE_AT_FULL_LOCK / drivingAngle) * RADIUS_AT_FULL_LOCK * PIXEL_PER_METER;

    if (abs(radius) < 1e5) {
        center.x += radius;
        circle(img, center, int(abs(radius)), Scalar(80, 120, 200), 1);
    } else {
        line(img, center, {int(center.x), 0}, Scalar(80, 120, 200), 1);
    }

    //ellipse(img, center, Size(int(abs(radius)), int(abs(radius))), M_PI_2, 0, M_PI_2, Scalar(80, 255, 135), 1);
}

tResult fcLaneDetection::Process(tTimeStamp tmTimeOfTrigger) {
    receiveEgoStateInstance();

    if (!cEgoState::singleton->isInitialized()) {
        RETURN_NOERROR;
    }

    tSignalValue steeringValue;
    if (IS_OK(readSignalValue(m_oReaderSteeringIn, steeringValue))) {
        m_medianLastSteeringAngles.pushValue(steeringValue.f32Value / 100.0f * 34.5f * tFloat32(M_PI) / 180.0f);
    }

    object_ptr<const ISample> pReadSample;
    if (IS_OK(m_oReaderVideo.GetNextSample(pReadSample))) {
        object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;
        Mat outputImage, debugOutputImage;
        //lock read buffer
        if (IS_OK(pReadSample->Lock(pReadBuffer))) {
            m_inputSize = cv::Size(m_InPinVideoFormat.m_ui32Width, m_InPinVideoFormat.m_ui32Height);
            const Point2f CENTER = Point2f(m_inputSize.width / 2.0f, m_inputSize.height);

            ///////// READ & FILTER IMAGE ///////

            //create a opencv matrix from the media sample buffer
            Mat inputImage(m_inputSize, CV_8UC3, (uchar *) pReadBuffer->GetPtr());
            Mat thresholded;
            applyThreshold(inputImage, thresholded);
            cvtColor(thresholded, debugOutputImage, COLOR_GRAY2RGB);


            ////////// DO RECOGNITION //////////


            Mat laneBoxMask = Mat::zeros(m_inputSize, CV_8UC1);
            Size laneBoxSize =  Size((m_maxLaneWidth + m_maxLineWidth) * 2, 100);
            // always show near field
            Rect nearField = {Point2f(CENTER.x - ((CAR_WIDTH + laneBoxSize.width) / 2.0f), CENTER.y - laneBoxSize.height), laneBoxSize};
            rectangle(laneBoxMask, nearField, Scalar(255), FILLED);

            if (m_showLineDetection) {
                rectangle(debugOutputImage, nearField, Scalar(255, 0, 0), 1);
            }

            cv::findContours(thresholded, m_contours, m_contour_hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);
            m_centerLines.clear();
            // iterate through all the top-level contours,
            // draw each connected component with its own random color
            for(int idx = 0; idx >= 0; idx = m_contour_hierarchy[idx][0]) {
                vector<Point> contour = m_contours.at(idx);
                Scalar color( rand()&255, rand()&255, rand()&255 );
                tFloat32 perimeter = cv::arcLength(contour, tTrue);
                tFloat32 area = cv::contourArea(contour);
                //drawContours(debugOutputImage, m_contours, idx, color, FILLED, 8, m_contour_hierarchy);
                Point2f points[4];
                cv::minAreaRect(contour).points(points);

                // if it touches the image bound -> dont use it
                if (points[0].y >= m_inputSize.height - 1 || points[1].y >= m_inputSize.height - 1 || points[2].y >= m_inputSize.height - 1 || points[3].y >= m_inputSize.height - 1) continue;
                if (points[0].y <= 0 || points[1].y <= 0 || points[2].y <= 0 || points[3].y <= 0) continue;
                if (points[0].x >= m_inputSize.width - 1 || points[1].x >= m_inputSize.width - 1 || points[2].x >= m_inputSize.width - 1 || points[3].x >= m_inputSize.width - 1) continue;
                if (points[0].x <= 0 || points[1].x <= 0 || points[2].x <= 0 || points[3].x <= 0) continue;


                tFloat32 d01 = calcDistance(points[0],points[1]);
                tFloat32 d12 = calcDistance(points[1],points[2]);
                tFloat32 rectArea = d01 * d12;

                tFloat32 minPerimeter = (m_minCenterStripeLength + m_minLineWidth) * 2;
                tFloat32 maxPerimeter = ((m_maxCenterStripeLength + m_maxLineWidth) * 2) * 1.0f; // factor for eindellungen :-)

                if (perimeter <= maxPerimeter && perimeter >= minPerimeter && rectArea / area < 2.0f) {
                    tFloat32 length = 0;
                    tFloat32 width = 0;
                    tFloat32 angle = 0;
                    if (d01 >= m_minLineWidth && d01 <= m_maxLineWidth) {
                        length = d12;
                        width = d01;
                        angle = angleOfLine(points[0], points[1]);
                    } else if (d12 >= m_minLineWidth && d12 <= m_maxLineWidth) {
                        length = d01;
                        width = d12;
                        angle = angleOfLine(points[1], points[2]);
                    }

                    if (length >= m_minCenterStripeLength && length <= m_maxCenterStripeLength) {

                        // TODO: overlapping boxes
                        Point2f middle = centerPoint(points[0], points[2]);
                        Point2f left = getCartesian(middle, (tFloat32(m_maxLaneWidth) + tFloat32(m_maxLineWidth)), angle - M_PI_2);
                        Point2f topLeft = getCartesian(left, 50.0f, angle);

                        vector<Point> laneBoxPoints;
                        laneBoxPoints.push_back(topLeft);
                        laneBoxPoints.push_back(getCartesian(topLeft, laneBoxSize.width, angle + M_PI_2));
                        laneBoxPoints.push_back(getCartesian(laneBoxPoints[1], -laneBoxSize.height, angle));
                        laneBoxPoints.push_back(getCartesian(topLeft, -laneBoxSize.height, angle));

                        tCenterLine line;
                        line.width = width;
                        line.length = length;
                        line.angle = angle;
                        line.middle = middle;
//                        line.rotatedBox = points; //TODO
                        line.validArea = laneBoxPoints;
                        m_centerLines.push_back(line);

                    } else if (m_showLineDetection) {
                        ostringstream str;
                        str << "l: " << length;
                        putText(debugOutputImage, str.str(), points[0], CV_FONT_HERSHEY_SIMPLEX, 0.25, color, 1, CV_AA);
                    }
                } else if (m_showLineDetection) {
                    ostringstream str;
                    str << "u: " << perimeter;
                    putText(debugOutputImage, str.str(), points[0], CV_FONT_HERSHEY_SIMPLEX, 0.25, color, 1, CV_AA);
                }
            }

            if (!m_centerLines.empty()) {
                sort(m_centerLines.begin(), m_centerLines.end(), [this, CENTER](tCenterLine a, tCenterLine b) {
                    return calcDistance(CENTER, a.middle) < calcDistance(CENTER, b.middle);
                });


                tCenterLine originCenterLine;
                originCenterLine.middle = Point2f(CENTER.x - CAR_WIDTH, CENTER.y);
                originCenterLine.angle = 0;

                tCenterLine &lastCenterLine = originCenterLine;
                for (tSize index = 0; index < m_centerLines.size(); ++index) {
                    tCenterLine &centerLine = m_centerLines.at(index);
                    if (centerLine.validArea.empty()) continue;

                    tFloat32 distanceToLast = calcDistance(lastCenterLine.middle, centerLine.middle);
                    tFloat32 angleDiff = lastCenterLine.angle - centerLine.angle;
                    if (distanceToLast > 200.0f || abs(angleDiff) > (35.0f * M_PI / 180.0f)) {
                        //this box is not a followup
                        if (m_showLineDetection) {
                            {
                                ostringstream str;
                                str << index;
                                putText(debugOutputImage, str.str(), centerLine.middle, CV_FONT_HERSHEY_SIMPLEX, 0.25, Scalar(0, 0, 255), 1, CV_AA);
                            }


                            for (tSize i = 0; i < centerLine.validArea.size(); i++) {
                                line(debugOutputImage,
                                     centerLine.validArea.at(i),
                                     centerLine.validArea.at((i+1) % centerLine.validArea.size()),
                                     Scalar(0, 0, 255),
                                     1,
                                     CV_AA
                                );
                            }
                        }
                        continue;
                    }
                    if (m_showLineDetection) {
                        {
                            ostringstream str;
                            str << index;
                            putText(debugOutputImage, str.str(), centerLine.middle, CV_FONT_HERSHEY_SIMPLEX, 0.25, Scalar(0, 255, 0), 1, CV_AA);
                        }
                        for (tSize i = 0; i < centerLine.validArea.size(); i++) {
                            line(debugOutputImage,
                                 centerLine.validArea.at(i),
                                 centerLine.validArea.at((i + 1) % centerLine.validArea.size()),
                                 Scalar(0, 255, 0),
                                 1,
                                 CV_AA
                            );
                        }

                        circle(debugOutputImage, centerLine.validArea.front(), 2, Scalar(255, 255, 0), FILLED);
                        line(debugOutputImage, centerLine.middle, lastCenterLine.middle, Scalar(0, 200, 200), 1);
                    }

                    fillConvexPoly(laneBoxMask, centerLine.validArea, Scalar(255));
                    lastCenterLine = centerLine;
                }

            }

            Mat zebraThresh = thresholded.clone();

            thresholded &= laneBoxMask;
            cvtColor(thresholded, outputImage, COLOR_GRAY2RGB);


            //////// FIND ZEBRA CROSSING ////////

            tBool zebraDetected = findZebraCrossing(outputImage, zebraThresh);


            //////////////// WIPE ///////////////

            static tFloat32 lastHeading = 0.0f;
            tFloat32 currentHeading = m_medianLastSteeringAngles.calculateMedian();
            tFloat32 curvature = currentHeading - lastHeading;

            tBool overrideFOV =  cEgoState::singleton->getOverrideLaneDetectionAngleOfFielOfView();
            tFloat32 AngleOfFieldOfView = overrideFOV ? cEgoState::singleton->getLaneDetectionAngleOfFieldOfView() : lastHeading ;
            //cv::findContours(thresholded, m_contours, m_contour_hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE); // refind contour on new image
            vector<tLineDetectionEvaluation> selectedLanes = wipe(outputImage, thresholded, CENTER, AngleOfFieldOfView, curvature);

            //vector<tLineDetectionEvaluation> selectedLanes = wipe(outputImage, thresholded, CENTER, lastHeading, curvature);

            tFloat32 numberOfLinesInMaximumDetectionDistance = 0;

            // use last angle if we found too few lanes
            tFloat32 drivingAngleRad = currentHeading;
            if (selectedLanes.size() > 4) {
                drivingAngleRad = calculateCenterLineOffset(CENTER, selectedLanes, numberOfLinesInMaximumDetectionDistance);
            } else {
                CONSOLE_LOG_INFO("Using last steering angle: %.2f", drivingAngleRad * 180.0f / tFloat32(M_PI));
            }
            /*
            tFloat32 centerLineOffset = calculateCenterLineOffset(CENTER, selectedLanes, numberOfLinesInMaximumDetectionDistance);
            tFloat32 drivingAngleRad = calculateSteeringAngle(centerLineOffset);
             */

            lastHeading = currentHeading;

            tFloat32 drivingAngle = drivingAngleRad * 180.0f / tFloat32(M_PI) / 34.5f * 100.0f;
            CONSOLE_LOG_INFO("driving Angle  %f (Percent)", drivingAngle);


            //////////   TRANSMITTING   /////////

            tBoolSignalValue zebraDetectedBoolSignalValue = {tUInt32(tmTimeOfTrigger), zebraDetected};
            tSignalValue laneCenterSplitSignalValue = {tUInt32(numberOfLinesInMaximumDetectionDistance), drivingAngle};

            transmitSignalValue(laneCenterSplitSignalValue, m_oWriterLaneCenterSplit);
            transmitBoolSignalValue(zebraDetectedBoolSignalValue, m_oWriterZebraCrossing);


            /////////////////////////////////////

            if (m_showLineDetection) {
                // Driving angle
                //line(outputImage, CENTER, getCartesian(CENTER, 50, drivingAngleRad), Scalar(0, 0, 255), 2, 1);
                drawTrajectory(outputImage, drivingAngleRad, CENTER);

                {
                    ostringstream str;
                    str << cString::Format("Left: %2.0f", m_lanes.fHasLaneToTheLeft) << "%";
                    putText(outputImage, str.str(), Point(20, outputImage.rows - 20), CV_FONT_HERSHEY_SIMPLEX, 0.7, Scalar(230, 230, 0), 1, CV_AA);
                }
                {
                    ostringstream str;
                    str << cString::Format("Right: %2.0f", m_lanes.fHasLaneToTheRight) << "%";
                    putText(outputImage, str.str(), Point(outputImage.cols - 150, outputImage.rows - 20), CV_FONT_HERSHEY_SIMPLEX, 0.7, Scalar(230, 230, 0), 1, CV_AA);
                }

            }

            pReadBuffer->Unlock();
        }
        //copy to outputimage

        //Write processed Image to Output Pin
        if (!outputImage.empty()) {
            //update output format if matrix size does not fit to
            if (outputImage.total() * outputImage.elemSize() != m_OutPinVideoFormat.m_szMaxByteSize) {
                setTypeFromMat(m_oWriterVideo, outputImage);
            }
            // write to pin
            writeMatToPin(m_oWriterVideo, outputImage, m_pClock->GetStreamTime());
        }
        //Write processed Image to Output Pin
        if (!debugOutputImage.empty()) {
            //update output format if matrix size does not fit to
            if (debugOutputImage.total() * debugOutputImage.elemSize() != m_OutPinDebugVideoFormat.m_szMaxByteSize) {
                setTypeFromMat(m_oWriterDebugVideo, debugOutputImage);
            }
            // write to pin
            writeMatToPin(m_oWriterDebugVideo, debugOutputImage, m_pClock->GetStreamTime());
        }

    }
    RETURN_NOERROR;
}

void fcLaneDetection::applyThreshold(Mat &src, Mat &dst) {
    cv::cuda::GpuMat gpuInput, gpuOutput;
    cuda::Stream stream;

    // upload to gpu and apply the bilateral blurring algorithm on a grayscaled image
    gpuInput.upload(src, stream);
    cv::cuda::cvtColor(gpuInput, gpuOutput, COLOR_BGR2GRAY, 0, stream);
    cuda::bilateralFilter(gpuOutput, gpuOutput, 5, 15, 15, BORDER_DEFAULT, stream);
    gpuOutput.download(dst, stream);
    stream.waitForCompletion();

    // then make it a binary image
    adaptiveThreshold(dst, dst, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 101, -20);

}

tFloat32 fcLaneDetection::calculateCenterLineOffset(const Point2f &origin, const vector<tLineDetectionEvaluation> &selectedLines, tFloat32 &confidence) {
    tUInt32 numberOfLinesInMaximumDetectionDistance = 0;

// TODO: does not work if large parts are 'obfuscated'
    ostringstream strstream;
    vector<tFloat32> lineAngles;
    tFloat32 steering = 0;
    tFloat32 totalDistance = -tFloat32(m_minSearchRadius);
    for (const tLineDetectionEvaluation &line: selectedLines) {
        if (totalDistance > m_maxDetectionDistance) break;

        numberOfLinesInMaximumDetectionDistance++;
        totalDistance += line.distanceToPredecessor;

        tFloat32 angleToOrigin = getAngle(origin, centerPoint(line.left, line.right));
        strstream << (angleToOrigin * 180.0f / M_PI) << " ";

        lineAngles.push_back(angleToOrigin);
        steering += angleToOrigin * max(0.0f, (m_maxDetectionDistance - max(0.0f, totalDistance)) / m_maxDetectionDistance);

    }

    steering = lineAngles.empty() ? 0 : steering / lineAngles.size();
    if (numberOfLinesInMaximumDetectionDistance < 5) {
        vector<Point2f> centerPoints;
        for (tSize i = 0; i < 5 && i < selectedLines.size(); i++) {
            auto &line = selectedLines.at(i);
            centerPoints.push_back(centerPoint(line.left, line.right));
        }

        tFloat32 radius = fitCircle(origin, centerPoints);
        steering = ANGLE_AT_FULL_LOCK * (radius / RADIUS_AT_FULL_LOCK) * PIXEL_PER_METER;
    }

    CONSOLE_LOG_INFO("Steering: %s-> %.2f", strstream.str().c_str(), (steering * 180.0f / M_PI));

    confidence = numberOfLinesInMaximumDetectionDistance;
    return steering;
}


tFloat32 fcLaneDetection::calcAccumulatedError(const Point2f &origin, tFloat32 radius, const vector<Point2f> &centerPoints) {

    tFloat32 accumulatedError = 0;
    for (const auto &centerOfLine: centerPoints) {
        accumulatedError += abs(calcDistance(origin, centerOfLine) - radius);
    }
    return accumulatedError;
}

tFloat32 fcLaneDetection::fitCircle(const Point2f &origin, const vector<Point2f> &centerPoints) {

    vector<tFloat32> angles = linspace(tFloat32(-34.5f * M_PI / 180.0f), tFloat32(34.5f * M_PI / 180.0f), 201);
    vector<tFloat32> radii;
    for (const auto &angle: angles) {
        if (abs(angle) < 1e-5) {
            tFloat32 radius = (ANGLE_AT_FULL_LOCK / angle) * RADIUS_AT_FULL_LOCK * PIXEL_PER_METER;
            radii.push_back(radius);
        }
    }

    tFloat32 bestRadius = 0;
    tFloat32 lowestError = calcAccumulatedError(origin, 0, centerPoints);

    for (const auto &radius: radii) {
        tFloat32 error = calcAccumulatedError(origin, radius, centerPoints);
        if (error < lowestError) {
            lowestError = error;
            bestRadius = radius;
        }
    }

    CONSOLE_LOG_INFO("Steering: fitCircle -> radius: %.2f", bestRadius);
    return bestRadius;
}

// ------ helper methods ------

tBool fcLaneDetection::findZebraCrossing(Mat &orig, Mat &thresholded) {
    int detectionLines = m_zdetectionLines;
    int distBetweenLines = orig.rows / detectionLines;

    float cutOffLeft = m_cutOff;
    float cutOffRight = 1 - cutOffLeft;

    int leftBound = int(orig.cols * cutOffLeft);
    int rightBound = int(orig.cols * cutOffRight);

    int minLineWidth = m_zminLineWidth;
    int maxLineWidt = m_zmaxLineWidth;
    int maxDistanceBetweenBars = m_maxDistanceBetweenBars;
    int amountBars = m_amountBars;
    int detectionThreshold = m_detectionThreshold;

    int marginTop = m_marginTop;

    vector<vector<Point>> lines;
    for (int y = marginTop; y < detectionLines; y++) { //ignore case, thus leave out five at the bottom
        Point left = {0, distBetweenLines * y};
        Point right = {orig.cols, distBetweenLines * y};

        if(m_showZebraDetection) {
            line(orig, right, left, Scalar(255, 255, 0));
        }

        vector<Point> points;
        uchar lastBrightness = 0;
        Point startPoint = {0, 0};
        bool startDetected = false;

        for (int x = leftBound; x < rightBound; x++) {
            Point currentPoint = {x, (y * distBetweenLines)};
            uchar currentBrightness = thresholded.at<uchar>(currentPoint.y, currentPoint.x);

            if (currentBrightness > lastBrightness) {
                startDetected = true;
                startPoint = currentPoint;
            } else if (startDetected && lastBrightness > currentBrightness) {
                tFloat32 distance = calcDistance(startPoint, currentPoint);
                Point p = centerPoint(startPoint, currentPoint);
                if (distance > minLineWidth && distance < maxLineWidt) {
                    //LOG_INFO("Accepted line width: %f", distance);
                    points.push_back(p);
                    startDetected = false;
                    startPoint = {0, 0};
                }
            }
            if (startDetected && calcDistance(startPoint, currentPoint) > maxDistanceBetweenBars) {
                startDetected = false;
                startPoint = {0,0};
            }
            lastBrightness = currentBrightness;
        }

        int arrSize = int(points.size());
        vector<Point> cleaned;
        if(arrSize >= amountBars) {
            arrSize--;
            for (int i = 0; i < arrSize; i++) {
                if (calcDistance(points.at(i), points.at(i + 1)) <= maxDistanceBetweenBars) {
                    cleaned.push_back(points.at(i));
                    //LOG_INFO("Distance: %f", calcDistance(points.at(i), points.at(i+1)));
                    if(m_showZebraDetection) {
                        circle(orig, points.at(arrSize), 2, Scalar(0, 255, 128), -1);
                    }
                }

            }
            if(calcDistance(points.at(arrSize), points.at(arrSize - 1)) <= maxDistanceBetweenBars) {
                cleaned.push_back(points.at(arrSize));
                if(m_showZebraDetection) {
                    circle(orig, points.at(arrSize), 2, Scalar(0, 255, 128), -1);
                }
            }
            if (int(cleaned.size()) >= 1) {
                lines.push_back(cleaned);
            }
        }
    }

    tBool result = int(lines.size()) >= detectionThreshold;
    return result;
}

Point2f fcLaneDetection::getCartesian(Point2f center, tFloat32 radius, tFloat32 angle) {
    tFloat32 x = (center.x + sin(angle) * radius);
    tFloat32 y = (center.y - cos(angle) * radius);

    return Point2f(x, y);
}

tFloat32 fcLaneDetection::getAngle(Point2f anchor, Point2f point) {
    tFloat32 dy = anchor.y - point.y; // y is pointing down
    tFloat32 dx = point.x - anchor.x;

    return atan2(dx, dy);
}

tBool fcLaneDetection::isValid(const Mat &image, const Point &point) {
    return (point.x >= 0 && point.y >= 0 && point.x < image.cols && point.y < image.rows);
}

tBool fcLaneDetection::isInFieldOfView(const Point2f &leftAnchor, const Point2f &rightAnchor, const Point2f &point, const tFloat32 heading) {
    const auto halfFieldOfView = tFloat32((m_fieldOfView / 2.0f) * M_PI / 180.0f);
    const auto MIN_ANGLE = heading - halfFieldOfView;
    const auto MAX_ANGLE = heading + halfFieldOfView;

    return (getAngle(leftAnchor, point) > MIN_ANGLE && getAngle(rightAnchor, point) < MAX_ANGLE);
}

Point2f fcLaneDetection::centerPoint(Point2f pointA, Point2f pointB) {
    tFloat32 x = pointA.x + (pointB.x - pointA.x) / 2.0f;
    tFloat32 y = pointA.y + (pointB.y - pointA.y) / 2.0f;
    return Point2f(x, y);
}

tFloat32 fcLaneDetection::calcDistance(Point2f pointA, Point2f pointB) {
    return tFloat32(sqrt(pow((pointA.x - pointB.x), 2) + pow((pointA.y - pointB.y), 2)));
}

tFloat32 fcLaneDetection::angleOfLine(Point2f left, Point2f right) {
    tFloat32 dx = (left.x - right.x);
    tFloat32 dy = (left.y - right.y);
    if (dx == 0.0f) {
        return 0.0f;
    }
    return atan(dy / dx);
}

vector<tFloat32> fcLaneDetection::linspace(tFloat32 min, tFloat32 max, size_t N) {
    vector<tFloat32> range;
    tFloat32 delta = (max - min) / tFloat32(N - 1);
    for (unsigned int i = 0; i < N; i++) {
        range.push_back(min + i * delta);
    }
    return range;
}

//TODO: ROI von birdseye so anpassen, dass die motorhaube nicht mehr im bild ist? -> weniger pixel = schneller?
vector<tLineDetectionEvaluation> fcLaneDetection::wipe(Mat &orig, Mat &thresholded, const Point2f &center, const tFloat32 heading, const tFloat32 curvature) {
    vector<tLineDetectionEvaluation> selectedLanes;

    const tUInt32 MIN_RADIUS = m_minSearchRadius;
    const tUInt32 MAX_RADIUS = min(tUInt32(m_maxSearchRadius), tUInt32(max(abs(orig.cols - center.x), abs(orig.rows - center.x))));

    const auto halfFieldOfView = tFloat32((m_fieldOfView / 2.0f) * M_PI / 180.0f);
    const auto MIN_ANGLE = heading - halfFieldOfView;
    const auto MAX_ANGLE = heading + halfFieldOfView;

    const tUInt32 ANGLE_STEPS = tUInt32(180 * m_stepsPerDegree);

    // lanes calculation stuff
    const tUInt32 MIN_LANEPOINTS_FOR_SIDELANE = 18; //TODO: make property?

    tUInt32 centerLaneDetections = 0;
    tUInt32 leftLaneDetections = 0;
    tUInt32 rightLaneDetections = 0;

    //CONSOLE_LOG_INFO("heading: %f | min: %f | max: %f", heading, MIN_ANGLE, MAX_ANGLE);


    Point2f leftAnchor = Point2f(center.x - m_maxLaneWidth / 2.0f, center.y);
    Point2f rightAnchor = Point2f(center.x + m_maxLaneWidth / 2.0f, center.y);

    Point2f lastPoint = center;


    vector<tFloat32> radii = linspace(MIN_RADIUS, MAX_RADIUS, m_detectionLines);
    vector<tFloat32> angles = linspace(-tFloat32(M_PI_2), tFloat32(M_PI_2), size_t(ANGLE_STEPS));
    //TODO: CALC Starting angle the right way (instead of just adding 0.5)l


    for (const tFloat32 radius : radii) {

        //circle(orig, center, int(radius), Scalar(255, 0, 0));

        vector<Point> points;
        uchar lastBrightness = 255;
        Point startPoint = {0, 0};
        bool startDetected = false;

        for (const tFloat32 angle : angles) {
            Point currentPoint = getCartesian(center, radius, angle);

            if (!isValid(orig, currentPoint) || !isInFieldOfView(leftAnchor, rightAnchor, currentPoint, heading)) continue;
            uchar currentBrightness = thresholded.at<uchar>(currentPoint.y, currentPoint.x);

            if (currentBrightness > lastBrightness) {
                startDetected = true;
                startPoint = currentPoint;
            } else if (startDetected && lastBrightness > currentBrightness) {
                tFloat32 distance = calcDistance(startPoint, currentPoint);
                Point p = centerPoint(startPoint, currentPoint);
                if (distance > m_minLineWidth && distance < m_maxLineWidth) {
                    if (m_showCalibInfo) {
                        LOG_INFO("Accepted line width: %f", distance);
                    }
                    points.push_back(p);

                    if (m_showCalibInfo || m_showLineDetection) {
                        circle(orig, p, 2, Scalar(0, 255, 0), -1);
                    }

                    startDetected = false;
                    startPoint = {0, 0};
                } else {
                    if (m_showCalibInfo || m_showLineDetection) {
                        circle(orig, p, 2, Scalar(255, 0, 0), -1);
                    }
                }
            }
            //we reached maximum line width limit, stop looking for end of line
            if (startDetected && calcDistance(startPoint, currentPoint) > m_maxLineWidth) {
                startDetected = false;
                startPoint = {0, 0};
            }

            lastBrightness = currentBrightness;
        }



        // Select all lines which fit

        if (points.size() >= 2) {

            vector<tLineDetectionEvaluation> lanes;
            for (unsigned int iA = 0; iA < points.size(); iA++) {
                Point2f left = points.at(iA);

                // get contour of point a
                vector<Point> contour;
                for(int idx = 0; idx >= 0; idx = m_contour_hierarchy[idx][0]) {
                    vector<Point> cnt = m_contours.at(idx);

                    if (cv::pointPolygonTest(cnt, left, tFalse) >= 0) {
                        contour = cnt;
                        break;
                    }
                }

                for (unsigned int iB = iA + 1; iB < points.size(); iB++) {
                    Point2f right = points.at(iB);


                    // check if point B is on same contour -> throw it away
                    //if (!contour.empty() && cv::pointPolygonTest(contour, right, tFalse) >= 0) continue;

                    tFloat32 distance = calcDistance(right, left);

                    if (distance < m_maxLaneWidth && distance > m_minLaneWidth) {

                        // discard lines that have too much white between end points

                        tFloat32 whiteInsideRatio = percentageOfWhiteBetweenPoints(thresholded, left, right);
                        if (whiteInsideRatio >= 0.85f) continue;


                        tFloat32 absoluteAngle = angleOfLine(left, right);

                        // calculate with respect to the angle of the last line
                        tFloat32 lastAngle = selectedLanes.empty() ? heading : selectedLanes.back().absoluteAngle;
                        Point2f relativeRight = transform(lastPoint, lastAngle, right);
                        Point2f relativeLeft = transform(lastPoint, lastAngle, left);


                        tFloat32 lastRelativeAngle = selectedLanes.empty() ? heading : selectedLanes.back().relativeAngle;
                        tFloat32 relativeAngle = angleOfLine(relativeLeft, relativeRight);
                        tFloat32 currentCurvature = relativeAngle - lastRelativeAngle;

                        Point2f relativeCenterOfLine = centerPoint(relativeLeft, relativeRight);

                        tFloat32 max_dx = abs(tan(m_fieldOfView / 2.0f * M_PI / 180.0f) * relativeCenterOfLine.y);
                        tFloat32 dx = abs(relativeCenterOfLine.x);
                        tFloat32 dy = abs(relativeCenterOfLine.y);

                        if (abs(relativeAngle) <= m_maxAngleDiff && dx <= max_dx) {
                            if(m_showCalibInfo) {
                               LOG_INFO("distance %f || angle %f || last Angle diff: %f ", distance, relativeAngle, selectedLanes.empty() ? 0.0f : selectedLanes.back().relativeAngle - relativeAngle);
                            }

                            //Found possible candidate
                            lanes.push_back({left, right, absoluteAngle, relativeAngle, currentCurvature, calcDistance(relativeCenterOfLine, {0, 0})});

                        } else if (abs(relativeAngle) > m_maxAngleDiff) {
                            if (m_showCalibInfo) {
                                LOG_INFO("Discarding: angleDiff: %f", relativeAngle);
                                line(orig, right, left, Scalar(255, 255, 0)); // Yellow
                            }
                        } else if (dx > max_dx) {
                            if (m_showCalibInfo) {
                                LOG_INFO("Discarding: dx: %f, max dx: %f", dx, max_dx);
                                line(orig, right, left, Scalar(255, 0, 255)); // Purple
                            }
                        } else {
                            if (m_showCalibInfo) {
                                LOG_INFO("Discarding: dy: %f", dy);
                                line(orig, right, left, Scalar(255, 127, 0)); // Orange
                            }
                        }
                    } else if (distance < m_maxLaneWidth * 1.6f) { // *1.6f??
                        if (m_showCalibInfo) {
                            line(orig, right, left, Scalar(175, 80, 116)); // Plum
                        }
                    } else {
                        if (m_showCalibInfo) {
                            LOG_INFO("Discarding, line too long! %f", distance);
                        }
                    }
                }
            }

            // Select best fitting lane
            if (!lanes.empty()) {
                tLineDetectionEvaluation bestLane = lanes.front();
                tFloat32 lowestCurvature = 999999999.0f;

                tFloat32 lastCurvature = curvature;
                tFloat32 lastRelativeAngle = selectedLanes.empty() ? heading : selectedLanes.back().relativeAngle;

                tFloat32 leastEndingsDistance = 999999999.0f;

                for (tLineDetectionEvaluation lane : lanes) {
                    tBool selectByAngleDiff = tTrue;

                    if(selectByAngleDiff) { // select best lane based on least change in angle
                        Point2f p = centerPoint(lane.left, lane.right);
                        tFloat32 distance = calcDistance(lastPoint, p);

                        // calculate curvature
                        tFloat32 changeOfCurvature = (lane.curvature - lastCurvature) * 100;

                        if (abs(changeOfCurvature) < 15.0f) {
                            if (abs(changeOfCurvature) < lowestCurvature) {
                                bestLane = lane;
                                lowestCurvature = changeOfCurvature;
                            }
                            //LOG_INFO("changeOfCurvature: \t%f", changeOfCurvature);
                        } else {
                            //LOG_INFO("Curvature: \t%f, lastCurv: \t%f, changeOfCurvature: \t%f, distance: \t%f, d²: \t%f", lane.curvature, lastCurvature, changeOfCurvature, distance, (distance * distance));
                            if (m_showLineDetection) {
                                line(orig, lastPoint, centerPoint(lane.left, lane.right), Scalar(255, 112, 11), 2);
                            }
                        }
                    }
                    else { // select based on last distance to lane endings
                        Point2f lastLeft;
                        Point2f lastRight;
                        if(!selectedLanes.empty()) {
                            lastLeft = selectedLanes.back().left;
                            lastRight = selectedLanes.back().right;
                        }
                        else { // first detected lane
                            tFloat32 avgLaneWidth = (m_maxLaneWidth + m_minLaneWidth) / 4.0f;
                            lastLeft.y = orig.rows - m_minSearchRadius + 10;
                            lastLeft.x = orig.cols / 2 - avgLaneWidth;
                            lastRight.y = orig.rows - m_minSearchRadius + 10;
                            lastRight.x = orig.cols / 2 + avgLaneWidth;
                            if (/*m_showLineDetection*/ true) {
                                circle(orig, lastLeft, 2, Scalar(255, 255, 0));
                                circle(orig, lastRight, 2, Scalar(255, 255, 0));
                            }
                        }

                        tFloat32 distLeftEnding = calcDistance(lastLeft, lane.left);
                        tFloat32 distRightEnding = calcDistance(lastRight, lane.right);

                        tFloat32 endingsDistance = distLeftEnding + distRightEnding;
                        if(endingsDistance < leastEndingsDistance) {
                            leastEndingsDistance = endingsDistance;
                            bestLane = lane;
                        }
                    }

                }

                selectedLanes.push_back(bestLane);
                lastCurvature = bestLane.curvature;

                Point2f currentPoint = centerPoint(bestLane.left, bestLane.right);
                if (m_showLineDetection) {
                    line(orig, lastPoint, currentPoint, Scalar(0, 0, 255), 1);
                    line(orig, bestLane.left, bestLane.right, Scalar(255, 0, 255), 1); // Purple
                }

                lastPoint = currentPoint;


                // TODO lanes: check if angle too large for center lane: mark as left or right
            }

            // ------ LANES: detecting left or right lanes present ...
            if(!lanes.empty()) {

                tLineDetectionEvaluation lastLane = lanes.back();
                centerLaneDetections++;

                tFloat32 leftAngle = lastLane.absoluteAngle - float(M_PI_2);
                Point2f leftOf_maxDist = getCartesian(lastLane.left, m_maxLaneWidth, leftAngle);
                Point2f leftOf_minDist = getCartesian(lastLane.left, m_minLaneWidth, leftAngle);

                if (containsLaneBorder(orig, thresholded, leftOf_maxDist, leftOf_minDist, lanes.back().left)) {
                    leftLaneDetections++;
                }


                tFloat32 rightAngle = lastLane.absoluteAngle + float(M_PI_2);
                Point2f rightOf_maxDist = getCartesian(lanes.back().right, m_maxLaneWidth, rightAngle);
                Point2f rightOf_minDist = getCartesian(lanes.back().right, m_minLaneWidth, rightAngle);

                if (containsLaneBorder(orig, thresholded, rightOf_maxDist, rightOf_minDist, lanes.back().right)) {
                    rightLaneDetections++;
                }

            }

        }

        // calculate if there is a left or a right lane present (if more than min linepoints per side present)
        calculateLanesProbability(centerLaneDetections, leftLaneDetections, rightLaneDetections);
        //LOG_INFO("Found lanes -- left: %d, right: %d, center: %d", leftLaneDetections, rightLaneDetections, centerLaneDetections);

    }

    transmitLanesData(m_lanes, m_oWriterLanes);

    if (m_showLineDetection) {
        line(orig, leftAnchor, getCartesian(leftAnchor, MAX_RADIUS, MIN_ANGLE), Scalar(0, 255, 0));
        line(orig, rightAnchor, getCartesian(rightAnchor, MAX_RADIUS, MAX_ANGLE), Scalar(0, 255, 0));
        circle(orig, center, MAX_RADIUS, Scalar(0, 255, 0));
        circle(orig, center, MIN_RADIUS, Scalar(0, 255, 0));
    }

    return selectedLanes;
}

tFloat32 fcLaneDetection::percentageOfWhiteBetweenPoints(Mat &thresholded, Point2f point1, Point2f point2){
    size_t detectionPointsBetweenLaneBorders = 40;
    vector<tFloat32> xPositions = linspace(point1.x, point2.x, detectionPointsBetweenLaneBorders);
    vector<tFloat32> yPositions = linspace(point1.y, point2.y, detectionPointsBetweenLaneBorders);

    tUInt32 whitePxlCount = 0;

    for (tUInt16 i = 0; i < xPositions.size(); i++) {
        Point2f currentPoint = Point2f(xPositions.at(i), yPositions.at(i));
        if (isValid(thresholded, currentPoint) && thresholded.at<uchar>(int(currentPoint.y), int(currentPoint.x)) >= 1) {
            whitePxlCount++;
        }
    }

    return tFloat32(whitePxlCount) / tFloat32(detectionPointsBetweenLaneBorders);
}

tBool fcLaneDetection::containsLaneBorder(Mat &orig, Mat &thresholded, Point2f point1, Point2f point2, Point2f originLaneBorder) {
    tUInt8 sideDetectionPoints = 20;

    vector<tFloat32> xPositions = linspace(point1.x, point2.x, sideDetectionPoints);
    vector<tFloat32> yPositions = linspace(point1.y, point2.y, sideDetectionPoints);

    uchar lastBrightness = 0;
    Point startPoint = {0, 0};
    bool startDetected = false;

    tUInt32 pos = 0;
    bool lineFound = false;

    while (pos < xPositions.size() && !lineFound) {
        Point2f currentPoint = Point2f(xPositions.at(pos), yPositions.at(pos));

        if (!isValid(orig, currentPoint)) {
            pos++;
            continue;
        }

        uchar currentBrightness = thresholded.at<uchar>(currentPoint.y, currentPoint.x);

        if (currentBrightness > lastBrightness) {
            // start detected
            startDetected = true;
            startPoint = currentPoint;
        } else if (startDetected && lastBrightness > currentBrightness) {
            // end detected
            tFloat32 distance = calcDistance(startPoint, currentPoint);
            Point p = centerPoint(startPoint, currentPoint);
            if (distance > m_minLineWidth && distance < m_maxLineWidth) {
                // line of an acceptable width detected
                if (m_showCalibInfo) {
                    LOG_INFO("Accepted line width: %f", distance);
                }
                if (m_showCalibInfo || m_showLineDetection) {
                    circle(orig, p, 2, Scalar(0, 102, 153), 1);             // blue
                    line(orig, p, originLaneBorder, Scalar(0, 102, 153));   // blue
                }

                startDetected = false;
                startPoint = {0, 0};
                lineFound = true;
            }
        }
        //we reached maximum line width limit, stop looking for end of line
        if (startDetected && calcDistance(startPoint, currentPoint) > m_maxLineWidth) {
            startDetected = false;
            startPoint = {0, 0};
        }

        lastBrightness = currentBrightness;
        pos++;
    }

    return lineFound;
}


Point2f fcLaneDetection::transform(const Point2f &origin, const tFloat32 heading, const Point2f &pointToTransform) {

    tFloat32 dx = pointToTransform.x - origin.x;
    tFloat32 dy = pointToTransform.y - origin.y;
    //tFloat32 dPhi = target.f32heading - currentPos.f32heading;

    // coordinates relative to our current heading
    tFloat32 dxStrich = dx * cos(heading) + dy * sin(heading);
    tFloat32 dyStrich = dx * -sin(heading) + dy * cos(heading);

    return { dxStrich, dyStrich };
}

void fcLaneDetection::calculateLanesProbability(tUInt32 centerLaneDetections, tUInt32 leftLaneDetections, tUInt32 rightLaneDetections)
{
    // add or subtract confidence percent value, if enough lanepoints detected
    //tFloat32 centerLaneConfidence = m_lanes.fIsOnLane + (centerLaneDetections >= m_minDetectionPointsForLane ? 2 : -1) * m_percentageOfLaneProbReinforcement;
    /*
    tFloat32 leftLaneConfidence = m_lanes.fHasLaneToTheLeft + (leftLaneDetections >= m_minDetectionPointsForLane? 2 : -0.5f) * m_percentageOfLaneProbReinforcement;
    tFloat32 rightLaneConfidence = m_lanes.fHasLaneToTheRight + (rightLaneDetections >= m_minDetectionPointsForLane ? 2 : -1) * m_percentageOfLaneProbReinforcement;
    */
    tFloat32 leftLaneConfidence = 0.0f;
    tFloat32 rightLaneConfidence = 0.0f;

    tFloat32 leftLaneWeightIncrease = 0.0f;
    tFloat32 rightLaneWeightIncrease = 0.0f;

    if(centerLaneDetections != 0) { // [0.0 - 1.0] => [-1, +1] for weighing added max percentage change
        // calculating percentage of recognition as value between 0 and 1
        leftLaneConfidence = (tFloat32(leftLaneDetections) / tFloat32(centerLaneDetections)); //* 2.0f - 1.0f;
        rightLaneConfidence = (tFloat32(rightLaneDetections) / tFloat32(centerLaneDetections)); //* 2.0f - 1.0f;
    }

    // calculating weight for added percentage value

    // sigmoid
    // leftLaneConfidence = 2.0f/(1+exp(-20.0f*(leftLaneConfidence-0.5f))) - 1.0f;
    // rightLaneConfidence = 2.0f/(1+exp(-20.0f*(rightLaneConfidence-0.5f))) - 1.0f;

    // linear
    leftLaneWeightIncrease = leftLaneConfidence <= 0.4f ? -1.0f : (leftLaneConfidence <= 0.6f ? 0.5f : 1.0f );
    rightLaneWeightIncrease = rightLaneConfidence <= 0.4f ? -2.0f : (rightLaneConfidence <= 0.6f ? 0.5f : 1.0f );

    // calculate increase/ decrease of detected lane in percent
    // left decreased more slowly, so the crossings won't lead to lane switching to false right

    //tFloat32 leftLaneConfidenceIncrease = (leftLaneConfidence > 0.0f ? leftLaneConfidence : leftLaneConfidence * 0.5f) * m_percentageOfLaneProbReinforcement;
    //tFloat32 rightLaneConfidenceIncrease = rightLaneConfidence * m_percentageOfLaneProbReinforcement;

    tFloat32 leftLaneConfidenceIncrease = leftLaneWeightIncrease * m_percentageOfLaneProbReinforcement;
    tFloat32 rightLaneConfidenceIncrease = rightLaneWeightIncrease * m_percentageOfLaneProbReinforcement;

    m_lanes.fIsOnLane = 100.0f; // currently not evaluated
    m_lanes.fHasLaneToTheLeft =  fmax(0.0f, fmin(100.0f, m_lanes.fHasLaneToTheLeft + leftLaneConfidenceIncrease ) );
    m_lanes.fHasLaneToTheRight = fmax(0.0f, fmin(100.0f, m_lanes.fHasLaneToTheRight + rightLaneConfidenceIncrease ) );


    //if (m_showCalibInfo) {
        //LOG_INFO(cString::Format("left: %d  ||  right:%d", leftLaneConfidence, rightLaneConfidence));
    //}
}
