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

#include "ObjectDetectionFilter.h"
#include "stdafx.h"
#include "ADTF3_OpenCV_helper.h"
#include "../Helpers/cariosity_structs.h"


/// This defines a data triggered filter and exposes it via a plugin class factory.
/// The Triggerfunction cSimpleDataGenerator will be embedded to the Filter
/// and called when the timer times out!
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_CARIOSITY_TIME_TRIGGERED_FILTER,
                                    "Object Detection Filter YOLO",
                                    fcObjectDetectionFilter,
                                    adtf::filter::timer_trigger(166000));


fcObjectDetectionFilter::fcObjectDetectionFilter() : cStdFilter(), m_medianEmergencyCar(3) {
    RegisterPropertyVariable("Weights", m_weights);
    RegisterPropertyVariable("Config", m_config);
    RegisterPropertyVariable("Namefile", m_nameList);
    RegisterPropertyVariable("Detectionthreshold:", m_threshold);
    RegisterPropertyVariable("Car threshold:", m_thresholdCar);
    RegisterPropertyVariable("FOV:", m_fov);
    RegisterPropertyVariable("Show FPS", m_showFPS);
    RegisterPropertyVariable("YOLO Angle Epsilon", m_yoloAngleEpsilon);
    RegisterPropertyVariable("YOLO Max Object Distance", m_maxObjectDistance);
    RegisterPropertyVariable("Pattern emergency lights", m_pattern);
    RegisterPropertyVariable("Debug Emergencycar", m_debugEmergency);

    m_medianEmergencyCar.windowSize = 3;

    //register output
    Register(m_egoState, "EgoState", getStreamType(POINTER_VALUE));

    //create and set inital input format type
    m_InPinVideoFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
    adtf::ucom::object_ptr<IStreamType> pTypeInput = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pTypeInput, m_InPinVideoFormat);
    //Register input pin
    Register(m_oInputFrame, "in", pTypeInput);

    //Register output pins
    adtf::ucom::object_ptr<IStreamType> pTypeOutput = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pTypeOutput, m_OutPinVideoFormat);
    Register(m_oOutputFrame, "out", pTypeOutput);
    //register callback for type changes
    m_oInputFrame.SetAcceptTypeCallback([this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType> &pType) -> tResult {
        return ChangeType(m_oInputFrame, *pType.Get());
    });



}


tResult fcObjectDetectionFilter::Configure() {
    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    RETURN_NOERROR;
}

tResult fcObjectDetectionFilter::ChangeType(adtf::streaming::cDynamicSampleReader &inputPin,
                                            const adtf::streaming::ant::IStreamType &oType) {
    if (oType == adtf::streaming::stream_meta_type_image()) {
        adtf::ucom::object_ptr<const adtf::streaming::IStreamType> pTypeInput;
        // get pType from input reader
        inputPin >> pTypeInput;
        adtf::streaming::get_stream_type_image_format(m_InPinVideoFormat, *pTypeInput);

        //set also output format
        adtf::streaming::get_stream_type_image_format(m_OutPinVideoFormat, *pTypeInput);
        // and set pType also to samplewriter
        m_oOutputFrame << pTypeInput;
    } else {
        RETURN_ERROR(ERR_INVALID_TYPE);
    }
    RETURN_NOERROR;
}

tResult fcObjectDetectionFilter::receiveEgoStateInstance() {

    tPointerValue value;
    RETURN_IF_FAILED(cStdFilter::readPointerData(m_egoState, value));

    cEgoState::singleton = reinterpret_cast<cEgoState *>(value.address);
    if (!cEgoState::isInitialized()) {
        LOG_ERROR("Yolo: EgoState: ERR_NOT_INITIALIZED.");
        //TODO: return cError(ERR_NOT_INITIALIZED);
        RETURN_NOERROR;
    }


    RETURN_NOERROR;
}

tBool fcObjectDetectionFilter::checkTemplate(Mat &img, Mat &templ, float threshold, Scalar color, double *maxVal) {
    Mat matchResult, matchSquareResult;
    matchResult.create(img.cols - templ.cols + 1, img.rows - templ.rows + 1, CV_32FC1);

    matchTemplate(img, templ, matchResult, CV_TM_CCORR_NORMED);

    double minVal = 0.0;
    Point minLoc, maxLoc;
    minMaxLoc(matchResult, &minVal, maxVal, &minLoc, &maxLoc);
    //LOG_INFO("maxVal: %f", *maxVal);

    if (m_debugEmergency) {
        rectangle(img, Point(0,0), Point(templ.cols - 1, templ.rows - 1), color);
    }

    tBool result =  *maxVal > threshold;

    if (result && m_debugEmergency) {
        circle(img, Point(maxLoc.x + templ.cols / 2, maxLoc.y + templ.rows / 2), 5, color);
    }

    return result;
}

tFloat32 fcObjectDetectionFilter::checkEmergencyCar(BoxSE box, Mat &img) {
    Mat orig = img.clone();
    tFloat32 detectionResult = 0.0f;

    if (box.width < 150 && box.height < 150) {
        {
            int x = max(0, box.x + box.width / 4);
            int y = max(0, box.y - (box.height / 3));

            int width = box.width / 2;
            int height = box.height / 2;

            Rect cutOut = {x, y, width, height};

            cv::cuda::GpuMat gpuInput;
            gpuInput.upload(img);

            cv::cuda::GpuMat roi(gpuInput, cutOut);

            cv::cuda::cvtColor(roi, roi, CV_RGB2GRAY);
            cv::cuda::cvtColor(roi, roi, CV_GRAY2RGB);

            roi.download(img);

        }

        img -= mean(img);

        static Mat templ;
        // Initialize pattern
        if (templ.data == nullptr && cFileSystem::Exists(m_pattern)) {
            cFilename pattern = m_pattern;
            templ = imread(pattern.GetPtr());
            LOG_INFO("Pattern initialized");
        }

        Mat templScaled = templ.clone();
        Mat templSmaller; // the template in smaller size

        threshold(img, img, 120, 255, THRESH_BINARY);
        blur(img, img, Size(3, 3));

        //LOG_INFO("Template rows: %d", templ.rows);
        static const tFloat32 stdHeight = 18.0f;
        static const tFloat32 stdHeightSmaller = 31.0f;
        if (img.rows < stdHeightSmaller) { //scale image
            tFloat32 aspectRatio = tFloat(img.cols) / img.rows;
            resize(img, img, Size(int(stdHeightSmaller * aspectRatio), int(stdHeightSmaller)), 0, 0, cv::INTER_LINEAR);
        }

        if (img.rows > stdHeight) { //scale pattern
            resize(templScaled, templSmaller, Size(), img.rows / stdHeightSmaller, img.rows / stdHeightSmaller);
            resize(templScaled, templScaled, Size(), img.rows / stdHeight, img.rows / stdHeight);
            //LOG_INFO("Image rows: %d | scale Factor: %f | template Rows: %d", img.rows, img.rows / stdHeight, templ.rows);
        }

        double maxVal1, maxVal2;
        const tFloat32 threshold = 0.85f, thresholdSmaller = 0.9f;
        checkTemplate(img, templScaled, threshold, Scalar(255, 0, 0), &maxVal1);
        checkTemplate(img, templSmaller, thresholdSmaller, Scalar(0, 255, 0), &maxVal2);


        if (maxVal1 >= threshold || maxVal2 >= thresholdSmaller) {
            detectionResult = max(maxVal1, maxVal2);
        }

        if (m_debugEmergency) {
            static int i2 = 0;
            i2++;
            cString text = cString::Format("%.2f_%.2f", maxVal1, maxVal2);
            imwrite(std::string("/tmp/testbild_") + std::to_string(i2) + text + std::string(".ppm"), img);

            //copy to black mat with fixed size for adtf to be able to display
            Mat zeros = Mat::zeros(Size(300, 300), CV_8UC3);
            img.copyTo(zeros(Rect(0, 0, img.cols, img.rows)));
            img = zeros;

        } else {
            img = orig;
        }
    }

    return detectionResult;
}

tResult fcObjectDetectionFilter::handleDetections(vector<BoxSE> boxes, Mat &img, YOLOv3 &detector) {
    vector<tRecognizedObject> persons;
    vector<tRecognizedObject> childs;
    vector<tRecognizedObject> cars;
    vector<tBool> emergencyCarPresent;

    for (auto &box : boxes) {
        String objClass = detector.Names(box.m_class);
        bool valid = false;
        bool found = tFalse;
        tFloat32 category = -1;

        if (objClass == "person") {
            valid = box.height < 150 && box.width < 80 && (box.y + box.height) >= img.rows / 2;
            category = 1; //TODO: use 3 for child
        }
        if (objClass == "car") {
            valid = box.m_score >= m_thresholdCar;
            found = true; //disable laserscanner verification
            category = 2;
        }

        tInt32 middle = img.cols / 2;
        tFloat32 pxPerAngle = img.cols / m_fov;
        tFloat32 angle = ((box.x + box.width / 2) - middle) / pxPerAngle;

        if (angle < 0) { //make it positive again
            angle += 360;
        }

        vector<tPolarCoordiante> lLSData = cEgoState::singleton->getLaserScannerData();
        for (unsigned int i = 0; i < lLSData.size() && !found; i++) {
            tPolarCoordiante lsObject = lLSData.at(i);
            tFloat32 lowerBound = angle - m_yoloAngleEpsilon;
            tFloat32 upperBound = angle + m_yoloAngleEpsilon;
            found = lsObject.f32Angle >= lowerBound && lsObject.f32Angle <= upperBound && lsObject.f32Radius <= m_maxObjectDistance;
        }

        if (!found && valid && !m_debugEmergency) { //draw black rectangle for unverified objects
            rectangle(img, box, Scalar(0, 0, 0), 2);
        }

        if (valid && found) {
            tRecognizedObject obj;
            obj.ui8Category = category;
            obj.f32Angle = angle;
            obj.f32Confidence = box.m_score;

            switch (obj.ui8Category) {
                case 1: //Person
                    if (obj.f32Confidence < 80) { //child
                        childs.push_back(obj);
                    } else {
                        persons.push_back(obj);
                    }
                    break;
                case 2: //Car
                {
                    Mat searchFrame = img.clone();
                    tBool isEmergencyCar = checkEmergencyCar(box, searchFrame) > 0.0f;
                    emergencyCarPresent.push_back(isEmergencyCar);

                    if(m_debugEmergency) {
                        img = searchFrame;
                    }

                    cars.push_back(obj);
                }
                    break;
                default:
                    LOG_ERROR("Magic did happen");
            }


            if(!m_debugEmergency) {
                putText(img, objClass, box.tl(), FONT_HERSHEY_SIMPLEX, 1.0, m_colors[box.m_class], 2);
                rectangle(img, box, m_colors[box.m_class], 2);
            }
            LOG_INFO("Found: %s with %f confidence at Position: %i at angle: %f ", objClass.c_str(), box.m_score, box.x, angle);
        }
    }

    tBool foundEmergencyCar = tFalse;
    for(unsigned int i = 0; i < emergencyCarPresent.size() && !foundEmergencyCar; i++) {
        foundEmergencyCar = emergencyCarPresent.at(i);
    }

    m_medianEmergencyCar.pushValue(foundEmergencyCar ? 1.0f : 0.0f);

    tBool medianResult = m_medianEmergencyCar.calculateMedian() > 0.0f;

    cEgoState::singleton->setPersons(persons);
    cEgoState::singleton->setCars(cars);
    cEgoState::singleton->setChilds(childs);
    cEgoState::singleton->setEmergencyCarPresent(medianResult);

    if(medianResult) {
        LOG_INFO("Emergency car detected");
    }

    //workaround for displaying the cutout.
    Mat r;
    if (!foundEmergencyCar && m_debugEmergency) {
        img = r;
    }
    RETURN_NOERROR;
}

tResult fcObjectDetectionFilter::Process(tTimeStamp tmTimeOfTrigger) {
    receiveEgoStateInstance();
    if (!cEgoState::singleton->isInitialized()) {
        RETURN_NOERROR;
    }

    if (!cEgoState::singleton->getYOLOState()) {
        //do nothing
        RETURN_NOERROR;
    }

    static const cFilename weights = m_weights;
    static const cFilename config = m_config;
    static const cFilename nameList = m_nameList;

    static YOLOv3 detector;

    //first run
    if (createYOLO) {
        if (!cFileSystem::Exists(weights)) {
            cEgoState::singleton->setYOLOState(tFalse);
            LOG_ERROR("Weights file could not be loaded, but it's okay. I will simply stop working. YOLO "
                      "- okay, I know this is a bad place for bad jokes, but we've got to have these somewhere. "
                      "I think it's stated in the rulebook somewhere. I mean it's not like it's forbidden to have them"
                      ", so why not?");

            RETURN_ERROR_DESC(ERR_INVALID_FILE,
                              cString::Format("Weights file could not be loaded from %s, but it's okay. I will simply stop working. :-)", weights.GetPtr()));
        }
        LOG_INFO("Loading Object detection model - this might take a few seconds");
        detector.Create(weights.GetPtr(), config.GetPtr(), nameList.GetPtr());
        createYOLO = false;
        LOG_INFO("Object model loaded");
    }


    //actual processing
    object_ptr<const ISample> pReadSample;

    if (IS_OK(m_oInputFrame.GetLastSample(pReadSample))) {
        object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;
        Mat outputImage;
        //lock read buffer
        if (IS_OK(pReadSample->Lock(pReadBuffer))) {

            Size inputSize = Size(m_InPinVideoFormat.m_ui32Width, m_InPinVideoFormat.m_ui32Height);
            //create a opencv matrix from the media sample buffer
            Mat inputImage(inputSize, CV_8UC3, (uchar *) pReadBuffer->GetPtr());
            outputImage = inputImage.clone();
            //draw line in the middle
            //cv::line(outputImage, {inputImage.cols / 2, 0}, {inputImage.cols / 2, inputImage.cols}, Scalar(0, 255, 255));

            outputImage = outputImage(Range(450, 600), Range(0, outputImage.cols));

            //detect and measure time it took
            std::chrono::system_clock::time_point t_beg, t_end;
            std::chrono::duration<double> diff;
            t_beg = std::chrono::system_clock::now();
            auto boxes = detector.Detect(outputImage, m_threshold);
            t_end = std::chrono::system_clock::now();
            diff = t_end - t_beg;
            if (m_showFPS) {
                LOG_INFO("FPS: %f", 1.0 / diff.count());
            }

            handleDetections(boxes, outputImage, detector);
            pReadBuffer->Unlock();
        }

        if (!outputImage.empty()) {
            //update output format if matrix size does not fit to
            if (outputImage.total() * outputImage.elemSize() != m_OutPinVideoFormat.m_szMaxByteSize) {
                setTypeFromMat(m_oOutputFrame, outputImage);
            }
            // write to pin
            writeMatToPin(m_oOutputFrame, outputImage, m_pClock->GetStreamTime());
        }

    }
    RETURN_NOERROR;
}


