#include "stdafx.h"
#include "BirdsEyeTransform.h"
#include "ADTF3_OpenCV_helper.h"

#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/cudaimgproc.hpp>

/// This defines a data triggered filter and exposes it via a plugin class factory.
/// The Triggerfunction cSimpleDataStatistics will be embedded to the Filter
/// and called repeatedly (last parameter of this macro)!
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_CARIOSITY_DATA_TRIGGERED_FILTER,
                                    "Birds Eye Transform",
                                    fcBirdsEyeTransform,
                                    adtf::filter::pin_trigger({ "in" }));


fcBirdsEyeTransform::fcBirdsEyeTransform() {

    //Register Properties
    RegisterPropertyVariable("ROIOffsetX [%]",      m_ROIOffsetX);
    RegisterPropertyVariable("ROIOffsetY [%]",      m_ROIOffsetY);
    RegisterPropertyVariable("ROIWidth   [%]",        m_ROIWidth);
    RegisterPropertyVariable("ROIHeight  [%]",       m_ROIHeight);
    RegisterPropertyVariable("Output Height  [Pixel]", m_OutputHeight);

    RegisterPropertyVariable("side inset at top    [%]", m_topSideInset);
    RegisterPropertyVariable("side inset at bottom [%]", m_bottomSideInset);
    RegisterPropertyVariable("Show ROI, no transform", m_showROIDebug);

    //create and set inital input format type
    m_InPinVideoFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
    adtf::ucom::object_ptr<IStreamType> pTypeInput = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pTypeInput, m_InPinVideoFormat);
    //Register input pin
    Register(m_oReaderVideo, "in", pTypeInput);

    //Register output pins   
    adtf::ucom::object_ptr<IStreamType> pTypeOutput = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pTypeOutput, m_OutPinVideoFormat);
    Register(m_oWriterVideo, "out", pTypeOutput);

    //register callback for type changes
    m_oReaderVideo.SetAcceptTypeCallback([this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pType) -> tResult
    {
        return ChangeType(m_oReaderVideo, *pType.Get());
    });

}

tResult fcBirdsEyeTransform::Configure()
{
    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));

    RETURN_NOERROR;
}

tResult fcBirdsEyeTransform::ChangeType(adtf::streaming::cDynamicSampleReader& inputPin, const adtf::streaming::ant::IStreamType& oType) {
    if (oType == adtf::streaming::stream_meta_type_image())
    {
        adtf::ucom::object_ptr<const adtf::streaming::IStreamType> pTypeInput;
        // get pType from input reader
        inputPin >> pTypeInput;
        adtf::streaming::get_stream_type_image_format(m_InPinVideoFormat, *pTypeInput);

        //set also output format 
        adtf::streaming::get_stream_type_image_format(m_OutPinVideoFormat, *pTypeInput);
        // and set pType also to samplewriter
        m_oWriterVideo << pTypeInput;
    }
    else
    {
        RETURN_ERROR(ERR_INVALID_TYPE);
    }

    RETURN_NOERROR;
}

tResult fcBirdsEyeTransform::Process(tTimeStamp tmTimeOfTrigger) {

    object_ptr<const ISample> pReadSample;
    if (IS_OK(m_oReaderVideo.GetNextSample(pReadSample))) {

        object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;
        Mat outputImage;
        cv::cuda::GpuMat dst, src;
        //lock read buffer
        if (IS_OK(pReadSample->Lock(pReadBuffer))) {


            RETURN_IF_FAILED(checkRoi());
            Size inputSize = cv::Size(m_InPinVideoFormat.m_ui32Width, m_InPinVideoFormat.m_ui32Height);

            //create a opencv matrix from the media sample buffer
            Mat inputImage(inputSize, CV_8UC3, (uchar*)pReadBuffer->GetPtr());
            inputImage = inputImage.clone();


            Rect roi(
                    inputSize.width  * m_ROIOffsetX,
                    inputSize.height * m_ROIOffsetY,
                    inputSize.width  * m_ROIWidth,
                    inputSize.height * m_ROIHeight
            );
            tInt64 aspectRatio = roi.width / roi.height;
            Size outputSize = cv::Size(m_OutputHeight * aspectRatio , m_OutputHeight);

            Point2f src_vertices[4];
            src_vertices[0] = Point2f(roi.x, roi.y);
            src_vertices[1] = Point2f(roi.x + roi.width, roi.y);
            src_vertices[2] = Point2f(roi.x + roi.width, roi.y + roi.height);
            src_vertices[3] = Point2f(roi.x, roi.y + roi.height);

            Point2f dst_vertices[4];
            dst_vertices[0] = Point2f(roi.x + roi.width * m_topSideInset, roi.y); // Point2f(inputSize.width * m_topSideInset, 0);
            dst_vertices[1] = Point2f(roi.x + roi.width * (1.0f - m_topSideInset), roi.y); // Point2f(inputSize.width * (1.0f - m_topSideInset), 0);
            dst_vertices[2] = Point2f(roi.x + (1.0f - m_bottomSideInset) * roi.width, roi.y + roi.height); // Point2f(inputSize.width * (1.0f - m_bottomSideInset), inputSize.height);
            dst_vertices[3] = Point2f(roi.x + m_bottomSideInset * roi.width, roi.y + roi.height); // Point2f(inputSize.width * m_bottomSideInset, inputSize.height);
            Mat M = getPerspectiveTransform(src_vertices, dst_vertices);

            if(m_showROIDebug) {
                outputImage = inputImage.clone();
                line(outputImage, src_vertices[0], src_vertices[1], Scalar(0,255,255), 2);
                line(outputImage, src_vertices[1], src_vertices[2], Scalar(0,255,255), 2);
                line(outputImage, src_vertices[2], src_vertices[3], Scalar(0,255,255), 2);
                line(outputImage, src_vertices[3], src_vertices[0], Scalar(0,255,255), 2);

                line(outputImage, dst_vertices[0], dst_vertices[1], Scalar(255,255,255), 2);
                line(outputImage, dst_vertices[1], dst_vertices[2], Scalar(255,255,255), 2);
                line(outputImage, dst_vertices[2], dst_vertices[3], Scalar(255,255,255), 2);
                line(outputImage, dst_vertices[3], dst_vertices[0], Scalar(255,255,255), 2);

                resize(outputImage, outputImage, outputSize);
            } else {
                src.upload(inputImage);

                cv::cuda::warpPerspective(src, dst, M, inputSize, INTER_LINEAR, BORDER_CONSTANT);
                // cv::cuda::resize(dst, dst, outputSize);
                dst.download(outputImage);

                outputImage = outputImage(roi).clone();
            }

            pReadBuffer->Unlock();
        }
        //copy to outputimage

        //Write processed Image to Output Pin
        if (!outputImage.empty())
        {
            //update output format if matrix size does not fit to
            if (outputImage.total() * outputImage.elemSize() != m_OutPinVideoFormat.m_szMaxByteSize)
            {
                setTypeFromMat(m_oWriterVideo, outputImage);
            }
            // write to pin
            writeMatToPin(m_oWriterVideo, outputImage, m_pClock->GetStreamTime());
        }

    }

    RETURN_NOERROR;
}

tResult fcBirdsEyeTransform::checkRoi(void)
{
    // if width or heigt are not set ignore the roi
    if (static_cast<tFloat32>(m_ROIWidth) == 0 || static_cast<tFloat32>(m_ROIHeight) == 0)
    {
        LOG_ERROR("ROI width or height is not set!");
        RETURN_ERROR_DESC(ERR_INVALID_ARG, "ROI width or height is not set!");
    }

    //check if we are within the boundaries of the image
    if ((static_cast<tFloat32>(m_ROIOffsetX) + static_cast<tFloat32>(m_ROIWidth)) > m_InPinVideoFormat.m_ui32Width)
    {
        LOG_ERROR("ROI is outside of image");
        RETURN_ERROR_DESC(ERR_INVALID_ARG, "ROI is outside of image");
    }

    if ((static_cast<tFloat32>(m_ROIOffsetY) + static_cast<tFloat32>(m_ROIHeight)) > m_InPinVideoFormat.m_ui32Height)
    {
        LOG_ERROR("ROI is outside of image");
        RETURN_ERROR_DESC(ERR_INVALID_ARG, "ROI is outside of image");
    }

    //create the rectangle
    m_LaneRoi = cv::Rect2f(static_cast<tFloat32>(m_ROIOffsetX), static_cast<tFloat32>(m_ROIOffsetY), static_cast<tFloat32>(m_ROIWidth), static_cast<tFloat32>(m_ROIHeight));


    RETURN_NOERROR;
}