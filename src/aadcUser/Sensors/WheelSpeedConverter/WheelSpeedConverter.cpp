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
#include "WheelSpeedConverter.h"
#include "aadc_structs.h"
#include "ADTF3_helper.h"

#define CW_SLOT_COUNT 60.f

ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_CARIOSITY_DATA_TRIGGERED_FILTER,
                                    "WheelSpeedConverter",
                                    fcWheelSpeedConverter,
                                    adtf::filter::pin_trigger({"wheel_left"}));

fcWheelSpeedConverter::fcWheelSpeedConverter() {
    //Register Properties
    RegisterPropertyVariable("wheel circumference [m]", m_f32wheelCircumference);
    RegisterPropertyVariable("filter constant of first order", m_f32FilterConstantfirstOrder);
    RegisterPropertyVariable("enable filtering", m_bEnableFiltering);

    //the wheel data
    object_ptr<IStreamType> pTypeWheelData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tWheelData", pTypeWheelData, m_WheelDataSampleFactory)) {
        adtf_ddl::access_element::find_index(m_WheelDataSampleFactory, "ui32ArduinoTimestamp", m_ddlWheelDataIndex.ArduinoTimestamp);
        adtf_ddl::access_element::find_index(m_WheelDataSampleFactory, "ui32WheelTach", m_ddlWheelDataIndex.WheelTach);
        adtf_ddl::access_element::find_index(m_WheelDataSampleFactory, "i8WheelDir", m_ddlWheelDataIndex.WheelDir);
    } else {
        LOG_INFO("No mediadescription for tWheelData found!");
    }
    Register(m_oInputWheelLeft, "wheel_left", pTypeWheelData);
    Register(m_oInputWheelRight, "wheel_right", pTypeWheelData);
    Register(m_oOutputWheelStructLeft, "wheel_left_plaus", pTypeWheelData);
    Register(m_oOutputWheelStructRight, "wheel_right_plaus", pTypeWheelData);

    object_ptr<IStreamType> pTypeSignalValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeSignalValue, m_SignalValueSampleFactory)) {
        (adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("ui32ArduinoTimestamp"), m_ddlSignalValueId.timeStamp));
        (adtf_ddl::access_element::find_index(m_SignalValueSampleFactory, cString("f32Value"), m_ddlSignalValueId.value));
    } else {
        LOG_INFO("No mediadescription for tSignalValue found!");
    }

    Register(m_oOutputCarSpeed, "vehicle_speed", pTypeSignalValue);
    Register(m_oOutputDistanceOverall, "distance_overall", pTypeSignalValue);
    Register(m_oOutputDistanceLastSample, "distance_last_sample", pTypeSignalValue);

    m_tBeforeLastStructLeft = {};
    m_tLastStructLeft = {};
    m_tBeforeLastStructRight = {};
    m_tLastStructRight = {};

}


//implement the Configure function to read ALL Properties
tResult fcWheelSpeedConverter::Configure() {
    RETURN_IF_FAILED(cTriggerFunction::Configure());
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));

    RETURN_NOERROR;
}

tResult fcWheelSpeedConverter::Process(tTimeStamp tmTimeOfTrigger) {

    //Wheel Left 
    object_ptr<const ISample> pSampleFromWheelLeft;

    if (IS_OK(m_oInputWheelLeft.GetLastSample(pSampleFromWheelLeft))) {
        if (m_bfirstSampleReceivedLeftWheel) {
            m_tBeforeLastStructLeft = m_tLastStructLeft;
        }

        auto oDecoder = m_WheelDataSampleFactory.MakeDecoderFor(*pSampleFromWheelLeft);
        RETURN_IF_FAILED(oDecoder.IsValid());

        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlWheelDataIndex.ArduinoTimestamp, &m_tLastStructLeft.ui32ArduinoTimestamp));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlWheelDataIndex.WheelTach, &m_tLastStructLeft.ui32WheelTach));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlWheelDataIndex.WheelDir, &m_tLastStructLeft.i8WheelDir));

    }



    //Wheel Right
    object_ptr<const ISample> pSampleFromWheelRight;

    if (IS_OK(m_oInputWheelRight.GetLastSample(pSampleFromWheelRight))) {
        if (m_bfirstSampleReceivedRightWheel) {
            m_tBeforeLastStructRight = m_tLastStructRight;
        }

        auto oDecoder = m_WheelDataSampleFactory.MakeDecoderFor(*pSampleFromWheelRight);
        RETURN_IF_FAILED(oDecoder.IsValid());

        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlWheelDataIndex.ArduinoTimestamp, &m_tLastStructRight.ui32ArduinoTimestamp));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlWheelDataIndex.WheelTach, &m_tLastStructRight.ui32WheelTach));
        RETURN_IF_FAILED(oDecoder.GetElementValue(m_ddlWheelDataIndex.WheelDir, &m_tLastStructRight.i8WheelDir));

    }



    ////////////////// Do Calculations ///////////////////


    // if the values are different, calculate new speed value
    if (m_bfirstSampleReceivedLeftWheel && m_tBeforeLastStructLeft.ui32ArduinoTimestamp != m_tLastStructLeft.ui32ArduinoTimestamp) {

        // doing the calculation and the transmit

        tFloat32 currentSpeed = calculateSpeed(
                m_tLastStructLeft.ui32ArduinoTimestamp,
                m_tBeforeLastStructLeft.ui32ArduinoTimestamp,
                m_tLastStructLeft.ui32WheelTach - m_tBeforeLastStructLeft.ui32WheelTach
        );

        // doing an minimal smoothing of the signal
        if (m_bEnableFiltering) {
            m_f32LastCalculatedSpeedLeft = m_f32LastCalculatedSpeedLeft + m_f32FilterConstantfirstOrder * (currentSpeed - m_f32LastCalculatedSpeedLeft);
        } else {
            m_f32LastCalculatedSpeedLeft = currentSpeed;
        }


    }


    // if the values are different, calculate new speed value
    if (m_bfirstSampleReceivedRightWheel && m_tBeforeLastStructRight.ui32ArduinoTimestamp != m_tLastStructRight.ui32ArduinoTimestamp) {

        // doing the calculation and the transmit

        tFloat32 currentSpeed = calculateSpeed(
                m_tLastStructRight.ui32ArduinoTimestamp,
                m_tBeforeLastStructRight.ui32ArduinoTimestamp,
                m_tLastStructRight.ui32WheelTach - m_tBeforeLastStructRight.ui32WheelTach
        );

        // doing an minimal smoothing of the signal
        if (m_bEnableFiltering) {
            m_f32LastCalculatedSpeedRight = m_f32LastCalculatedSpeedRight + m_f32FilterConstantfirstOrder * (currentSpeed - m_f32LastCalculatedSpeedRight);
        } else {
            m_f32LastCalculatedSpeedRight = currentSpeed;
        }
    }

    if (m_bfirstSampleReceivedLeftWheel && m_bfirstSampleReceivedRightWheel && (m_tBeforeLastStructRight.ui32ArduinoTimestamp != m_tLastStructRight.ui32ArduinoTimestamp || m_tBeforeLastStructLeft.ui32ArduinoTimestamp != m_tLastStructLeft.ui32ArduinoTimestamp)) {
        // transmit only if anything changed
        TransmitSamples();
    }

    if (!m_bfirstSampleReceivedLeftWheel) {
        m_bfirstSampleReceivedLeftWheel = tTrue;
    }
    if (!m_bfirstSampleReceivedRightWheel) {
        m_bfirstSampleReceivedRightWheel = tTrue;
    }

RETURN_NOERROR;
}

tFloat32 fcWheelSpeedConverter::calculateSpeed(const tUInt32 &ui32CurrentTimeStamp, const tUInt32 &ui32LastTimeStamp, const tUInt32 &ui32Ticks) {
    // return if time difference is 0, if time difference is smaller than 0, if ticks are 0 or smaller 0
    if ((ui32CurrentTimeStamp - ui32LastTimeStamp == 0) || (ui32Ticks == 0)) return 0;
    //          circumference      SlotsInTimeDiff
    // speed =  -------------- *  -------------
    //           TotalSlots*          TimeDiff
    return (m_f32wheelCircumference / CW_SLOT_COUNT * static_cast<tFloat32>(ui32Ticks)) /
           (static_cast<tFloat32>(ui32CurrentTimeStamp - ui32LastTimeStamp) / static_cast<tFloat32>(1e6));
}

tResult fcWheelSpeedConverter::TransmitSamples() {

    if (m_tLastStructLeft.i8WheelDir == m_tLastStructRight.i8WheelDir) {
        m_i8olddirection = m_tLastStructLeft.i8WheelDir;
    }



    // calculate the average of both wheel speeds
    tFloat32 f32speed = (m_f32LastCalculatedSpeedRight + m_f32LastCalculatedSpeedLeft) / 2.0f;
    // as speed is an absolute value take the direction into account
    if (m_i8olddirection == 1) f32speed *= -1;


    // distance since last sample
    tFloat32 f32distance =
            (
                    (m_tLastStructLeft.ui32WheelTach - m_tBeforeLastStructLeft.ui32WheelTach)
                    + (m_tLastStructRight.ui32WheelTach - m_tBeforeLastStructRight.ui32WheelTach)
            ) / 2.0f
            * m_f32wheelCircumference / CW_SLOT_COUNT;


    m_f32OverallDistance += f32distance;
    //LOG_INFO("add distance: %.2f - %.2f", f32distance, m_f32OverallDistance);


    //calculate the average of the arduino timestamp
    tUInt32 ui32arduinoTimestamp = (m_tLastStructLeft.ui32ArduinoTimestamp + m_tLastStructRight.ui32ArduinoTimestamp) / 2;

    //Transmit Values 

    RETURN_IF_FAILED(transmitSignalValue(m_oOutputCarSpeed, m_pClock->GetStreamTime(), m_SignalValueSampleFactory, m_ddlSignalValueId.timeStamp, ui32arduinoTimestamp, m_ddlSignalValueId.value, f32speed));

    RETURN_IF_FAILED(transmitSignalValue(m_oOutputDistanceOverall, m_pClock->GetStreamTime(), m_SignalValueSampleFactory, m_ddlSignalValueId.timeStamp, ui32arduinoTimestamp, m_ddlSignalValueId.value, m_f32OverallDistance));

    RETURN_IF_FAILED(transmitSignalValue(m_oOutputDistanceLastSample, m_pClock->GetStreamTime(), m_SignalValueSampleFactory, m_ddlSignalValueId.timeStamp, ui32arduinoTimestamp, m_ddlSignalValueId.value, f32distance));

    object_ptr<ISample> pWriteWheelLeftSample;

    if (IS_OK(alloc_sample(pWriteWheelLeftSample))) {
        auto oCodec = m_WheelDataSampleFactory.MakeCodecFor(pWriteWheelLeftSample);

        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlWheelDataIndex.ArduinoTimestamp, m_tLastStructLeft.ui32ArduinoTimestamp));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlWheelDataIndex.WheelTach, m_tLastStructLeft.ui32WheelTach));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlWheelDataIndex.WheelDir, m_tLastStructLeft.i8WheelDir));


    }
    m_oOutputWheelStructLeft << pWriteWheelLeftSample << flush << trigger;

    object_ptr<ISample> pWriteWheelRightSample;

    if (IS_OK(alloc_sample(pWriteWheelRightSample))) {
        auto oCodec = m_WheelDataSampleFactory.MakeCodecFor(pWriteWheelRightSample);

        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlWheelDataIndex.ArduinoTimestamp, m_tLastStructRight.ui32ArduinoTimestamp));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlWheelDataIndex.WheelTach, m_tLastStructRight.ui32WheelTach));
        RETURN_IF_FAILED(oCodec.SetElementValue(m_ddlWheelDataIndex.WheelDir, m_tLastStructRight.i8WheelDir));


    }
    m_oOutputWheelStructRight << pWriteWheelRightSample << flush << trigger;

    RETURN_NOERROR;
}
