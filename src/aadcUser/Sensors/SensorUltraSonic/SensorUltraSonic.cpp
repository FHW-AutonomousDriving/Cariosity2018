#include "stdafx.h"
#include "SensorUltraSonic.h"


ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_CARIOSITY_DATA_TRIGGERED_FILTER,
                                    "SensorUltraSonic",
                                    fcSensorUltraSonic,
                                    adtf::filter::pin_trigger({"input"}));


fcSensorUltraSonic::fcSensorUltraSonic() : cStdFilter(),
m_mSideRight(3),
m_mRearRight(3),
m_mRearCenter(3),
m_mRearLeft(3),
m_mSideLeft(3){
    // Register Pins
    Register(m_oReaderUltraSonic, "input", getStreamType(ULTRASONIC));
    Register(m_oWriterUltraSonic, "output", getStreamType(ULTRASONIC));
    Register(m_oWriterSideRight, "SideRight", getStreamType(SIGNAL_VALUE));
    Register(m_oWriterRearRight, "RearRight", getStreamType(SIGNAL_VALUE));
    Register(m_oWriterRearCenter, "RearCenter", getStreamType(SIGNAL_VALUE));
    Register(m_oWriterRearLeft, "RearLeft", getStreamType(SIGNAL_VALUE));
    Register(m_oWriterSideLeft, "SideLeft", getStreamType(SIGNAL_VALUE));

    // Register Properties
    RegisterPropertyVariable("Window size of the median", m_propMedianWindowSize);
    RegisterPropertyVariable("Maximum distance to be considered", m_propMaxValidValue);

    m_mSideRight.windowSize = m_propMedianWindowSize;
    m_mRearRight.windowSize = m_propMedianWindowSize;
    m_mRearCenter.windowSize = m_propMedianWindowSize;
    m_mRearLeft.windowSize = m_propMedianWindowSize;
    m_mSideLeft.windowSize = m_propMedianWindowSize;

}


//implement the Configure function to read ALL Properties
tResult fcSensorUltraSonic::Configure() {
    RETURN_NOERROR;
}

tResult fcSensorUltraSonic::Process(tTimeStamp tmTimeOfTrigger) {

    tUltrasonicStruct USSData;

    RETURN_IF_FAILED(readUltrasonicData(m_oReaderUltraSonic, USSData));

    tUltrasonicStruct sanitized = sanitize(USSData);

    RETURN_IF_FAILED(transmitUltrasonicStruct(sanitized, m_oWriterUltraSonic));


    RETURN_NOERROR;
}

tUltrasonicStruct fcSensorUltraSonic::sanitize(tUltrasonicStruct USSData) {
    tUltrasonicStruct result;
    if (isValid(USSData.tSideRight.f32Value)) {
        m_mSideRight.pushValue(USSData.tSideRight.f32Value);
        tSignalValue SideRight = {USSData.tSideRight.ui32ArduinoTimestamp, m_mSideRight.calculateMedian()};
        transmitSignalValue(SideRight, m_oWriterSideRight);
        result.tSideRight.f32Value = m_mSideRight.calculateMedian();
    }

    if (isValid(USSData.tRearRight.f32Value)) {
        m_mRearRight.pushValue(USSData.tRearRight.f32Value);
        tSignalValue RearRight = {USSData.tRearRight.ui32ArduinoTimestamp, m_mRearRight.calculateMedian()};
        transmitSignalValue(RearRight, m_oWriterRearRight);
        result.tRearRight.f32Value = m_mRearRight.calculateMedian();
    }

    if (isValid(USSData.tRearCenter.f32Value)) {
        m_mRearCenter.pushValue(USSData.tRearCenter.f32Value);
        tSignalValue RearCenter = {USSData.tRearCenter.ui32ArduinoTimestamp, m_mRearCenter.calculateMedian()};
        transmitSignalValue(RearCenter, m_oWriterRearCenter);
        result.tRearCenter.f32Value = m_mRearCenter.calculateMedian();
    }

    if (isValid(USSData.tRearLeft.f32Value)) {
        m_mRearLeft.pushValue(USSData.tRearLeft.f32Value);
        tSignalValue RearLeft = {USSData.tRearLeft.ui32ArduinoTimestamp, m_mRearLeft.calculateMedian()};
        transmitSignalValue(RearLeft, m_oWriterRearLeft);
        result.tRearLeft.f32Value = m_mRearLeft.calculateMedian();
    }

    if (isValid(USSData.tSideLeft.f32Value)) {
        m_mSideLeft.pushValue(USSData.tSideLeft.f32Value);
        tSignalValue SideLeft = {USSData.tSideLeft.ui32ArduinoTimestamp, m_mSideLeft.calculateMedian()};
        transmitSignalValue(SideLeft, m_oWriterSideLeft);
        result.tSideLeft.f32Value = m_mSideLeft.calculateMedian();
    }


    return result;
}

tBool fcSensorUltraSonic::isValid(tFloat32 value) {
    return value > 0 && value < m_propMaxValidValue;
}