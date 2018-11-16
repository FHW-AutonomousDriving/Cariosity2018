#include <adtf3.h>
#include <stdlib.h>
#include <aadc_structs.h>
#include "AngleToServo.h"
#include "ADTF3_helper.h"

/// This defines a data triggered filter and exposes it via a plugin class factory.
/// The Triggerfunction cSimpleDataStatistics will be embedded to the Filter
/// and called repeatedly (last parameter of this macro)!
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_CARIOSITY_DATA_TRIGGERED_FILTER,
                                    "AngleToServo",
                                    fcAngleToServo,
                                    adtf::filter::pin_trigger({"angle"}));

fcAngleToServo::fcAngleToServo() : cStdFilter() {

    RegisterPropertyVariable("maximum deflection of front left tire", m_f32MaxLeftAngle);
    RegisterPropertyVariable("maximum deflection of front right tire", m_f32MaxRightAngle);

    Register(m_oInputAngle, "angle", getStreamType(SIGNAL_VALUE));
    Register(m_oOutputServo, "servo", getStreamType(SIGNAL_VALUE));

}


tResult fcAngleToServo::Configure()
{
    RETURN_NOERROR;
}


tResult fcAngleToServo::Process(tTimeStamp tmTimeOfTrigger)
{

    tSignalValue inputAngle;
    RETURN_IF_FAILED(readSignalValue(m_oInputAngle, inputAngle));

    tSignalValue outputValue = {inputAngle.ui32ArduinoTimestamp, convertAngleToServoValue(inputAngle.f32Value)};

    RETURN_IF_FAILED(transmitSignalValue(outputValue, m_oOutputServo));

    RETURN_NOERROR;
}


tFloat32 fcAngleToServo::convertAngleToServoValue(tFloat32 angle) {

    tFloat32 percentage = 0;

    if (fabs(angle) <= 0.1f) {
        return 0;
    }

    if (angle < 0) {
        // go left

        angle = fabs(angle);
        if (angle > m_f32MaxRightAngle) {
            percentage = -1;
        } else {
            percentage = -(angle / m_f32MaxRightAngle);
        }

    } else {
        // go right

        if (angle > m_f32MaxLeftAngle) {
            percentage = 1;
        } else {
            percentage = angle / m_f32MaxLeftAngle;
        }

    }

    return percentage * 100.0f;
}