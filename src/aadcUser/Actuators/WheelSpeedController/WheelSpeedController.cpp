/**********************************************************************
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: �This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.�
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS �AS IS� AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: hart#$  $Date:: 2017-05-19 08:12:10#$ $Rev:: 63515   $
**********************************************************************/
#include <adtf3.h>
#include <stdlib.h>
#include <aadc_structs.h>
#include "WheelSpeedController.h"
#include "ADTF3_helper.h"


/// This defines a data triggered filter and exposes it via a plugin class factory.
/// The Triggerfunction cSimpleDataStatistics will be embedded to the Filter
/// and called repeatedly (last parameter of this macro)!
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_CARIOSITY_DATA_TRIGGERED_FILTER,
                                    "WheelSpeedController2",
                                    fcWheelSpeedController,
                                    adtf::filter::pin_trigger({"measured_vehicle_speed", "emergency_break"}));

fcWheelSpeedController::fcWheelSpeedController() : cStdFilter()
{
    //Register Properties
    RegisterPropertyVariable("PID_proportional factor for PID Controller ", m_f64PIDKp);
    RegisterPropertyVariable("PID_integral factor for PID Controller", m_f64PIDKi);
    RegisterPropertyVariable("PID_differential factor for PID Controller", m_f64PIDKd);
    RegisterPropertyVariable("PID_fixed sampling frequency [Hz]", m_f64FixedSamplingFrequency);

    RegisterPropertyVariable("output_gain", m_f64Gain);
    RegisterPropertyVariable("output_the minimum output value for the controller [%]", m_f64PIDMinimumOutput);
    RegisterPropertyVariable("output_the maximum output value for the controller [%]", m_f64PIDMaximumOutput);

    RegisterPropertyVariable("show debug output", m_bShowDebug);

    Register(m_oInputMeasWheelSpeed, "measured_vehicle_speed", getStreamType(SIGNAL_VALUE));
    Register(m_oInputSetWheelSpeed,  "desired_vehicle_speed", getStreamType(SIGNAL_VALUE));
    Register(m_oInputEmergencyBreak, "emergency_break", getStreamType(SIGNAL_VALUE));
    Register(m_oOutputActuator,      "actuator_output", getStreamType(SIGNAL_VALUE));
}
//implement the Configure function to read ALL Properties
tResult fcWheelSpeedController::Configure() {
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    resetController();

    RETURN_NOERROR;
}

///this funtion will be executed each time a trigger occured 
///Due to the change of the data receive events it can be possible that more than one sample was pushed to the 
/// Readers queue. So make sure the execution of this funtion will read ALL Samples of ALL Readers until the queues are empty.
tResult fcWheelSpeedController::Process(tTimeStamp tmTimeOfTrigger) {

    if (m_bShowDebug) {
        LOG_INFO("Process");
    }

    tSignalValue setSpeed;
    if(IS_OK(readSignalValue(m_oInputSetWheelSpeed, setSpeed))) {
        if (m_bShowDebug) {
            LOG_INFO("Read set Speed");
        }
        m_desiredSpeed = setSpeed.f32Value;
    }

    tSignalValue emergencyStop;
    if(IS_OK(readSignalValue(m_oInputEmergencyBreak, emergencyStop))) {
        if (m_bShowDebug) {
            LOG_INFO("Read emergency Brake");
        }
        m_emergencyStopEnabled = emergencyStop.f32Value;
    }

    tSignalValue measuredSpeedValue;
    if(IS_OK(readSignalValue(m_oInputMeasWheelSpeed, measuredSpeedValue))) {
        if (m_bShowDebug) {
            LOG_INFO("Read measured Speed");
        }
        m_measuredSpeed = measuredSpeedValue.f32Value;
    }

    if (m_bShowDebug) {
        LOG_INFO("Set Speed: %f\t Measured Speed: %f\t emergencyStop: %f", m_desiredSpeed, m_measuredSpeed, m_emergencyStopEnabled);
    }

    //calculation
    // if the desired output speed is 0 and we have stopped, immediately stop the motor
    // if the system just started, wait for the controller to start up
    // if a emergency stop is requested, reset Controller to stop immediately
    if (m_emergencyStopEnabled != 0.0f || (m_desiredSpeed == 0 && m_measuredSpeed == 0)) { //TODO 5*10e6 > GetTime() ||
        resetController();
    } else {
        m_f64LastOutput = getControllerValue(m_measuredSpeed);
    }


    tSignalValue outputValue = {tUInt32(tmTimeOfTrigger), tFloat32(m_f64LastOutput)};
    RETURN_IF_FAILED(transmitSignalValue(outputValue, m_oOutputActuator));
    RETURN_NOERROR;
}

void fcWheelSpeedController::resetController() {
    if(m_bShowDebug) {
        LOG_INFO("Resetting controller");
    }
    m_f64LastOutput = 0;
    m_f64LastMeasuredError = 0;
    m_desiredSpeed = 0;
    m_lastSampleTime = GetTime();
    m_f64accumulatedError = 0;
}

tFloat64 fcWheelSpeedController::getControllerValue(tFloat64 currentSpeed)
{

        // calculate time delta since last sample
    tTimeStamp currentTime = GetTime();
    tTimeStamp deltaT = (m_f64FixedSamplingFrequency != tUInt64(0)) ? tTimeStamp(1e6 / m_f64FixedSamplingFrequency) : currentTime - m_lastSampleTime;
    m_lastSampleTime = currentTime;

    if (m_bShowDebug) {
        LOG_INFO("deltaT: %d || Speed: %f || Desired Value: %f", deltaT, currentSpeed, m_desiredSpeed);
    }

    tFloat64 f64Result = 0;
    tFloat64 f64Error = (m_desiredSpeed - currentSpeed);

    //algorithm:
    //esum = esum + e
    //y = Kp * e + Ki * Ta * esum + Kd * (e - ealt) / deltaT
    //ealt = e

    // accumulated error (in m/s):
    m_f64accumulatedError += f64Error * deltaT / 1e6;

    tFloat64 y_p = m_f64PIDKp * f64Error;
    tFloat64 y_i = m_f64PIDKi * m_f64accumulatedError;
    tFloat64 y_d = m_f64PIDKd * (f64Error - m_f64LastMeasuredError) / (deltaT / 1e6);

    f64Result = y_p + y_i + y_d;

    m_f64LastMeasuredError = f64Error;

    if (m_bShowDebug) {
        LOG_INFO("Error: %f || Accu. Error: %f", f64Error, m_f64accumulatedError);
        LOG_INFO("Output Value before Limit: %f", f64Result);
    }
    // Apply gain
    f64Result *= m_f64Gain;

    // checking for minimum and maximum limits
    //f64Result = min(m_f64PIDMaximumOutput, max(m_f64PIDMinimumOutput, f64Result));
    if (f64Result > m_f64PIDMaximumOutput) f64Result = m_f64PIDMaximumOutput;
    if (f64Result < m_f64PIDMinimumOutput) f64Result = m_f64PIDMinimumOutput;

    if (m_bShowDebug) {
        LOG_INFO("Output after limit/gain: %f", f64Result);
        LOG_INFO("Using following specs: P: %f, I: %f, D: %f", y_p, y_i, y_d);
    }

    return f64Result;

}

tTimeStamp fcWheelSpeedController::GetTime()
{
    return adtf_util::cHighResTimer::GetTime();
}
