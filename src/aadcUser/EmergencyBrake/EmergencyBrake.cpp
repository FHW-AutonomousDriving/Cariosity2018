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
#include "EmergencyBrake.h"

#define CONSOLE_LOG_INFO(...) if (m_propShowDebug) LOG_ADD_ENTRY(A_UTILS_NS::log::tLogLevel::Info, __VA_ARGS__)

ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_CARIOSITY_DATA_TRIGGERED_FILTER,
    "EmergencyBrake",
    fcEmergencyBrake,
    adtf::filter::pin_trigger({"laserScanner"}));


fcEmergencyBrake::fcEmergencyBrake() : cStdFilter()
{
    Register(m_oReaderLaser, "laserScanner", getStreamType(LASER_SCANNER));
    Register(m_oWriterValue, "output", getStreamType(SIGNAL_VALUE));
    Register(m_oReaderSteeringAngle, "in_steeringAngle", getStreamType(SIGNAL_VALUE));

    // Register Properties
    RegisterPropertyVariable("Field of view min angle", m_propMinFoVAngle);
    RegisterPropertyVariable("Field of view max angle", m_propMaxFoVAngle);
    RegisterPropertyVariable("Minimum distance to object", m_propMinObstacleDistance);
    RegisterPropertyVariable("Minimum distance on sides", m_propMinSideDistance);

    RegisterPropertyVariable("Width of the vehicle", m_propVehicleWidth);
    RegisterPropertyVariable("Show debug output", m_propShowDebug);

    m_currentSteeringAngleValue.f32Value = 0.0f;

}


//implement the Configure function to read ALL Properties
tResult fcEmergencyBrake::Configure()
{
    RETURN_NOERROR;
}

tResult fcEmergencyBrake::Process(tTimeStamp tmTimeOfTrigger)
{
    static tFloat32 lastValue;
    vector<tPolarCoordiante> scan;
    RETURN_IF_FAILED(readLaserScannerData(m_oReaderLaser, scan));
    RETURN_IF_FAILED(readSignalValue(m_oReaderSteeringAngle, m_currentSteeringAngleValue));


    tFloat32 currentValue = CheckEmergencyBrake(scan);
    if (currentValue != lastValue) {
        tSignalValue signalValue;
        signalValue.ui32ArduinoTimestamp = 0;
        signalValue.f32Value = currentValue;

        RETURN_IF_FAILED(transmitSignalValue(signalValue, m_oWriterValue))
        lastValue = currentValue;
    }


    RETURN_NOERROR;
}


tFloat32 fcEmergencyBrake::CheckEmergencyBrake(vector<tPolarCoordiante> vector)
{
    for (tSize i = 0; i < vector.size(); ++i) {
        auto sample = vector.at(i);

        auto angle = sample.f32Angle * M_PI / 180;

        auto distance = cos(angle)*sample.f32Radius;
        auto steeringCompensation = tan(m_currentSteeringAngleValue.f32Value * M_PI / (180 * 3)) * distance; // / 3 to approximate steering angle
        auto distFromCenter = sin(angle)*sample.f32Radius - steeringCompensation;


        //printf("Input Angle: %f\tCalculated Angle: %f\tRadius: %f\tDist from center: %f\tDistance: %f\n", sample.f32Angle, angle, sample.f32Radius, distFromCenter, distance);

        if (abs(distFromCenter) < ((CAR_WIDTH*10)/2 + m_propMinSideDistance) && distance < m_propMinObstacleDistance) {
            CONSOLE_LOG_INFO("Compensation: %f\t Steering: %f", steeringCompensation, m_currentSteeringAngleValue.f32Value);
            CONSOLE_LOG_INFO("distFromCenter: %f\t Distance: %f", distFromCenter, distance);
            //LOG_ERROR("Collision at distance: %f with angle: %f", sample.f32Radius, angle);
            return 1.0f;
        }
    }

    return 0.0;
}
