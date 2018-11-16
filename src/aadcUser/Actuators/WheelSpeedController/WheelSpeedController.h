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

#define CID_CARIOSITY_DATA_TRIGGERED_FILTER "wheel_speed_controller.filter.user.aadc.cid"

using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;

#include "../../Helpers/StdFilter.h"

/*! This is the main class for the controller for handling wheel speed. */
class fcWheelSpeedController : public cStdFilter
{
private:

    //Pins
    /*! Input Pin for wheel struct*/
    cPinReader      m_oInputMeasWheelSpeed;
    /*! Input Pin for wheel struct*/
    cPinReader      m_oInputSetWheelSpeed;
    /*! Input Pin for wheel struct*/
    cPinReader      m_oInputEmergencyBreak;


    /*! output pin for the the speed of the wheels */
    cPinWriter      m_oOutputActuator;

    /*! holds the last measuredValue */
    tFloat64 m_measuredSpeed;
    /*! holds the last measured error */
    tFloat64 m_f64LastMeasuredError;
    /*! holds the last setpoint */
    tFloat64 m_desiredSpeed;
    /*! holds the last output */
    tFloat64 m_f64LastOutput;
    /*! holds the last sample time */
    tTimeStamp m_lastSampleTime;
    /*! holds the accumulatedError for the controller */
    tFloat64 m_f64accumulatedError;

    tFloat32 m_emergencyStopEnabled;

    /*! The clock */
    object_ptr<adtf::services::IReferenceClock> m_pClock;

    /*!
    * Gets the time.
    *
    * \return  The time streamtime in milliseconds
    */
    tTimeStamp GetTime();

    /*! calculates the manipulated value for the given values, it uses the setpoint in m_setPoint
    * \param currentSpeed    the measuredValue
    * \return the controller output for wheel speed
    */
    tFloat64 getControllerValue(tFloat64 currentSpeed);
    void resetController();

    // PID-Controller values
    /*! proportional factor for PID Controller */
    property_variable<tFloat64>    m_f64PIDKp = 10;
    /*! integral factor for PID Controller */
    property_variable<tFloat64>    m_f64PIDKi = 0.85;
    /*! differential factor for PID Controller */
    property_variable<tFloat64>    m_f64PIDKd = 0.01;

    /*! fixed sampling frequency */
    property_variable<tUInt64> m_f64FixedSamplingFrequency = 50;

    /*! Gain */
    property_variable<tFloat64> m_f64Gain = 1;
    /*! the minimum output value for the controller */
    property_variable<tFloat64> m_f64PIDMinimumOutput = -20;
    /*! the maximum output value for the controller */
    property_variable<tFloat64> m_f64PIDMaximumOutput = 20;


    /*! The property for show debug */
    property_variable<tBool>       m_bShowDebug = tFalse;


public:

    /*! Default constructor. */
    fcWheelSpeedController();

    /*! Destructor. */
    virtual ~fcWheelSpeedController() = default;

    tResult Configure() override;

    /*!
     * Process the given tmTimeOfTrigger.
     *
     * \param   tmTimeOfTrigger The time time of trigger.
     *
     * \return  Standard Result Code.
     */
    tResult Process(tTimeStamp tmTimeOfTrigger) override;
};


//*************************************************************************************************
