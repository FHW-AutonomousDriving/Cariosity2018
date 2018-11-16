#pragma once

#define CID_CARIOSITY_DATA_TRIGGERED_FILTER "angle_to_servo.filter.user.aadc.cid"

using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;

#include "../../Helpers/StdFilter.h"

class fcAngleToServo : public cStdFilter {
private:

    /* the radius */
    cPinReader m_oInputAngle;
    /* the angle */
    cPinWriter m_oOutputServo;

    tFloat32 convertAngleToServoValue(tFloat32 angle);

    // Properties
    property_variable<tFloat32> m_f32MaxLeftAngle = 34.2;
    property_variable<tFloat32> m_f32MaxRightAngle = 34.2;

public:

    /*! Default constructor. */
    fcAngleToServo();

    /*! Destructor. */
    virtual ~fcAngleToServo() = default;

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
