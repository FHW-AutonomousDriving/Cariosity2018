#include "stdafx.h"
#include "SensorLidar.h"
#include <cmath>


ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_CARIOSITY_DATA_TRIGGERED_FILTER,
                                    "SensorLidar",
                                    fcSensorLidar,
                                    adtf::filter::pin_trigger({"input"}));
fcSensorLidar::fcSensorLidar() : cStdFilter() {
    // Register Pins
    Register(m_oReaderLaser, "input", getStreamType(LASER_SCANNER));
    Register(m_oWriterLaser, "output", getStreamType(LASER_SCANNER));

    // Register Properties

    RegisterPropertyVariable("Epsilon for removing zero values", m_propEpsilon);

}


//implement the Configure function to read ALL Properties
tResult fcSensorLidar::Configure() {
    RETURN_NOERROR;
}

tResult fcSensorLidar::Process(tTimeStamp tmTimeOfTrigger) {


    vector<tPolarCoordiante> scan;

    RETURN_IF_FAILED(readLaserScannerData(m_oReaderLaser, scan));

    vector<tPolarCoordiante> sanitized = sanitize(scan);

    RETURN_IF_FAILED(transmitLaserScannerData(sanitized, m_oWriterLaser));


    RETURN_NOERROR;
}
/*
vector<tPolarCoordiante> fcSensorLidar::sanitize(vector<tPolarCoordiante> scan) {
    vector<tPolarCoordiante> result;

    cMedian median = cMedian(5);

    if (scan.size() >= 2) {
        // add first two values
        median.pushValue(scan.at(0).f32Radius);
        median.pushValue(scan.at(1).f32Radius);
    } else if (!scan.empty()) {
        median.pushValue(scan.front().f32Radius);
        median.pushValue(scan.front().f32Radius);
    }
    for (tSize i = 0; i < scan.size() - 2; ++i) {

        if (i + 2 < scan.size()) {
            // take the next value into the median
            median.pushValue(scan.at(i + 2).f32Radius);
        } else {
            // take the last value into the median
            median.pushValue(scan.back().f32Radius);
        }

        tPolarCoordiante polarCoordinate = scan.at(i);
        polarCoordinate.f32Radius = median.calculateMedian();

        if (abs(polarCoordinate.f32Radius) > m_propEpsilon) {
            result.push_back(polarCoordinate);
        }
    }

    return result;
}
*/
vector<tPolarCoordiante> fcSensorLidar::sanitize(vector<tPolarCoordiante> scan) {
    vector<tPolarCoordiante> result;

    if (!scan.empty()) {

        cMedian median = cMedian(3);
        median.pushValue(scan.front().f32Radius);

        for (tSize i = 0; i < scan.size() - 1; ++i) {

            if (i + 1 < scan.size()) {
                // take the next value into the median
                median.pushValue(scan.at(i + 1).f32Radius);
            } else {
                // take the last value into the median
                median.pushValue(scan.back().f32Radius);
            }

            tPolarCoordiante polarCoordinate = scan.at(i);
            polarCoordinate.f32Radius = median.calculateMedian();

            if (abs(polarCoordinate.f32Radius) > m_propEpsilon) {
                result.push_back(polarCoordinate);
            }
        }
    }

    return result;
}