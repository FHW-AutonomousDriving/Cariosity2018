//
// Created by aadc on 04.09.18.
//

#include "stdafx.h"
#include "StdFilter.h"
#include "cariosity_structs.h"

using namespace adtf::util;

cStdFilter::cStdFilter() {}


object_ptr<IStreamType> cStdFilter::getBoolSignalValuePtr() {
    object_ptr<IStreamType> pTypeBoolValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tBoolSignalValue",
                                                                                       pTypeBoolValue,
                                                                                       g_BoolSignalValueSampleFactory)) {
        (adtf_ddl::access_element::find_index(g_BoolSignalValueSampleFactory, cString("ui32ArduinoTimestamp"),
                                              g_ddlBoolSignalValueIndex.timeStamp));
        (adtf_ddl::access_element::find_index(g_BoolSignalValueSampleFactory, cString("bValue"),
                                              g_ddlBoolSignalValueIndex.value));
    } else {
        LOG_WARNING("No mediadescription for tBoolValue found!");
    }
    return pTypeBoolValue;
}


object_ptr<IStreamType> cStdFilter::getSignalValuePtr() {
    object_ptr<IStreamType> pTypeSignalValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tSignalValue", pTypeSignalValue,
                                                                                       g_SignalValueSampleFactory)) {
        (adtf_ddl::access_element::find_index(g_SignalValueSampleFactory, cString("ui32ArduinoTimestamp"),
                                              g_ddlSignalValueIndex.timeStamp));
        (adtf_ddl::access_element::find_index(g_SignalValueSampleFactory, cString("f32Value"),
                                              g_ddlSignalValueIndex.value));
    } else {
        LOG_WARNING("No mediadescription for tSignalValue found!");
    }
    return pTypeSignalValue;
}

object_ptr<IStreamType> cStdFilter::getLaserScannerPtr() {
    object_ptr<IStreamType> pTypeLSData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tLaserScannerData", pTypeLSData,
                                                                                       g_LaserScannerDataSampleFactory)) {
        (adtf_ddl::access_element::find_index(g_LaserScannerDataSampleFactory, cString("ui32Size"),
                                              g_ddlLaserScannerDataIndex.size));
        (adtf_ddl::access_element::find_array_index(g_LaserScannerDataSampleFactory, cString("tScanArray"),
                                                    g_ddlLaserScannerDataIndex.scanArray));
    } else {
        LOG_WARNING("No mediadescription for tLaserScanner found!");
    }
    return pTypeLSData;
}

object_ptr<IStreamType> cStdFilter::getInerMeasUnitPtr() {
    object_ptr<IStreamType> pTypeIMUData;
    if IS_OK(
            adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tInerMeasUnitData", pTypeIMUData,
                                                                                      g_InerMeasUnitDataSampleFactory)) {
        adtf_ddl::access_element::find_index(g_InerMeasUnitDataSampleFactory, "ui32ArduinoTimestamp",
                                             g_ddlInerMeasUnitDataIndex.timeStamp);
        adtf_ddl::access_element::find_index(g_InerMeasUnitDataSampleFactory, "f32A_x", g_ddlInerMeasUnitDataIndex.A_x);
        adtf_ddl::access_element::find_index(g_InerMeasUnitDataSampleFactory, "f32A_y", g_ddlInerMeasUnitDataIndex.A_y);
        adtf_ddl::access_element::find_index(g_InerMeasUnitDataSampleFactory, "f32A_z", g_ddlInerMeasUnitDataIndex.A_z);
        adtf_ddl::access_element::find_index(g_InerMeasUnitDataSampleFactory, "f32G_x", g_ddlInerMeasUnitDataIndex.G_x);
        adtf_ddl::access_element::find_index(g_InerMeasUnitDataSampleFactory, "f32G_y", g_ddlInerMeasUnitDataIndex.G_y);
        adtf_ddl::access_element::find_index(g_InerMeasUnitDataSampleFactory, "f32G_z", g_ddlInerMeasUnitDataIndex.G_z);
        adtf_ddl::access_element::find_index(g_InerMeasUnitDataSampleFactory, "f32M_x", g_ddlInerMeasUnitDataIndex.M_x);
        adtf_ddl::access_element::find_index(g_InerMeasUnitDataSampleFactory, "f32M_y", g_ddlInerMeasUnitDataIndex.M_y);
        adtf_ddl::access_element::find_index(g_InerMeasUnitDataSampleFactory, "f32M_z", g_ddlInerMeasUnitDataIndex.M_z);
    } else {
        LOG_INFO("No mediadescription for tInerMeasUnitData found!");
    }
    return pTypeIMUData;
}

object_ptr<IStreamType> cStdFilter::getUltraSonicPtr() {
    object_ptr<IStreamType> pTypeUSData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tUltrasonicStruct", pTypeUSData,
                                                                                       g_UltrasonicDataSampleFactory)) {
        (adtf_ddl::access_element::find_index(g_UltrasonicDataSampleFactory,
                                              cString("tSideLeft") + cString(".ui32ArduinoTimestamp"),
                                              g_ddlUltrasonicDataIndex.SideLeft.timeStamp));
        (adtf_ddl::access_element::find_index(g_UltrasonicDataSampleFactory,
                                              cString("tSideLeft") + cString(".f32Value"),
                                              g_ddlUltrasonicDataIndex.SideLeft.value));
        (adtf_ddl::access_element::find_index(g_UltrasonicDataSampleFactory,
                                              cString("tSideRight") + cString(".ui32ArduinoTimestamp"),
                                              g_ddlUltrasonicDataIndex.SideRight.timeStamp));
        (adtf_ddl::access_element::find_index(g_UltrasonicDataSampleFactory,
                                              cString("tSideRight") + cString(".f32Value"),
                                              g_ddlUltrasonicDataIndex.SideRight.value));
        (adtf_ddl::access_element::find_index(g_UltrasonicDataSampleFactory,
                                              cString("tRearLeft") + cString(".ui32ArduinoTimestamp"),
                                              g_ddlUltrasonicDataIndex.RearLeft.timeStamp));
        (adtf_ddl::access_element::find_index(g_UltrasonicDataSampleFactory,
                                              cString("tRearLeft") + cString(".f32Value"),
                                              g_ddlUltrasonicDataIndex.RearLeft.value));
        (adtf_ddl::access_element::find_index(g_UltrasonicDataSampleFactory,
                                              cString("tRearCenter") + cString(".ui32ArduinoTimestamp"),
                                              g_ddlUltrasonicDataIndex.RearCenter.timeStamp));
        (adtf_ddl::access_element::find_index(g_UltrasonicDataSampleFactory,
                                              cString("tRearCenter") + cString(".f32Value"),
                                              g_ddlUltrasonicDataIndex.RearCenter.value));
        (adtf_ddl::access_element::find_index(g_UltrasonicDataSampleFactory,
                                              cString("tRearRight") + cString(".ui32ArduinoTimestamp"),
                                              g_ddlUltrasonicDataIndex.RearRight.timeStamp));
        (adtf_ddl::access_element::find_index(g_UltrasonicDataSampleFactory,
                                              cString("tRearRight") + cString(".f32Value"),
                                              g_ddlUltrasonicDataIndex.RearRight.value));
    } else {
        LOG_INFO("No mediadescription for tUltrasonicStruct found!");
    }
    return pTypeUSData;
}

object_ptr<IStreamType> cStdFilter::getWheelPtr() {
    object_ptr<IStreamType> pTypeWheelData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tWheelData", pTypeWheelData,
                                                                                       g_WheelDataSampleFactory)) {
        adtf_ddl::access_element::find_index(g_WheelDataSampleFactory, "ui32ArduinoTimestamp",
                                             g_ddlWheelDataIndex.timeStamp);
        adtf_ddl::access_element::find_index(g_WheelDataSampleFactory, "ui32WheelTach", g_ddlWheelDataIndex.WheelTach);
        adtf_ddl::access_element::find_index(g_WheelDataSampleFactory, "i8WheelDir", g_ddlWheelDataIndex.WheelDir);
    } else {
        LOG_INFO("No mediadescription for tWheelData found!");
    }
    return pTypeWheelData;
}

object_ptr<IStreamType> cStdFilter::getVoltagePtr() {
    object_ptr<IStreamType> pTypeVoltageStruct;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tVoltageStruct",
                                                                                       pTypeVoltageStruct,
                                                                                       g_VoltageDataSampleFactory)) {
        adtf_ddl::access_element::find_index(g_VoltageDataSampleFactory,
                                             cString("tActuatorVoltage") + cString(".ui32ArduinoTimestamp"),
                                             g_ddlVoltageDataIndex.ActuatorVoltage.timeStamp);
        adtf_ddl::access_element::find_index(g_VoltageDataSampleFactory,
                                             cString("tActuatorCell1") + cString(".ui32ArduinoTimestamp"),
                                             g_ddlVoltageDataIndex.ActuatorCell1.timeStamp);
        adtf_ddl::access_element::find_index(g_VoltageDataSampleFactory,
                                             cString("tActuatorCell2") + cString(".ui32ArduinoTimestamp"),
                                             g_ddlVoltageDataIndex.ActuatorCell2.timeStamp);
        adtf_ddl::access_element::find_index(g_VoltageDataSampleFactory,
                                             cString("tSensorVoltage") + cString(".ui32ArduinoTimestamp"),
                                             g_ddlVoltageDataIndex.SensorVoltage.timeStamp);
        adtf_ddl::access_element::find_index(g_VoltageDataSampleFactory,
                                             cString("tSensorCell1") + cString(".ui32ArduinoTimestamp"),
                                             g_ddlVoltageDataIndex.SensorCell1.timeStamp);
        adtf_ddl::access_element::find_index(g_VoltageDataSampleFactory,
                                             cString("tSensorCell2") + cString(".ui32ArduinoTimestamp"),
                                             g_ddlVoltageDataIndex.SensorCell2.timeStamp);
        adtf_ddl::access_element::find_index(g_VoltageDataSampleFactory,
                                             cString("tSensorCell3") + cString(".ui32ArduinoTimestamp"),
                                             g_ddlVoltageDataIndex.SensorCell3.timeStamp);
        adtf_ddl::access_element::find_index(g_VoltageDataSampleFactory,
                                             cString("tSensorCell4") + cString(".ui32ArduinoTimestamp"),
                                             g_ddlVoltageDataIndex.SensorCell4.timeStamp);
        adtf_ddl::access_element::find_index(g_VoltageDataSampleFactory,
                                             cString("tSensorCell5") + cString(".ui32ArduinoTimestamp"),
                                             g_ddlVoltageDataIndex.SensorCell5.timeStamp);
        adtf_ddl::access_element::find_index(g_VoltageDataSampleFactory,
                                             cString("tSensorCell6") + cString(".ui32ArduinoTimestamp"),
                                             g_ddlVoltageDataIndex.SensorCell6.timeStamp);

        adtf_ddl::access_element::find_index(g_VoltageDataSampleFactory,
                                             cString("tActuatorVoltage") + cString(".f32Value"),
                                             g_ddlVoltageDataIndex.ActuatorVoltage.value);
        adtf_ddl::access_element::find_index(g_VoltageDataSampleFactory,
                                             cString("tActuatorCell1") + cString(".f32Value"),
                                             g_ddlVoltageDataIndex.ActuatorCell1.value);
        adtf_ddl::access_element::find_index(g_VoltageDataSampleFactory,
                                             cString("tActuatorCell2") + cString(".f32Value"),
                                             g_ddlVoltageDataIndex.ActuatorCell2.value);
        adtf_ddl::access_element::find_index(g_VoltageDataSampleFactory,
                                             cString("tSensorVoltage") + cString(".f32Value"),
                                             g_ddlVoltageDataIndex.SensorVoltage.value);
        adtf_ddl::access_element::find_index(g_VoltageDataSampleFactory,
                                             cString("tSensorCell1") + cString(".f32Value"),
                                             g_ddlVoltageDataIndex.SensorCell1.value);
        adtf_ddl::access_element::find_index(g_VoltageDataSampleFactory,
                                             cString("tSensorCell2") + cString(".f32Value"),
                                             g_ddlVoltageDataIndex.SensorCell2.value);
        adtf_ddl::access_element::find_index(g_VoltageDataSampleFactory,
                                             cString("tSensorCell3") + cString(".f32Value"),
                                             g_ddlVoltageDataIndex.SensorCell3.value);
        adtf_ddl::access_element::find_index(g_VoltageDataSampleFactory,
                                             cString("tSensorCell4") + cString(".f32Value"),
                                             g_ddlVoltageDataIndex.SensorCell4.value);
        adtf_ddl::access_element::find_index(g_VoltageDataSampleFactory,
                                             cString("tSensorCell5") + cString(".f32Value"),
                                             g_ddlVoltageDataIndex.SensorCell5.value);
        adtf_ddl::access_element::find_index(g_VoltageDataSampleFactory,
                                             cString("tSensorCell6") + cString(".f32Value"),
                                             g_ddlVoltageDataIndex.SensorCell6.value);
    } else {
        LOG_INFO("No mediadescription for tVoltageStruct found!");
    }
    return pTypeVoltageStruct;
}

object_ptr<IStreamType> cStdFilter::getDriverStatePtr() {
    object_ptr<IStreamType> pTypeDriverStruct;
    if (IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tDriverStruct",
                                                                                        pTypeDriverStruct,
                                                                                        g_DriverDataSampleFactory))) {
        adtf_ddl::access_element::find_index(g_DriverDataSampleFactory, cString("i16StateID"),
                                             g_ddlDriverDataIndex.stateId);
        adtf_ddl::access_element::find_index(g_DriverDataSampleFactory, cString("i16ManeuverEntry"),
                                             g_ddlDriverDataIndex.maneuverEntry);
    } else {
        LOG_WARNING("No mediadescription for tDriverStruct found!");
    }
    return pTypeDriverStruct;
}

object_ptr<IStreamType> cStdFilter::getJuryInstructionPtr() {
    object_ptr<IStreamType> pTypeJuryStruct;
    if (IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tJuryStruct", pTypeJuryStruct,
                                                                                        g_JuryDataSampleFactory))) {
        adtf_ddl::access_element::find_index(g_JuryDataSampleFactory, cString("i16ActionID"),
                                             g_ddlJuryDataIndex.actionId);
        adtf_ddl::access_element::find_index(g_JuryDataSampleFactory, cString("i16ManeuverEntry"),
                                             g_ddlJuryDataIndex.maneuverEntry);
    } else {
        LOG_WARNING("No mediadescription for tJuryStruct found!");
    }
    return pTypeJuryStruct;
}

// ------ own data types

object_ptr<IStreamType> cStdFilter::getEgoStatePtr() {
    object_ptr<IStreamType> pTypeEgoSate;
    if (IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tEgoState", pTypeEgoSate,
                                                                                        g_EgoStateDataSampleFactory))) {
        adtf_ddl::access_element::find_index(g_EgoStateDataSampleFactory, cString("tPosVehiclePosition.f32x"),
                                             g_ddlEgoStateDataIndex.vehiclePosition.x);
        adtf_ddl::access_element::find_index(g_EgoStateDataSampleFactory, cString("tPosVehiclePosition.f32y"),
                                             g_ddlEgoStateDataIndex.vehiclePosition.y);
        adtf_ddl::access_element::find_index(g_EgoStateDataSampleFactory, cString("tPosVehiclePosition.f32radius"),
                                             g_ddlEgoStateDataIndex.vehiclePosition.radius);
        adtf_ddl::access_element::find_index(g_EgoStateDataSampleFactory, cString("tPosVehiclePosition.f32speed"),
                                             g_ddlEgoStateDataIndex.vehiclePosition.speed);
        adtf_ddl::access_element::find_index(g_EgoStateDataSampleFactory, cString("tPosVehiclePosition.f32heading"),
                                             g_ddlEgoStateDataIndex.vehiclePosition.heading);
        adtf_ddl::access_element::find_index(g_EgoStateDataSampleFactory, cString("f32TotalDistance"),
                                             g_ddlEgoStateDataIndex.totalDistance);
        adtf_ddl::access_element::find_index(g_EgoStateDataSampleFactory, cString("f32RelativeLanePositioning"),
                                             g_ddlEgoStateDataIndex.relativeLanePositioning);
        adtf_ddl::access_element::find_index(g_EgoStateDataSampleFactory, cString("i16EnvironmentSituation"),
                                             g_ddlEgoStateDataIndex.environmentSituation);

    } else {
        LOG_WARNING("No mediadescription for tEgoState found!");
    }
    return pTypeEgoSate;
}

object_ptr<IStreamType> cStdFilter::getRecognizedObjectPtr() {
    object_ptr<IStreamType> pTypeRecognizedObjects;
    if (IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tRecognizedObject",
                                                                                        pTypeRecognizedObjects,
                                                                                        g_RecognizedObjectDataSampleFactory))) {
        adtf_ddl::access_element::find_index(g_RecognizedObjectDataSampleFactory, "ui8Category",
                                             g_ddlRecognizedObjectDataIndex.ui8Category);
        adtf_ddl::access_element::find_index(g_RecognizedObjectDataSampleFactory, "f32Angle",
                                             g_ddlRecognizedObjectDataIndex.f32Angle);
        adtf_ddl::access_element::find_index(g_RecognizedObjectDataSampleFactory, "f32Confidence",
                                             g_ddlRecognizedObjectDataIndex.f32Confidence);
    } else {
        LOG_WARNING("No mediadescription for tRegcognizedObject found!");
    }
    return pTypeRecognizedObjects;
}

object_ptr<IStreamType> cStdFilter::getPointerValuePtr() {
    object_ptr<IStreamType> pTypePointerValue;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service(
            "tPointerValue", pTypePointerValue, g_PointerValueFactory))
    {
        adtf_ddl::access_element::find_index(g_PointerValueFactory, cString("ui64Address"), g_ddlPointerValueIndex.address);
    } else {
        LOG_WARNING("No mediadescription for tPointerValue found!");
    }

    return pTypePointerValue;
}

// Lanes
object_ptr<IStreamType> cStdFilter::getLanesPtr() {
    object_ptr<IStreamType> pTypeLanesData;
    if IS_OK(adtf::mediadescription::ant::create_adtf_default_stream_type_from_service("tLanes", pTypeLanesData,
                                                                                       g_LanesDataSampleFactory)) {
        adtf_ddl::access_element::find_index(g_LanesDataSampleFactory, "fHasLaneToTheLeft",
                                             g_ddlLanesDataIndex.fHasLaneToTheLeft);
        adtf_ddl::access_element::find_index(g_LanesDataSampleFactory, "fIsOnLane",
                                             g_ddlLanesDataIndex.fIsOnLane);
        adtf_ddl::access_element::find_index(g_LanesDataSampleFactory, "fHasLaneToTheRight",
                                             g_ddlLanesDataIndex.fHasLaneToTheRight);
    } else {
        LOG_INFO("No mediadescription for tLanes found!");
    }
    return pTypeLanesData;
}

//
// BEGIN ACTUAL CODE
//


object_ptr<IStreamType> cStdFilter::getStreamType(PinType type) {
    object_ptr<IStreamType> pData = NULL;
    switch (type) {
        case BOOL_SIGNAL_VALUE:
            pData = getBoolSignalValuePtr();
            break;
        case SIGNAL_VALUE :
            pData = getSignalValuePtr();
            break;
        case LASER_SCANNER :
            pData = getLaserScannerPtr();
            break;
        case INER_MEAS_UNIT :
            pData = getInerMeasUnitPtr();
            break;
        case ULTRASONIC :
            pData = getUltraSonicPtr();
            break;
        case WHEEL :
            pData = getWheelPtr();
            break;
        case VOLTAGE :
            pData = getVoltagePtr();
            break;
        case DRIVER_STATE :
            pData = getDriverStatePtr();
            break;
        case JURY_INSTRUCTION :
            pData = getJuryInstructionPtr();
            break;
        case EGO_STATE:
            pData = getEgoStatePtr();
            break;
        case RECOGNIZED_OBJECT:
            pData = getRecognizedObjectPtr();
            break;
        case POINTER_VALUE:
            pData = getPointerValuePtr();
            break;
        case LANES :
            pData = getLanesPtr();
            break;
        default:
            LOG_ERROR("ERROR: No valid PinType provided! Failed registering Pin of type: %d", type);

    }
    return pData;
}

tResult cStdFilter::readBoolSignalValue(cPinReader &inputPin, tBoolSignalValue &inputData) {
    static mutex m;
    lock_guard<mutex> lock_guard(m);
    object_ptr<const ISample> pReadSample;

    RETURN_IF_FAILED(inputPin.GetLastSample(pReadSample));
    auto oDecoder = g_BoolSignalValueSampleFactory.MakeDecoderFor(*pReadSample);

    RETURN_IF_FAILED(oDecoder.IsValid());

    // retrieve the values (using convenience methods that return a variant)
    RETURN_IF_FAILED(oDecoder.GetElementValue(g_ddlBoolSignalValueIndex.value, &(inputData.bValue)));
    RETURN_IF_FAILED(oDecoder.GetElementValue(g_ddlBoolSignalValueIndex.timeStamp, &(inputData.ui32ArduinoTimestamp)));

    return ERR_NOERROR;
}

tResult cStdFilter::transmitBoolSignalValue(tBoolSignalValue &outputData, cPinWriter &outputPin) {
    static mutex m;
    lock_guard<mutex> lock_guard(m);
    object_ptr<ISample> pWriteSample;
    RETURN_IF_FAILED(alloc_sample(pWriteSample))
    {
        auto oCodec = g_BoolSignalValueSampleFactory.MakeCodecFor(pWriteSample);
        RETURN_IF_FAILED(oCodec.SetElementValue(g_ddlBoolSignalValueIndex.timeStamp, (outputData.ui32ArduinoTimestamp)));
        RETURN_IF_FAILED(oCodec.SetElementValue(g_ddlBoolSignalValueIndex.value, (outputData.bValue)));
    }
    outputPin << pWriteSample << flush << trigger;
    RETURN_NOERROR;
}

tResult cStdFilter::readSignalValue(cPinReader &inputPin, tSignalValue &inputData) {
    static mutex m;
    lock_guard<mutex> lock_guard(m);
    object_ptr<const ISample> pReadSample;

    RETURN_IF_FAILED(inputPin.GetLastSample(pReadSample));
    // TODO: does this actually work? intention: we would like to clear the queue to prevent it from growing to infinity
    // --> The cDynamicSampleReader will create a sample reader which will create a internal sample queue with unlimited size.
    // --> inputPin.Clear(); ?

    auto oDecoder = g_SignalValueSampleFactory.MakeDecoderFor(*pReadSample);

    RETURN_IF_FAILED(oDecoder.IsValid());

    // retrieve the values (using convenience methods that return a variant)
    RETURN_IF_FAILED(oDecoder.GetElementValue(g_ddlSignalValueIndex.value, &(inputData.f32Value)));
    RETURN_IF_FAILED(oDecoder.GetElementValue(g_ddlSignalValueIndex.timeStamp, &(inputData.ui32ArduinoTimestamp)));

    return ERR_NOERROR;
}

tResult cStdFilter::transmitSignalValue(tSignalValue &value, cPinWriter &outputPin) {
    static mutex m;
    lock_guard<mutex> lock_guard(m);
    object_ptr<ISample> pWriteSample;

    RETURN_IF_FAILED(alloc_sample(pWriteSample))
    {

        auto oCodec = g_SignalValueSampleFactory.MakeCodecFor(pWriteSample);
        RETURN_IF_FAILED(oCodec.SetElementValue(g_ddlSignalValueIndex.timeStamp, (value.ui32ArduinoTimestamp)));
        RETURN_IF_FAILED(oCodec.SetElementValue(g_ddlSignalValueIndex.value, (value.f32Value)));

    }
    outputPin << pWriteSample << flush << trigger;

    RETURN_NOERROR;
}


tResult cStdFilter::readLaserScannerData(cPinReader &inputPin, std::vector<tPolarCoordiante> &scan) {
    static mutex m;
    lock_guard<mutex> lock_guard(m);

    object_ptr<const ISample> pReadSampleLS;


    if (IS_OK(inputPin.GetLastSample(pReadSampleLS))) {
        auto oDecoder = g_LaserScannerDataSampleFactory.MakeDecoderFor(*pReadSampleLS);

        RETURN_IF_FAILED(oDecoder.IsValid());
        tSize numOfScanPoints = 0;
        RETURN_IF_FAILED(oDecoder.GetElementValue(g_ddlLaserScannerDataIndex.size, &numOfScanPoints));

        const tPolarCoordiante *polarCoordinates = reinterpret_cast<const tPolarCoordiante *>(oDecoder.GetElementAddress(
                g_ddlLaserScannerDataIndex.scanArray));

        tPolarCoordiante scanPoint;
        for (tSize i = 0; i < numOfScanPoints; ++i) {
            scanPoint.f32Radius = polarCoordinates[i].f32Radius;
            scanPoint.f32Angle = polarCoordinates[i].f32Angle;
            scan.push_back(scanPoint);
        }

    } else {
        LOG_ERROR("Laserscanner did not supply correct values - did it already start?");
    }

    RETURN_NOERROR;
}

tResult cStdFilter::transmitLaserScannerData(const std::vector<tPolarCoordiante> &scan, cPinWriter &outputPin) {
    static mutex m;
    lock_guard<mutex> lock_guard(m);
    object_ptr<ISample> pSample;
    RETURN_IF_FAILED(alloc_sample(pSample, 0));
    auto oCodec = g_LaserScannerDataSampleFactory.MakeCodecFor(pSample);

    oCodec.SetElementValue(g_ddlLaserScannerDataIndex.size, scan.size());

    tPolarCoordiante *pCoordinates = reinterpret_cast<tPolarCoordiante *>(oCodec.GetElementAddress(
            g_ddlLaserScannerDataIndex.scanArray));

    //init array with zeros
    memset(pCoordinates, 0, scan.size() * sizeof(tPolarCoordiante));

    tUInt32 nIdx = 0;
    for (const tPolarCoordiante &pPolarCoordinate : scan) {
        pCoordinates[nIdx].f32Angle = pPolarCoordinate.f32Angle;
        pCoordinates[nIdx++].f32Radius = pPolarCoordinate.f32Radius;
    }

    outputPin << pSample << flush << trigger;
    RETURN_NOERROR;
}


tResult cStdFilter::readUltrasonicData(cPinReader &inputPin, tUltrasonicStruct &usData) {
    static mutex m;
    lock_guard<mutex> lock_guard(m);

    object_ptr<const ISample> pSampleFromUS;

    RETURN_IF_FAILED(inputPin.GetLastSample(pSampleFromUS));
    auto oDecoderUS = g_UltrasonicDataSampleFactory.MakeDecoderFor(*pSampleFromUS);

    RETURN_IF_FAILED(oDecoderUS.IsValid());

    RETURN_IF_FAILED(oDecoderUS.GetElementValue(g_ddlUltrasonicDataIndex.SideLeft.timeStamp,
                                                &usData.tSideLeft.ui32ArduinoTimestamp));
    RETURN_IF_FAILED(oDecoderUS.GetElementValue(g_ddlUltrasonicDataIndex.SideLeft.value, &usData.tSideLeft.f32Value));
    RETURN_IF_FAILED(oDecoderUS.GetElementValue(g_ddlUltrasonicDataIndex.SideRight.timeStamp,
                                                &usData.tSideRight.ui32ArduinoTimestamp));
    RETURN_IF_FAILED(oDecoderUS.GetElementValue(g_ddlUltrasonicDataIndex.SideRight.value, &usData.tSideRight.f32Value));
    RETURN_IF_FAILED(oDecoderUS.GetElementValue(g_ddlUltrasonicDataIndex.RearLeft.timeStamp,
                                                &usData.tRearLeft.ui32ArduinoTimestamp));
    RETURN_IF_FAILED(oDecoderUS.GetElementValue(g_ddlUltrasonicDataIndex.RearLeft.value, &usData.tRearLeft.f32Value));
    RETURN_IF_FAILED(oDecoderUS.GetElementValue(g_ddlUltrasonicDataIndex.RearCenter.timeStamp,
                                                &usData.tRearCenter.ui32ArduinoTimestamp));
    RETURN_IF_FAILED(
            oDecoderUS.GetElementValue(g_ddlUltrasonicDataIndex.RearCenter.value, &usData.tRearCenter.f32Value));
    RETURN_IF_FAILED(oDecoderUS.GetElementValue(g_ddlUltrasonicDataIndex.RearRight.timeStamp,
                                                &usData.tRearRight.ui32ArduinoTimestamp));
    RETURN_IF_FAILED(oDecoderUS.GetElementValue(g_ddlUltrasonicDataIndex.RearRight.value, &usData.tRearRight.f32Value));

    RETURN_NOERROR;
}

tResult cStdFilter::transmitUltrasonicStruct(tUltrasonicStruct &usData, cPinWriter &outputPin) {
    static mutex m;
    lock_guard<mutex> lock_guard(m);

    object_ptr<ISample> pUltrasonicStructWriteSample;

    RETURN_IF_FAILED(alloc_sample(pUltrasonicStructWriteSample))

    auto oCodec = g_UltrasonicDataSampleFactory.MakeCodecFor(pUltrasonicStructWriteSample);
    RETURN_IF_FAILED(oCodec.SetElementValue(g_ddlUltrasonicDataIndex.SideLeft.timeStamp,
                                            &usData.tSideLeft.ui32ArduinoTimestamp));
    RETURN_IF_FAILED(oCodec.SetElementValue(g_ddlUltrasonicDataIndex.SideLeft.value, &usData.tSideLeft.f32Value));
    RETURN_IF_FAILED(oCodec.SetElementValue(g_ddlUltrasonicDataIndex.SideRight.timeStamp,
                                            &usData.tSideRight.ui32ArduinoTimestamp));
    RETURN_IF_FAILED(oCodec.SetElementValue(g_ddlUltrasonicDataIndex.SideRight.value, &usData.tSideRight.f32Value));
    RETURN_IF_FAILED(oCodec.SetElementValue(g_ddlUltrasonicDataIndex.RearLeft.timeStamp,
                                            &usData.tRearLeft.ui32ArduinoTimestamp));
    RETURN_IF_FAILED(oCodec.SetElementValue(g_ddlUltrasonicDataIndex.RearLeft.value, &usData.tRearLeft.f32Value));
    RETURN_IF_FAILED(oCodec.SetElementValue(g_ddlUltrasonicDataIndex.RearCenter.timeStamp,
                                            &usData.tRearCenter.ui32ArduinoTimestamp));
    RETURN_IF_FAILED(oCodec.SetElementValue(g_ddlUltrasonicDataIndex.RearCenter.value, &usData.tRearCenter.f32Value));
    RETURN_IF_FAILED(oCodec.SetElementValue(g_ddlUltrasonicDataIndex.RearRight.timeStamp,
                                            &usData.tRearRight.ui32ArduinoTimestamp));
    RETURN_IF_FAILED(oCodec.SetElementValue(g_ddlUltrasonicDataIndex.RearRight.value, &usData.tRearRight.f32Value));

    outputPin << pUltrasonicStructWriteSample << flush << trigger;
    RETURN_NOERROR;
}

tResult cStdFilter::readIMUData(cPinReader &inputPin, tInerMeasUnitData &inerMeasUnitData) {
    static mutex m;
    lock_guard<mutex> lock_guard(m);

    object_ptr<const ISample> pIMUDataSample;

    RETURN_IF_FAILED(inputPin.GetLastSample(pIMUDataSample));
    auto oDecoderIMU = g_InerMeasUnitDataSampleFactory.MakeDecoderFor(*pIMUDataSample);

    RETURN_IF_FAILED(oDecoderIMU.IsValid());

    RETURN_IF_FAILED(
            oDecoderIMU.GetElementValue(g_ddlInerMeasUnitDataIndex.timeStamp, &inerMeasUnitData.ui32ArduinoTimestamp));
    RETURN_IF_FAILED(oDecoderIMU.GetElementValue(g_ddlInerMeasUnitDataIndex.A_x, &inerMeasUnitData.f32A_x));
    RETURN_IF_FAILED(oDecoderIMU.GetElementValue(g_ddlInerMeasUnitDataIndex.A_y, &inerMeasUnitData.f32A_y));
    RETURN_IF_FAILED(oDecoderIMU.GetElementValue(g_ddlInerMeasUnitDataIndex.A_z, &inerMeasUnitData.f32A_z));
    RETURN_IF_FAILED(oDecoderIMU.GetElementValue(g_ddlInerMeasUnitDataIndex.G_x, &inerMeasUnitData.f32G_x));
    RETURN_IF_FAILED(oDecoderIMU.GetElementValue(g_ddlInerMeasUnitDataIndex.G_y, &inerMeasUnitData.f32G_y));
    RETURN_IF_FAILED(oDecoderIMU.GetElementValue(g_ddlInerMeasUnitDataIndex.G_z, &inerMeasUnitData.f32G_z));
    RETURN_IF_FAILED(oDecoderIMU.GetElementValue(g_ddlInerMeasUnitDataIndex.M_x, &inerMeasUnitData.f32M_x));
    RETURN_IF_FAILED(oDecoderIMU.GetElementValue(g_ddlInerMeasUnitDataIndex.M_y, &inerMeasUnitData.f32M_y));
    RETURN_IF_FAILED(oDecoderIMU.GetElementValue(g_ddlInerMeasUnitDataIndex.M_z, &inerMeasUnitData.f32M_z));

    RETURN_NOERROR;
}

tResult cStdFilter::transmitIMUData(tInerMeasUnitData &inerMeasUnitData, cPinWriter &outputPin) {
    static mutex m;
    lock_guard<mutex> lock_guard(m);

    object_ptr<ISample> pIMUDataSample;

    RETURN_IF_FAILED(alloc_sample(pIMUDataSample))

    auto oCodec = g_InerMeasUnitDataSampleFactory.MakeCodecFor(pIMUDataSample);

    RETURN_IF_FAILED(
            oCodec.SetElementValue(g_ddlInerMeasUnitDataIndex.timeStamp, &inerMeasUnitData.ui32ArduinoTimestamp));
    RETURN_IF_FAILED(oCodec.SetElementValue(g_ddlInerMeasUnitDataIndex.A_x, &inerMeasUnitData.f32A_x));
    RETURN_IF_FAILED(oCodec.SetElementValue(g_ddlInerMeasUnitDataIndex.A_y, &inerMeasUnitData.f32A_y));
    RETURN_IF_FAILED(oCodec.SetElementValue(g_ddlInerMeasUnitDataIndex.A_z, &inerMeasUnitData.f32A_z));
    RETURN_IF_FAILED(oCodec.SetElementValue(g_ddlInerMeasUnitDataIndex.G_x, &inerMeasUnitData.f32G_x));
    RETURN_IF_FAILED(oCodec.SetElementValue(g_ddlInerMeasUnitDataIndex.G_y, &inerMeasUnitData.f32G_y));
    RETURN_IF_FAILED(oCodec.SetElementValue(g_ddlInerMeasUnitDataIndex.G_z, &inerMeasUnitData.f32G_z));
    RETURN_IF_FAILED(oCodec.SetElementValue(g_ddlInerMeasUnitDataIndex.M_x, &inerMeasUnitData.f32M_x));
    RETURN_IF_FAILED(oCodec.SetElementValue(g_ddlInerMeasUnitDataIndex.M_y, &inerMeasUnitData.f32M_y));
    RETURN_IF_FAILED(oCodec.SetElementValue(g_ddlInerMeasUnitDataIndex.M_z, &inerMeasUnitData.f32M_z));

    outputPin << pIMUDataSample << flush << trigger;
    RETURN_NOERROR;
}

tResult cStdFilter::readWheelData(cPinReader &inputPin, tWheelData &wheelData) {
    static mutex m;
    lock_guard<mutex> lock_guard(m);
    object_ptr<const ISample> pSampleWheelData;

    RETURN_IF_FAILED(inputPin.GetLastSample(pSampleWheelData));
    auto oDecoder = g_WheelDataSampleFactory.MakeDecoderFor(*pSampleWheelData);
    RETURN_IF_FAILED(oDecoder.IsValid());
    RETURN_IF_FAILED(oDecoder.GetElementValue(g_ddlWheelDataIndex.WheelTach, &wheelData.ui32WheelTach));
    RETURN_IF_FAILED(oDecoder.GetElementValue(g_ddlWheelDataIndex.timeStamp, &wheelData.ui32ArduinoTimestamp));
    RETURN_IF_FAILED(oDecoder.GetElementValue(g_ddlWheelDataIndex.WheelDir, &wheelData.i8WheelDir));

    RETURN_NOERROR;

}


// ------ jury module interaction

tResult cStdFilter::readDriverStateData(cPinReader &inputPin, tDriverStruct &driverStateData) {
    static mutex m;
    lock_guard<mutex> lock_guard(m);
    object_ptr<const ISample> pReadSample;

    RETURN_IF_FAILED(inputPin.GetNextSample(pReadSample));
    auto oDecoder = g_DriverDataSampleFactory.MakeDecoderFor(*pReadSample);

    RETURN_IF_FAILED(oDecoder.IsValid());

    // retrieve the values (using convenience methods that return a variant)
    RETURN_IF_FAILED(oDecoder.GetElementValue(g_ddlDriverDataIndex.stateId, &driverStateData.i16StateID));
    RETURN_IF_FAILED(oDecoder.GetElementValue(g_ddlDriverDataIndex.maneuverEntry, &driverStateData.i16ManeuverEntry));

    inputPin.Clear();

    return ERR_NOERROR;
}

tResult cStdFilter::transmitDriverStateData(tDriverStruct &driverStateData, cPinWriter &outputPin) {
    static mutex m;
    lock_guard<mutex> lock_guard(m);
    object_ptr<ISample> pWriteSample;

    RETURN_IF_FAILED(alloc_sample(pWriteSample));

    auto oCodec = g_DriverDataSampleFactory.MakeCodecFor(pWriteSample);
    RETURN_IF_FAILED(oCodec.SetElementValue(g_ddlDriverDataIndex.stateId, &driverStateData.i16StateID));
    RETURN_IF_FAILED(oCodec.SetElementValue(g_ddlDriverDataIndex.maneuverEntry, &driverStateData.i16ManeuverEntry));

    outputPin << pWriteSample << flush << trigger;

    RETURN_NOERROR;
}

tResult cStdFilter::readJuryInstructionData(cPinReader &inputPin, tJuryStruct &juryInstructionData) {
    static mutex m;
    lock_guard<mutex> lock_guard(m);
    object_ptr<const ISample> pReadSample;

    RETURN_IF_FAILED(inputPin.GetNextSample(pReadSample));
    auto oDecoder = g_JuryDataSampleFactory.MakeDecoderFor(*pReadSample);

    RETURN_IF_FAILED(oDecoder.IsValid());

    // retrieve the values (using convenience methods that return a variant)
    RETURN_IF_FAILED(oDecoder.GetElementValue(g_ddlJuryDataIndex.maneuverEntry, &juryInstructionData.i16ManeuverEntry));
    RETURN_IF_FAILED(oDecoder.GetElementValue(g_ddlJuryDataIndex.actionId, &juryInstructionData.i16ActionID));

    inputPin.Clear();

    return ERR_NOERROR;
}

tResult cStdFilter::transmitJuryInstructionData(tJuryStruct &juryInstructionData, cPinWriter &outputPin) {
    static mutex m;
    lock_guard<mutex> lock_guard(m);
    object_ptr<ISample> pWriteSample;

    RETURN_IF_FAILED(alloc_sample(pWriteSample));

    auto oCodec = g_JuryDataSampleFactory.MakeCodecFor(pWriteSample);
    RETURN_IF_FAILED(oCodec.SetElementValue(g_ddlJuryDataIndex.maneuverEntry, &juryInstructionData.i16ManeuverEntry));
    RETURN_IF_FAILED(oCodec.SetElementValue(g_ddlJuryDataIndex.actionId, &juryInstructionData.i16ActionID));

    outputPin << pWriteSample << flush << trigger;

    RETURN_NOERROR;
}


// ------ own data types

tResult cStdFilter::readEgoStateData(cPinReader &inputPin, tEgoState &egoStateData) {
    static mutex m;
    lock_guard<mutex> lock_guard(m);
    object_ptr<const ISample> pReadSample;

    RETURN_IF_FAILED(inputPin.GetNextSample(pReadSample));
    auto oDecoder = g_EgoStateDataSampleFactory.MakeDecoderFor(*pReadSample);

    RETURN_IF_FAILED(oDecoder.IsValid());

    // retrieve the values (using convenience methods that return a variant)
    RETURN_IF_FAILED(oDecoder.GetElementValue(g_ddlEgoStateDataIndex.vehiclePosition.x, &egoStateData.tPosVehiclePosition.f32x));
    RETURN_IF_FAILED(oDecoder.GetElementValue(g_ddlEgoStateDataIndex.vehiclePosition.y, &egoStateData.tPosVehiclePosition.f32y));
    RETURN_IF_FAILED(oDecoder.GetElementValue(g_ddlEgoStateDataIndex.vehiclePosition.radius, &egoStateData.tPosVehiclePosition.f32radius));
    RETURN_IF_FAILED(oDecoder.GetElementValue(g_ddlEgoStateDataIndex.vehiclePosition.speed, &egoStateData.tPosVehiclePosition.f32speed));
    RETURN_IF_FAILED(oDecoder.GetElementValue(g_ddlEgoStateDataIndex.vehiclePosition.heading, &egoStateData.tPosVehiclePosition.f32heading));
    RETURN_IF_FAILED(oDecoder.GetElementValue(g_ddlEgoStateDataIndex.totalDistance, &egoStateData.f32TotalDistance));
    RETURN_IF_FAILED(oDecoder.GetElementValue(g_ddlEgoStateDataIndex.relativeLanePositioning,
                                              &egoStateData.f32RelativeLanePositioning));
    RETURN_IF_FAILED(oDecoder.GetElementValue(g_ddlEgoStateDataIndex.environmentSituation,
                                              &egoStateData.i16EnvironmentSituation));

    return ERR_NOERROR;
}

tResult cStdFilter::transmitEgoStateData(tEgoState &egoStateData, cPinWriter &outputPin) {
    static mutex m;
    lock_guard<mutex> lock_guard(m);
    object_ptr<ISample> pWriteSample;

    RETURN_IF_FAILED(alloc_sample(pWriteSample));

    auto oCodec = g_EgoStateDataSampleFactory.MakeCodecFor(pWriteSample);

    RETURN_IF_FAILED(oCodec.SetElementValue(g_ddlEgoStateDataIndex.vehiclePosition.x, &egoStateData.tPosVehiclePosition.f32x));
    RETURN_IF_FAILED(oCodec.SetElementValue(g_ddlEgoStateDataIndex.vehiclePosition.y, &egoStateData.tPosVehiclePosition.f32y));
    RETURN_IF_FAILED(oCodec.SetElementValue(g_ddlEgoStateDataIndex.vehiclePosition.radius, &egoStateData.tPosVehiclePosition.f32radius));
    RETURN_IF_FAILED(oCodec.SetElementValue(g_ddlEgoStateDataIndex.vehiclePosition.speed, &egoStateData.tPosVehiclePosition.f32speed));
    RETURN_IF_FAILED(oCodec.SetElementValue(g_ddlEgoStateDataIndex.vehiclePosition.heading, &egoStateData.tPosVehiclePosition.f32heading));
    RETURN_IF_FAILED(oCodec.SetElementValue(g_ddlEgoStateDataIndex.totalDistance, &egoStateData.f32TotalDistance));
    RETURN_IF_FAILED(oCodec.SetElementValue(g_ddlEgoStateDataIndex.relativeLanePositioning,
                                            &egoStateData.f32RelativeLanePositioning));
    RETURN_IF_FAILED(oCodec.SetElementValue(g_ddlEgoStateDataIndex.environmentSituation, &egoStateData.i16EnvironmentSituation));

    outputPin << pWriteSample << flush << trigger;

    RETURN_NOERROR;
}


tResult cStdFilter::readPointerData(cPinReader &inputPin, tPointerValue &pointerValue) {
    static mutex m;
    lock_guard<mutex> lock_guard(m);

    object_ptr<const ISample> pReadSample;
    RETURN_IF_FAILED(inputPin.GetNextSample(pReadSample));

    auto oDecoder = g_PointerValueFactory.MakeDecoderFor(*pReadSample);
    RETURN_IF_FAILED(oDecoder.IsValid());

    // retrieve the values (using convenience methods that return a variant)
    RETURN_IF_FAILED(oDecoder.GetElementValue(g_ddlPointerValueIndex.address, &(pointerValue.address)));

    RETURN_NOERROR;
}


tResult cStdFilter::transmitPointerData(tUInt64 pointerAddress, cPinWriter &outputPin) {
    tPointerValue value = {pointerAddress};

    static mutex m;
    lock_guard<mutex> lock_guard(m);

    object_ptr<ISample> pWriteSample;
    RETURN_IF_FAILED(alloc_sample(pWriteSample));
    {
        auto oCodec = g_PointerValueFactory.MakeCodecFor(pWriteSample);
        //this no longer crashes :D
        RETURN_IF_FAILED(oCodec.SetElementValue(g_ddlPointerValueIndex.address, &value.address));
    }
    outputPin << pWriteSample << flush << trigger;
    RETURN_NOERROR;
}
tResult cStdFilter::readRecognizedObjectData(cPinReader &inputPin, tRecognizedObject &recognizedObjectData)  {
    object_ptr<const ISample> pReadSample;
    static mutex m;
    lock_guard<mutex> lock_guard(m);


    RETURN_IF_FAILED(inputPin.GetNextSample(pReadSample));
    auto oDecoder = g_RecognizedObjectDataSampleFactory.MakeDecoderFor(*pReadSample);

    RETURN_IF_FAILED(oDecoder.IsValid());

    // retrieve the values (using convenience methods that return a variant)
    RETURN_IF_FAILED(oDecoder.GetElementValue(g_ddlRecognizedObjectDataIndex.ui8Category, &recognizedObjectData.ui8Category));
    RETURN_IF_FAILED(oDecoder.GetElementValue(g_ddlRecognizedObjectDataIndex.f32Angle, &recognizedObjectData.f32Angle));
    RETURN_IF_FAILED(oDecoder.GetElementValue(g_ddlRecognizedObjectDataIndex.f32Confidence, &recognizedObjectData.f32Confidence));

    return ERR_NOERROR;
}

tResult cStdFilter::transmitRecognizedObjectData(tRecognizedObject &recognizedObjectData, cPinWriter &outputPin) {
    static mutex m;
    lock_guard<mutex> lock_guard(m);

    object_ptr<ISample> pWriteSample;

    RETURN_IF_FAILED(alloc_sample(pWriteSample));

    auto oCodec = g_RecognizedObjectDataSampleFactory.MakeCodecFor(pWriteSample);

    RETURN_IF_FAILED(oCodec.SetElementValue(g_ddlRecognizedObjectDataIndex.ui8Category, &recognizedObjectData.ui8Category));
    RETURN_IF_FAILED(oCodec.SetElementValue(g_ddlRecognizedObjectDataIndex.f32Angle, &recognizedObjectData.f32Angle));
    RETURN_IF_FAILED(oCodec.SetElementValue(g_ddlRecognizedObjectDataIndex.f32Confidence, &recognizedObjectData.f32Confidence));

    outputPin << pWriteSample << flush << trigger;
    RETURN_NOERROR;
}


// LANES
tResult cStdFilter::readLanesData(cPinReader &inputPin, tLanes &lanesData) {
    static mutex m;
    lock_guard<mutex> lock_guard(m);
    object_ptr<const ISample> pSampleLanesData;

    RETURN_IF_FAILED(inputPin.GetLastSample(pSampleLanesData));
    auto oDecoder = g_LanesDataSampleFactory.MakeDecoderFor(*pSampleLanesData);
    RETURN_IF_FAILED(oDecoder.IsValid());

    RETURN_IF_FAILED(oDecoder.GetElementValue(g_ddlLanesDataIndex.fHasLaneToTheLeft, &lanesData.fHasLaneToTheLeft));
    RETURN_IF_FAILED(oDecoder.GetElementValue(g_ddlLanesDataIndex.fIsOnLane, &lanesData.fIsOnLane));
    RETURN_IF_FAILED(oDecoder.GetElementValue(g_ddlLanesDataIndex.fHasLaneToTheRight, &lanesData.fHasLaneToTheRight));

    RETURN_NOERROR;
}

tResult cStdFilter::transmitLanesData(tLanes &lanesData, cPinWriter &outputPin) {
    static mutex m;
    lock_guard<mutex> lock_guard(m);
    object_ptr<ISample> pWriteLanesSample;
    RETURN_IF_FAILED(alloc_sample(pWriteLanesSample))
    {
        auto oCodec = g_LanesDataSampleFactory.MakeCodecFor(pWriteLanesSample);
        RETURN_IF_FAILED(oCodec.SetElementValue(g_ddlLanesDataIndex.fHasLaneToTheLeft, (lanesData.fHasLaneToTheLeft)));
        RETURN_IF_FAILED(oCodec.SetElementValue(g_ddlLanesDataIndex.fIsOnLane, (lanesData.fIsOnLane)));
        RETURN_IF_FAILED(oCodec.SetElementValue(g_ddlLanesDataIndex.fHasLaneToTheRight, (lanesData.fHasLaneToTheRight)));
    }
    outputPin << pWriteLanesSample << flush << trigger;
    RETURN_NOERROR;
}

