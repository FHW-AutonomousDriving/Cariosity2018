//
// Created by aadc on 04.09.18.
//
#pragma once
#pragma GCC diagnostic ignored "-Wunused-variable"

#include "stdafx.h"
#include "cariosity_structs.h"
#include "ddlStructs.h"


using namespace adtf::ucom;
using namespace adtf::streaming;
using namespace adtf::filter;

using namespace fhw;

class cStdFilter : public cTriggerFunction {

public:
    enum PinType {
        BOOL_SIGNAL_VALUE,
        SIGNAL_VALUE,
        LASER_SCANNER,
        INER_MEAS_UNIT,
        ULTRASONIC,
        WHEEL,
        VOLTAGE,
        DRIVER_STATE,
        JURY_INSTRUCTION,
        MANEUVER_LIST,
        EGO_STATE,
        RECOGNIZED_OBJECT,
        POINTER_VALUE,
        LANES
    };


protected:
    /*! Default constructor. */
    cStdFilter();

    /*! Destructor. */
    virtual ~cStdFilter() = default;

    object_ptr<IStreamType> getStreamType(PinType type);


    // BOOL
    tResult readBoolSignalValue(cPinReader &inputPin, tBoolSignalValue &inputData);
    tResult transmitBoolSignalValue(tBoolSignalValue &outputData, cPinWriter &outputPin);


    // FLOAT
    tResult readSignalValue(cPinReader &inputPin, tSignalValue &inputData);
    tResult transmitSignalValue(tSignalValue &outputData, cPinWriter &outputPin);


    // LASER_SCANNER
    tResult readLaserScannerData(cPinReader &inputPin, std::vector<tPolarCoordiante> &scan);
    tResult transmitLaserScannerData(const std::vector<tPolarCoordiante> &scan, cPinWriter &outputPin);


    // IMU
    tResult readIMUData(cPinReader &inputPin, tInerMeasUnitData &inerMeasUnitData);
    tResult transmitIMUData(tInerMeasUnitData &inerMeasUnitData, cPinWriter &outputPin);


    // ULTRA_SONIC
    tResult readUltrasonicData(cPinReader &inputPin, tUltrasonicStruct &usData);
    tResult transmitUltrasonicStruct(tUltrasonicStruct &usData, cPinWriter &outputPin);


    // WHEEL
    tResult readWheelData(cPinReader &inputPin, tWheelData &wheelData);
    // tResult transmitWheelData(tWheelData &wheelData, cPinWriter &outputPin);

    // LANES
    tResult readLanesData(cPinReader &inputPin, tLanes &lanesData);
    tResult transmitLanesData(tLanes &lanesData, cPinWriter &outputPin);

    // JURY MODULE stuff ...

    // DRIVER
    tResult readDriverStateData(cPinReader &inputPin, tDriverStruct &driverStateData);
    tResult transmitDriverStateData(tDriverStruct &driverStateData, cPinWriter &outputPin);

    // JURY
    tResult readJuryInstructionData(cPinReader &inputPin, tJuryStruct &juryInstructionData);
    tResult transmitJuryInstructionData(tJuryStruct &juryInstructionData, cPinWriter &outputPin);

    // EGO_STATE
    tResult readEgoStateData(cPinReader &inputPin, tEgoState &egoStateData);
    tResult transmitEgoStateData(tEgoState &egoStateData, cPinWriter &outputPin);
    //NOT EGOSTATE; RECOG OBJECT.
    tResult readRecognizedObjectData(cPinReader &inputPin, tRecognizedObject &recognizedObjectData);
    tResult transmitRecognizedObjectData(tRecognizedObject &recognizedObjectData, cPinWriter &outputPin);

    tResult readPointerData(cPinReader &inputPin, tPointerValue &pointerValue);
    tResult transmitPointerData(tUInt64 pointerAddress, cPinWriter &outputPin);

private:

    object_ptr<IStreamType> getBoolSignalValuePtr();
    object_ptr<IStreamType> getSignalValuePtr();
    object_ptr<IStreamType> getLaserScannerPtr();
    object_ptr<IStreamType> getInerMeasUnitPtr();
    object_ptr<IStreamType> getUltraSonicPtr();
    object_ptr<IStreamType> getWheelPtr();
    object_ptr<IStreamType> getVoltagePtr();

    object_ptr<IStreamType> getDriverStatePtr();
    object_ptr<IStreamType> getJuryInstructionPtr();

// ------ own data types

    object_ptr<IStreamType> getEgoStatePtr();
    object_ptr<IStreamType> getRecognizedObjectPtr();
    object_ptr<IStreamType> getPointerValuePtr();
    object_ptr<IStreamType> getLanesPtr();


};