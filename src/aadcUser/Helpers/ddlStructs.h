/*
 * Definition off all DDL Structs
 */

#pragma once
#pragma GCC diagnostic ignored "-Wunused-variable"

#include "stdafx.h"

using namespace adtf::mediadescription;

namespace fhw {


    static struct tDDLBoolSignalValueIndex
    {
        tSize timeStamp;
        tSize value;
    } g_ddlBoolSignalValueIndex;
    static cSampleCodecFactory g_BoolSignalValueSampleFactory;


    /*! A ddl identifier for the signalvalue */
    static struct tDDLSignalValueIndex
    {
        tSize timeStamp;
        tSize value;
    } g_ddlSignalValueIndex;
    static cSampleCodecFactory g_SignalValueSampleFactory;


    /*! A ddl identifier for the laser scanner struct - tLaserScannerData */
    static struct tDDLLaserScannerDataIndex
    {
        tSize size;
        tSize scanArray;
    } g_ddlLaserScannerDataIndex;
    static cSampleCodecFactory g_LaserScannerDataSampleFactory;



    /*! A ddl iner meas unit data index - tInerMeasUnitData */
    static struct tDDLInerMeasUnitDataIndex
    {
        tSize timeStamp;
        tSize A_x;
        tSize A_y;
        tSize A_z;
        tSize G_x;
        tSize G_y;
        tSize G_z;
        tSize M_x;
        tSize M_y;
        tSize M_z;
    } g_ddlInerMeasUnitDataIndex;

    /*! The imu data sample factory */
    static cSampleCodecFactory g_InerMeasUnitDataSampleFactory;


    /* Felix Ende */

    /*! A ddl ultrasonic structure index - tUltrasonicStruct */
    static struct tDDLUltrasonicDataIndex
    {
        tDDLSignalValueIndex SideLeft;
        tDDLSignalValueIndex SideRight;
        tDDLSignalValueIndex RearLeft;
        tDDLSignalValueIndex RearCenter;
        tDDLSignalValueIndex RearRight;

    } g_ddlUltrasonicDataIndex;
    static cSampleCodecFactory g_UltrasonicDataSampleFactory;



    /*! A ddl wheel data index - tWheelData */
    static struct tDDLWheelDataIndex
    {
        tSize timeStamp;
        tSize WheelTach;
        tSize WheelDir;
    } g_ddlWheelDataIndex;
    static cSampleCodecFactory g_WheelDataSampleFactory;



    /*! A ddl voltage structure index - tVoltageStruct */
    static struct tDDLVoltageDataIndex
    {
        tDDLSignalValueIndex ActuatorVoltage;
        tDDLSignalValueIndex ActuatorCell1;
        tDDLSignalValueIndex ActuatorCell2;
        tDDLSignalValueIndex SensorVoltage;
        tDDLSignalValueIndex SensorCell1;
        tDDLSignalValueIndex SensorCell2;
        tDDLSignalValueIndex SensorCell3;
        tDDLSignalValueIndex SensorCell4;
        tDDLSignalValueIndex SensorCell5;
        tDDLSignalValueIndex SensorCell6;
    } g_ddlVoltageDataIndex;


    /*! The voltage structure sample factory */
    static cSampleCodecFactory g_VoltageDataSampleFactory;


    /*! A ddl egoState structure index - tVehiclePosition (tPosition)*/
    static struct tDDLPositionDataIndex
    {
        tSize x;
        tSize y;
        tSize radius;
        tSize speed;
        tSize heading;
    } g_ddlPositionDataIndex;

    /*! A ddl egoState structure index - tEgoState */
    static struct tDDLEgoStateDataIndex
    {
        tDDLPositionDataIndex vehiclePosition;
        tSize totalDistance;
        tSize relativeLanePositioning;
        tSize environmentSituation;
    } g_ddlEgoStateDataIndex;

    /*! The voltage structure sample factory */
    static cSampleCodecFactory g_EgoStateDataSampleFactory;

    /*! A ddl regcognized object structure index - YOLO */
    static struct tDDLRecognizedObjectDataIndex
    {
        tSize ui8Category;
        tSize f32Angle;
        tSize f32Confidence;
    } g_ddlRecognizedObjectDataIndex;

    /*! The regconized object structure sample factory */
    static cSampleCodecFactory g_RecognizedObjectDataSampleFactory;


    /*! A ddl lanes structure index - tLanes */
    static struct tDDLLanesDataIndex
    {
        tSize fHasLaneToTheLeft;
        tSize fIsOnLane;
        tSize fHasLaneToTheRight;
    } g_ddlLanesDataIndex;

    /*! The ddl lanes structure data sample factory */
    static cSampleCodecFactory g_LanesDataSampleFactory;


// ------ Jury Module Communication ------

    /*! A ddl jury structure index - tJuryStruct */
    static struct tDDLJuryDataIndex
    {
        tSize actionId;         // stop(-1), getready(0), start(1)
        tSize maneuverEntry;    // absolute id of maneuver in maneuverlist
    } g_ddlJuryDataIndex;

    /*! The jury structure data sample factory */
    static cSampleCodecFactory g_JuryDataSampleFactory;



    /*! A ddl driver structure index - tDriverStruct */
    static struct tDDLDriverDataIndex
    {
        tSize stateId;          // startup(-2), erroro(-1), redy(0), running(1), complete(2)
        tSize maneuverEntry;    // absolute id of maneuver in maneuverlist
    } g_ddlDriverDataIndex;

    /*! The driver structure data sample factory */
    static cSampleCodecFactory g_DriverDataSampleFactory;


// ------- Pointer

    static struct tDDLPointerValueIndex
    {
        tSize address;
    } g_ddlPointerValueIndex;

    static cSampleCodecFactory g_PointerValueFactory;
}

