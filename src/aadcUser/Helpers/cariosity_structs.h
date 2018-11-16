#pragma once

#include <adtf_utils.h>
#include <aadc_structs.h>
#include "cariosity_enums.h"
#include "./OpenDriveAnalyzer/mapReader/openDriveReader.h"

#pragma pack(push,1)
typedef struct
{
    tBool bHeadLight;
    tBool bTurnSignalLeft;
    tBool bTurnSignalRight;
    tBool bBrakeLight;
    tBool bHazardLight;
    tBool bReverseLight;
} tLightStruct;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tFloat32 f32x;
    tFloat32 f32y;
    tFloat32 f32radius;
    tFloat32 f32speed;
    tFloat32 f32heading;
} tVehiclePosition;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
    tBool bEmergencyBrake;
    tFloat32 f32speed;
    tFloat32 f32steeringAngle;
    tLightStruct lights;
    fhw::extraManeuvers immediateManeuver;
} tTaskResult;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct
{
    tVehiclePosition tPosVehiclePosition;
    tFloat32 f32TotalDistance;
    tFloat32 f32RelativeLanePositioning;
    tInt16 i16EnvironmentSituation;
} tEgoState;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct
{
    tUInt8 ui8Category;
    tFloat32 f32Angle;
    tFloat32 f32Confidence;
} tRecognizedObject;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
    int id;
    int junctionID;
    ODReader::Pose3D entryPoint;
    fhw::RoadSignType roadSign;

    std::vector<ODReader::Pose3D> leftTurn;
    std::vector<ODReader::Pose3D> straight;
    std::vector<ODReader::Pose3D> rightTurn;
} tJunctionEntry;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
    fhw::RoadSignType type;
    tBool bInit;
    tFloat32 f32X;
    tFloat32 f32Y;
    tFloat32 f32Radius;
    tFloat32 f32Direction;
} tRoadSign;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
    tUInt64 address;
} tPointerValue;
#pragma pack(pop)

#pragma pack(push, 1)
typedef struct {
    tFloat32 fHasLaneToTheLeft;
    tFloat32 fIsOnLane;
    tFloat32 fHasLaneToTheRight;
} tLanes;
#pragma pack(pop)