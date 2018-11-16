//
// Created by aadc on 26.09.18.
//

#pragma once
#include "./mapReader/openDriveReader.h"
#include "adtf3.h"
#include "../stdafx.h"
#include "../cariosity_structs.h"
#include "../cariosity_enums.h"

using namespace ODReader;
using namespace std;
using namespace fhw;
#define STRAIGHT_EPSILON (15)
#define RADIUS_OF_EQUALITY (0.75)
#define MAX_ENTRYPOINT_MERGE_RADIUS (0.3f)
#define HEADING_DIFF_OF_EQUALITY (30.0 * M_PI / 180.0)
#define MAX_ROAD_SIGN_MATCH_RADIUS (0.75)

class cOpenDriveMapAnalyzer {
private:
    /*! The od reader */
    ODReader::openDriveReader *m_odReader = nullptr;

public:
    cOpenDriveMapAnalyzer() = default;
    ~cOpenDriveMapAnalyzer() = default;

    //Minimum and maximum map coordinates
    tFloat32    m_minX = 99999999.0f, m_maxX = -99999999.0f,
                m_minY = 99999999.0f, m_maxY = -99999999.0f,
                m_minZ = 99999999.0f, m_maxZ = -99999999.0f;

    vector<tJunctionEntry> junctionEntries;
    vector<vector<Pose3D>> casualLanes;

    vector<tRoadSign> remainingRoadSigns;

    map<int, tRoadSign> parkingSpaces;

    tResult analyzeOpenDriveXML(const char *xml, tUInt8 numLineSegments);
    tResult analyzeRoadSignXML(const char *xml);

    void classifyAndAddToJunctionEntryIfNecessary(const roadElement &element, vector<Pose3D> lanePoses);
    tJunctionEntry* inRangeOfJunctionEntry(tVehiclePosition position);
    void calculateSize();

    void fuseRoadSignsIntoJunctionEntries();

    static tFloat32 calcDistance(Point a, Point b);
    static LaneTurnClassification classifyJunctionLane(Pose3D entryPoint, Pose3D exitPoint);
    static tVehiclePosition convertPoseToVehiclePosition(Pose3D pose);
    static tFloat32 normalizeAngleDegree(tFloat32 angle);
    static tFloat32 normalizeAngle(tFloat32 angle);


    static cOpenDriveMapAnalyzer *singleton;
    static tBool isInitialized();
};