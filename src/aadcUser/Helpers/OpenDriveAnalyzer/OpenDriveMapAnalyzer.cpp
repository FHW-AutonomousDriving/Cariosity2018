//
// Created by aadc on 26.09.18.
//

#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

#include "OpenDriveMapAnalyzer.h"
#include "../cariosity_structs.h"

#define SHOW_DEBUG (false)
#define CONSOLE_LOG_INFO(...) if (SHOW_DEBUG) LOG_ADD_ENTRY(A_UTILS_NS::log::tLogLevel::Info, __VA_ARGS__)

using namespace adtf::util;

cOpenDriveMapAnalyzer *cOpenDriveMapAnalyzer::singleton = nullptr;

tResult cOpenDriveMapAnalyzer::analyzeOpenDriveXML(const char *xml, tUInt8 numLineSegments) {
    //odReader = new ODReader::openDriveReader("/home/aadc/openDRIVE/aadc2017competition.xodr");

    m_odReader = new openDriveReader();

    //Check if reader is success
    if (!m_odReader->ParseText(xml, numLineSegments) || m_odReader->FileReadErr > 0) {
        LOG_WARNING("File Read Error: %d", m_odReader->FileReadErr);
        RETURN_NOERROR;
    }

    junctionEntries = vector<tJunctionEntry>();
    casualLanes = vector<vector<Pose3D>>();

    for (const roadElement &element: m_odReader->RoadList) {

        vector<Pose3D> leftLanePoses = m_odReader->GetRoadPoints(element, numLineSegments, DRIVING_POINTS, LEFT_LANE);
        reverse(leftLanePoses.begin(), leftLanePoses.end());
        for (Pose3D &pose : leftLanePoses) {
            Euler euler = openDriveReader::toEulerianAngle(pose.q);
            euler.yaw += euler.yaw > 0 ? -M_PI : M_PI;

            pose.q = openDriveReader::toQuaternion(euler.pitch, euler.roll, euler.yaw);
        }

        vector<Pose3D> rightLanePoses = m_odReader->GetRoadPoints(element, numLineSegments, DRIVING_POINTS, RIGHT_LANE);

        if (element.junction != -1) {
            classifyAndAddToJunctionEntryIfNecessary(element, leftLanePoses);
            classifyAndAddToJunctionEntryIfNecessary(element, rightLanePoses);
        } else {
            casualLanes.push_back(leftLanePoses);
            casualLanes.push_back(rightLanePoses);
        }
    }

    //check if road sign map is initialized and fuse signs into junction entries
    fuseRoadSignsIntoJunctionEntries();

    RETURN_NOERROR;
}

tResult cOpenDriveMapAnalyzer::analyzeRoadSignXML(const char *xml) {

    adtf::util::cDOM oDOM;
    oDOM.FromString(xml);

    remainingRoadSigns.clear();

    cDOMElementRefList oElems;

//    ------ Add signs to intersections ------

    if (IS_FAILED(oDOM.FindNodes("configuration/roadSign", oElems))) {
        RETURN_ERROR(ERR_INVALID_FILE);
    }

    for (cDOMElementRefList::iterator itElem = oElems.begin(); itElem != oElems.end(); ++itElem) {
        tRoadSign item;
        item.type = RoadSignType((*itElem)->GetAttributeUInt32("id", 0));
        item.f32X = tFloat32((*itElem)->GetAttributeFloat64("x", 0));
        item.f32Y = tFloat32((*itElem)->GetAttributeFloat64("y", 0));
        item.f32Radius = tFloat32((*itElem)->GetAttributeFloat64("radius", 0));
        item.f32Direction = tFloat32((*itElem)->GetAttributeFloat64("direction", 0) * M_PI / 180);
        item.bInit = ((*itElem)->GetAttributeUInt32("init", 0)) != 0;

        //CONSOLE_LOG_INFO_LOG_INFO((cString::Format("LoadConfiguration::Id %d XY %f %f Radius %f Direction %f",
        //    item.u16Id, item.f32X, item.f32Y, item.f32Radius, item.f32Direction).GetPtr()));

        remainingRoadSigns.push_back(item);

    }

//    ------ Collect parking spaces ------

    if (IS_FAILED(oDOM.FindNodes("configuration/parkingSpace", oElems))) {
        RETURN_ERROR(ERR_INVALID_FILE);
    }

    for (cDOMElementRefList::iterator itElem = oElems.begin(); itElem != oElems.end(); ++itElem) {
        tRoadSign item;
        item.f32Radius = RoadSignType((*itElem)->GetAttributeFloat64("id", 0));
        item.f32X = tFloat32((*itElem)->GetAttributeFloat64("x", 0));
        item.f32Y = tFloat32((*itElem)->GetAttributeFloat64("y", 0));
        item.f32Direction = tFloat32((*itElem)->GetAttributeFloat64("direction", 0));

        parkingSpaces.insert(pair<int, tRoadSign>(int(item.f32Radius), item));
    }

    CONSOLE_LOG_INFO("Found %u parkings paces!", parkingSpaces.size());

    fuseRoadSignsIntoJunctionEntries();

    RETURN_NOERROR;
}

void cOpenDriveMapAnalyzer::fuseRoadSignsIntoJunctionEntries() {
    // Don't execute if not all data is loaded yet
    if (junctionEntries.empty() || remainingRoadSigns.empty()) return;

    for (auto &junctionEntry : junctionEntries) {
        int bestFittingSign = -1;
        tFloat32 lowestDistance = MAX_ROAD_SIGN_MATCH_RADIUS;

        for (tSize i = 0; i < remainingRoadSigns.size(); ++i) {
            auto &roadSign = remainingRoadSigns.at(i);

            Point p = { roadSign.f32X, roadSign.f32Y, 0.0f };
            tFloat32 distance = calcDistance(p, junctionEntry.entryPoint.p);

            if (distance < lowestDistance) {
                bestFittingSign = int(i);
                lowestDistance = distance;
            }
        }

        if (bestFittingSign != -1) {
            auto &roadSign = remainingRoadSigns.at(bestFittingSign);
            // We found a road sign for a junction entry
            junctionEntry.roadSign = roadSign.type;
            remainingRoadSigns.erase(remainingRoadSigns.begin() + bestFittingSign);
        } else {
            LOG_INFO("Did not find a road sign in radius for junction entry at (%.2f, %.2f) (ID: %d)",
                     junctionEntry.entryPoint.p.x, junctionEntry.entryPoint.p.y, junctionEntry.junctionID);
        }
    }

    LOG_INFO("Number of remaining road signs after fusing into junction entries: %u", remainingRoadSigns.size());
}

void cOpenDriveMapAnalyzer::classifyAndAddToJunctionEntryIfNecessary(const roadElement &element, vector<Pose3D> lanePoses) {
    if (lanePoses.empty() || element.junction == -1) { return; }

    Pose3D entryPoint = lanePoses.front();
    LaneTurnClassification classification = classifyJunctionLane(entryPoint, lanePoses.back());

    tInt index = -1;
    for (tUInt i = 0; i < junctionEntries.size(); i++) {
        tFloat32 distance = calcDistance(entryPoint.p, junctionEntries[i].entryPoint.p);
        if (distance < MAX_ENTRYPOINT_MERGE_RADIUS) {
            CONSOLE_LOG_INFO("Merging into existing entryPoint. %f", distance);
            index = i; break;
        }
    }

    if (index == -1) { // if we found none, create one
        index = tInt(junctionEntries.size());
        tJunctionEntry junctionEntry;
        junctionEntry.id = index;
        junctionEntry.junctionID = element.junction;
        junctionEntry.entryPoint = entryPoint;
        junctionEntry.roadSign = RoadSignType::RECHTS_VOR_LINKS;
        junctionEntries.push_back(junctionEntry);
    }
    switch (classification) {
        case JUNCTION_LEFT_TURN:
            junctionEntries[index].leftTurn = lanePoses; break;
        case JUNCTION_RIGHT_TURN:
            junctionEntries[index].rightTurn = lanePoses; break;
        case JUNCTION_STRAIGHT:
            junctionEntries[index].straight = lanePoses; break;
        default: break;
    }
}


tFloat32 cOpenDriveMapAnalyzer::normalizeAngleDegree(tFloat32 angle) {
    return fmod(angle + 360.0f, 360.0f);
}

tFloat32 cOpenDriveMapAnalyzer::normalizeAngle(tFloat32 angle) {
    return tFloat32(fmod(angle + 2.0f * M_PI, 2.0f * M_PI));
}

LaneTurnClassification cOpenDriveMapAnalyzer::classifyJunctionLane(Pose3D entryPoint, Pose3D exitPoint) {
    tFloat32 entryHeading = normalizeAngleDegree(tFloat32(openDriveReader::toEulerianAngle(entryPoint.q).yaw * 180 / M_PI));
    tFloat32 exitHeading = normalizeAngleDegree(tFloat32(openDriveReader::toEulerianAngle(exitPoint.q).yaw * 180 / M_PI));
    tFloat32 relativeAngle = entryHeading - exitHeading;

    if (relativeAngle > STRAIGHT_EPSILON) {
        // turn right
        return JUNCTION_RIGHT_TURN;
    } else if (relativeAngle < -STRAIGHT_EPSILON) {
        // turn left
        return JUNCTION_LEFT_TURN;
    }

    return JUNCTION_STRAIGHT;
}

tFloat32 cOpenDriveMapAnalyzer::calcDistance(Point a, Point b) {
    return tFloat32(sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2)));
}

tJunctionEntry* cOpenDriveMapAnalyzer::inRangeOfJunctionEntry(tVehiclePosition position) {
    Point pos = {position.f32x, position.f32y, 0};

    CONSOLE_LOG_INFO("------ Scanning all %u Intersections: I am at ( %f | %f ) with heading %f.", junctionEntries.size(), pos.x, pos.y, position.f32heading * 180.0f / M_PI);


    position.f32heading = normalizeAngle(position.f32heading);

    for (unsigned int i = 0; i < junctionEntries.size(); i++) {

        tVehiclePosition junctionEntry = convertPoseToVehiclePosition(junctionEntries.at(i).entryPoint);
        junctionEntry.f32heading = normalizeAngle(junctionEntry.f32heading);

        tFloat32 distance = calcDistance(pos, {junctionEntry.f32x, junctionEntry.f32y, 0.0f});
        tFloat32 headingDiff = normalizeAngle(abs(junctionEntry.f32heading - position.f32heading));


        // TODO: Do the equality check based on a square of side length = 2 * RADIUS_OF_EQUALITY?
        if (distance < RADIUS_OF_EQUALITY) {
            if(abs(headingDiff) < HEADING_DIFF_OF_EQUALITY || abs(headingDiff - 2.0f*M_PI) < HEADING_DIFF_OF_EQUALITY) {

                LOG_INFO("In Range of JunctionEntry at (\t%f\t|\t%f\t) with heading (junction:\t%f\t| my:\t%f\t)... \t\tDistance: %f \t| HeadingDiff: %f",
                         junctionEntry.f32x, junctionEntry.f32y, junctionEntry.f32heading * 180.0f / M_PI, position.f32heading * 180.0f / M_PI, distance, headingDiff * 180.0f / M_PI);
                return &(junctionEntries.at(i));
            } else {
                CONSOLE_LOG_INFO("Discarding JunctionEntry at (\t%f\t|\t%f\t) due to heading (junction:\t%f\t| my:\t%f\t)... \t\tDistance: %f \t| HeadingDiff: %f",
                                 junctionEntry.f32x, junctionEntry.f32y, junctionEntry.f32heading * 180.0f / M_PI, position.f32heading * 180.0f / M_PI, distance, headingDiff * 180.0f / M_PI);

                //junctionEntry.f32heading * 180.0f / M_PI
            }
        } else {
            CONSOLE_LOG_INFO("Discarding JunctionEntry at (\t%f\t|\t%f\t) due to distance\t\tDistance: %f \t| Heading: (junction:\t%f\t| my:\t%f\t).",
                     junctionEntry.f32x, junctionEntry.f32y, distance, junctionEntry.f32heading * 180.0f / M_PI, position.f32heading * 180.0f / M_PI);
        }

    }

    return nullptr;
}

tVehiclePosition cOpenDriveMapAnalyzer::convertPoseToVehiclePosition(Pose3D pose) {
    tVehiclePosition pos;

    pos.f32x = pose.p.x;
    pos.f32y = pose.p.y;
    pos.f32heading = openDriveReader::toEulerianAngle(pose.q).yaw;

    return pos;
}



void cOpenDriveMapAnalyzer::calculateSize() {
    m_minX = 99999999.0f, m_maxX = -99999999.0f;
    m_minY = 99999999.0f, m_maxY = -99999999.0f;
    m_minZ = 99999999.0f, m_maxZ = -99999999.0f;
    tUInt8 num = 10;//Number of points
    vector<Pose3D> borderPoints = m_odReader->GetRoadPoints(m_odReader->RoadList, num, BORDER_POINTS);
    for (tUInt i = 0; i < borderPoints.size(); i++) {
        float x = borderPoints[i].p.x;
        float y = borderPoints[i].p.y;
        float z = borderPoints[i].p.z;
        if (x < m_minX) {
            m_minX = x;
        } else if (x > m_maxX) {
            m_maxX = x;
        }
        if (y < m_minY) {
            m_minY = y;
        } else if (y > m_maxY) {
            m_maxY = y;
        }
        if (z < m_minZ) {
            m_minZ = z;
        } else if (z > m_maxZ) {
            m_maxZ = z;
        }
    }
}

tBool cOpenDriveMapAnalyzer::isInitialized() {
    return singleton != nullptr && !singleton->junctionEntries.empty();
}
