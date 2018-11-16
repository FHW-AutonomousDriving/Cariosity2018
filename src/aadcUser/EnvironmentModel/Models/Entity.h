//
// Created by aadc on 18.09.18.
//

#pragma once

#include <limits>
#include "../stdafx.h"

using namespace adtf_util;
using namespace std;

enum eEntityType : tUInt8 {
    EMPTY = 0,

    CENTER_LINE = 31,
    SOLID_LINE = 63,
    DASHED_LINE = 95,
    STOP_LINE = 127,
    PARK_MARKING = 159,

    OBSTACLE = 255,
    PERSON = 223
};

class cEntity {

private:
    static int entityID;

    eEntityType entityType;
    tUInt16 confidence;
    tTimeStamp timeLastUpdated; // to calculate decreasing confidence
    tTimeStamp lastSeen;
public:

    cEntity();

    eEntityType getEntityType();

    void setEntityType(eEntityType entityType);

    tUInt16 getConfidence();

    void calculateConfidenceDecay();

};
