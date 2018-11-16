//
// Created by aadc on 18.09.18.
//

#include "Entity.h"

#define TO_MILLIS(time)(tTimeStamp(time / 1e3))
#define GET_MILLIS (TO_MILLIS(adtf_util::cHighResTimer::GetTime()))

int cEntity::entityID = 0;

cEntity::cEntity() {
    entityType = EMPTY;
    confidence = 0;
    timeLastUpdated = GET_MILLIS;
    lastSeen = timeLastUpdated;
//    LOG_WARNING("new cEntity %d", entityID++);
}

eEntityType cEntity::getEntityType() {
//    LOG_INFO("getting before: %u", confidence);
    //calculateConfidenceDecay();
//    LOG_INFO("getting after: %u", confidence);
    return entityType;
}

void cEntity::setEntityType(eEntityType entityType) {
//    LOG_INFO("setting before: %u", confidence);

    calculateConfidenceDecay();
    confidence += 2;
    lastSeen = GET_MILLIS;
    this->entityType = entityType;

//    LOG_INFO("setting after: %u", confidence);
}

tUInt16 cEntity::getConfidence() {
    calculateConfidenceDecay();
    tTimeStamp currentTime = GET_MILLIS;
    tTimeStamp deltaT = min(currentTime - lastSeen, tTimeStamp(numeric_limits<tUInt16>::max()));
    if (deltaT > 65536) {
        LOG_ERROR("deltaT underflowing: %u", deltaT);
    }
    return tUInt16(min(confidence * 1e3 / deltaT, 255.0));
}

void cEntity::calculateConfidenceDecay() {
    static mutex m;
    lock_guard<mutex> lock_guard(m);

    tTimeStamp currentTime = GET_MILLIS;
    tUInt16 deltaT = tUInt16(min(currentTime - timeLastUpdated, tTimeStamp(numeric_limits<tUInt16>::max())));

//    LOG_ERROR("before: lastUpdated: %u || DeltaT: %u || conf: %u", timeLastUpdated, deltaT, confidence);
    // the more confident we are, the less we want to decrease it
    tUInt16 decrease = tUInt16(deltaT / max(confidence, tUInt16(1)));
    if (decrease >= confidence) {
        confidence = 0;
        entityType = EMPTY;
    } else {
        confidence -= decrease;
    }

    if (decrease != 0) {
        // if confidence grows to large we wont ever release this object
        // because deltaT is always kind of the same
        // --> therefore we only update it if decrease is not zero
        timeLastUpdated = currentTime;
    }
//    LOG_ERROR("after: lastUpdated: %u || conf: %u", timeLastUpdated, confidence);
}