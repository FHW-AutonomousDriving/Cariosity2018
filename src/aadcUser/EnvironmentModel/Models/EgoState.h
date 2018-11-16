#pragma once

#include "../../Helpers/cariosity_structs.h"

using namespace std;

namespace fhw {
/**
 * Stores information about the relative, close environment and ego positioning,
 * such as steering angle, speed, state, relativeLanePositioning, set of situational descriptors (crossing, obsacle ahead, ramp etc.)
 */
class cEgoState {

    private:
        vector<tPolarCoordiante> laserScannerData;
        tUltrasonicStruct ultrasonicStruct;
        tInerMeasUnitData inerMeasUnitData;
        tVehiclePosition position;
        tFloat32 totalDistance;

        tFloat32 relativeLanePositioning;
        tUInt32 laneDetectionConfidence;
        tLanes lanes;

        tTimeStamp lastUpdateTime;
        vector<tRecognizedObject> persons;
        vector<tRecognizedObject> childs;
        vector<tRecognizedObject> cars;
        tBool emergencyCarPresent = tFalse;
        map<int, tRoadSign> parkingSpaces;

        tFloat32 LaneDetectionAngleOfFieldOfView = 0.0f;
        tBool overrideLaneDetectionAngleOfFielOfView = tFalse;

        tBool YOLOState = tTrue;


private:

    fhw::environmentSituation m_environmentSituation = fhw::environmentSituation::lanekeeping;

    public:
        static cEgoState *singleton;
        static tBool isInitialized();

    public:
        /*! Default constructor. */
        cEgoState();

        /*! Constructor for copy */
        cEgoState(const cEgoState &) = delete;


    // ------ getters + setters for data fields ------

        void setLaserScannerData(vector<tPolarCoordiante> scannerData);
        vector<tPolarCoordiante> getLaserScannerData();

        void setUltrasonicStruct(tUltrasonicStruct ultrasonicStruct);
        void setUSSideRight(tSignalValue USSideRight);
        void setUSRearRight(tSignalValue USRearRight);
        void setUSRearCenter(tSignalValue USRearCenter);
        void setUSRearLeft(tSignalValue USRearLeft);
        void setUSSideLeft(tSignalValue USSideLeft);
        tUltrasonicStruct getUltrasonicStruct();

        void setInerMeasUnitData(tInerMeasUnitData inerMeasUnitData);
        tInerMeasUnitData getInerMeasUnitData();

        tFloat32 getRelativeLanePositioning();
        void setRelativeLanePositioning(tFloat32 lrelativeLanePositioning);

        tUInt32 getLaneDetectionConfidence();
        void setLaneDetectionConfidence(tUInt32 confidence);

        tLanes getLanes();
        void setLanes(tLanes newLanesData);


        tFloat32 getTotalDistance();
        void setTotalDistance(tFloat32 totalDistance);

        tVehiclePosition getVehiclePosition();
        void setVehiclePosition(tVehiclePosition position);

        tTimeStamp getLastUpdated();
        void setLastUpdated(tTimeStamp lastUpdateTime);

        vector<tRecognizedObject> getPersons();
        void setPersons(vector<tRecognizedObject> personArray);

        vector<tRecognizedObject> getChilds();
        void setChilds(vector<tRecognizedObject> childsArray);

        vector<tRecognizedObject> getCars();
        void setCars(vector<tRecognizedObject> carArray);

        tBool isEmergencyCarPresent();

        void setEmergencyCarPresent(tBool isPresent);

        map<int, tRoadSign> getParkingSpaces();
        void setParkingSpaces(map<int, tRoadSign> parkingSpaces);

        fhw::environmentSituation getEnvironmentSituation();
        void setEnvironmentSituation(fhw::environmentSituation newState);

        tBool getOverrideLaneDetectionAngleOfFielOfView() const;

        void setOverrideLaneDetectionAngleOfFielOfView(tBool overrideLaneDetectionAngleOfFielOfView);

        tFloat32 getLaneDetectionAngleOfFieldOfView() const;

        void setLaneDetectionAngleOfFieldOfView(tFloat32 LaneDetectionAngleOfFieldOfView);

        tBool getYOLOState() const;

        void setYOLOState(tBool YOLOState);
    };

}

