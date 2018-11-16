#include <utility>

#include "EgoState.h"
#include "../../Helpers/cariosity_structs.h"

using namespace fhw;

cEgoState *cEgoState::singleton = nullptr;

cEgoState::cEgoState() {
    LOG_WARNING("New EgoState");
}


tBool cEgoState::isInitialized() {
    return singleton != nullptr;
}



void cEgoState::setLaserScannerData(vector<tPolarCoordiante> scannerData) {
    this->laserScannerData = std::move(scannerData);
}

vector<tPolarCoordiante> cEgoState::getLaserScannerData() {
    return this->laserScannerData;
}

void cEgoState::setUltrasonicStruct(tUltrasonicStruct ultrasonicStruct) {
    this->ultrasonicStruct = ultrasonicStruct;
}

void cEgoState::setUSSideRight(tSignalValue USSideRight){
    this->ultrasonicStruct.tSideRight = USSideRight;
}

void cEgoState::setUSRearRight(tSignalValue USRearRight){
    this->ultrasonicStruct.tRearRight = USRearRight;
}

void cEgoState::setUSRearCenter(tSignalValue USRearCenter){
    this->ultrasonicStruct.tRearCenter = USRearCenter;
}

void cEgoState::setUSRearLeft(tSignalValue USRearLeft){
    this->ultrasonicStruct.tRearLeft = USRearLeft;
}

void cEgoState::setUSSideLeft(tSignalValue USSideLeft){
    this->ultrasonicStruct.tSideLeft = USSideLeft;
}


tUltrasonicStruct cEgoState::getUltrasonicStruct() {
    return this->ultrasonicStruct;
}

void cEgoState::setInerMeasUnitData(tInerMeasUnitData inerMeasUnitData) {
    this->inerMeasUnitData = inerMeasUnitData;
}

tInerMeasUnitData cEgoState::getInerMeasUnitData() {
    return this->inerMeasUnitData;
}

tFloat32 cEgoState::getRelativeLanePositioning() {
    return this->relativeLanePositioning;
}

void cEgoState::setRelativeLanePositioning(tFloat32 lrelativeLanePositioning) {
    this->relativeLanePositioning = lrelativeLanePositioning;
}

tFloat32 cEgoState::getTotalDistance() {
    return totalDistance;
}

void cEgoState::setTotalDistance(tFloat32 totalDistance) {
    this->totalDistance = totalDistance;
}

tVehiclePosition cEgoState::getVehiclePosition() {
    return position;
}

void cEgoState::setVehiclePosition(tVehiclePosition position) {
    this->position = position;
}

tTimeStamp cEgoState::getLastUpdated() {
    return lastUpdateTime;
}

void cEgoState::setLastUpdated(tTimeStamp lastUpdateTime) {
    this->lastUpdateTime = lastUpdateTime;
}

vector<tRecognizedObject> cEgoState::getPersons(){
    return persons;
}
void cEgoState::setPersons(vector<tRecognizedObject> personArray){
    this->persons = std::move(personArray);
}

vector<tRecognizedObject> cEgoState::getChilds(){
    return childs;
}
void cEgoState::setChilds(vector<tRecognizedObject> childsArray){
    this->childs = std::move(childsArray);
}

vector<tRecognizedObject> cEgoState::getCars(){
    return cars;
}
void cEgoState::setCars(vector<tRecognizedObject> carArray){
    this->cars = std::move(carArray);
}

tBool cEgoState::isEmergencyCarPresent(){
    return emergencyCarPresent;
}
void cEgoState::setEmergencyCarPresent(tBool isPresent){
    this->emergencyCarPresent = tFalse;
}

map<int, tRoadSign> cEgoState::getParkingSpaces(){
    return parkingSpaces;
}

void cEgoState::setParkingSpaces(map<int, tRoadSign> parkingSpaces) {
    this->parkingSpaces = std::move(parkingSpaces);
}

tUInt32 cEgoState::getLaneDetectionConfidence() {
    return laneDetectionConfidence;
}

void cEgoState::setLaneDetectionConfidence(tUInt32 confidence) {
    this->laneDetectionConfidence = confidence;
}

tLanes cEgoState::getLanes() {
    return lanes;
}

void cEgoState::setLanes(tLanes newLanesData) {
    lanes = newLanesData;
}

fhw::environmentSituation cEgoState::getEnvironmentSituation(){
    return m_environmentSituation;
}

void cEgoState::setEnvironmentSituation(fhw::environmentSituation newState){
    this->m_environmentSituation = newState;
}

tBool cEgoState::getOverrideLaneDetectionAngleOfFielOfView() const {
    return overrideLaneDetectionAngleOfFielOfView;
}

void cEgoState::setOverrideLaneDetectionAngleOfFielOfView(tBool overrideLaneDetectionAngleOfFielOfView) {
    this->overrideLaneDetectionAngleOfFielOfView = overrideLaneDetectionAngleOfFielOfView;
}

tFloat32 cEgoState::getLaneDetectionAngleOfFieldOfView() const {
    return LaneDetectionAngleOfFieldOfView;
}

void cEgoState::setLaneDetectionAngleOfFieldOfView(tFloat32 LaneDetectionAngleOfFieldOfView) {
    this->LaneDetectionAngleOfFieldOfView = LaneDetectionAngleOfFieldOfView;
}

tBool cEgoState::getYOLOState() const {
    return YOLOState;
}

void cEgoState::setYOLOState(tBool YOLOState) {
    this->YOLOState = YOLOState;
}
