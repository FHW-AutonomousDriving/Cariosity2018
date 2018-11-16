#include "stdafx.h"
#include "MissionController.h"
#include "../Helpers/cariosity_structs.h"

//*************************************************************************************************

ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_CARIOSITY_DATA_TRIGGERED_FILTER,
                                    "MissionController",
                                    fcMissionController,
                                    adtf::filter::pin_trigger({"in_jurystruct",
                                                               "in_maneuver_list",
                                                               "in_emergency_brake",
                                                               "in_ego_state"}));

#define CONSOLE_LOG(_text, _log_level) if (m_propEnableConsoleOutput) { LOG_ADD_ENTRY(_log_level, _text); }    //!< enables log function if console output is activated
#define CONSOLE_LOG_INFO(...) if (m_propEnableConsoleOutput) LOG_ADD_ENTRY(A_UTILS_NS::log::tLogLevel::Info, __VA_ARGS__)


#define goalController cGoalController::getInstance()
#define currentTask goalController.getCurrentTask()

//*************************************************************************************************

fcMissionController::fcMissionController() : cStdFilter()
{
    // Register properties
    RegisterPropertyVariable("Enable console log", m_propEnableConsoleOutput);
    RegisterPropertyVariable("Minimum time before startup [sec]", m_propMinimumStartupTime);

    // general pintype for large unspecific data: maneuverList
    object_ptr<IStreamType> pTypeManeuver = make_object_ptr<cStreamType>(stream_meta_type_anonymous());
    object_ptr<IStreamType> pTypeOpenDrive = make_object_ptr<cStreamType>(stream_meta_type_anonymous());
    // Register pins
    // ------ in
    Register(m_oInputJuryStruct, "in_jurystruct" , getStreamType(JURY_INSTRUCTION));
    Register(m_oInputManeuverList, "in_maneuver_list", pTypeManeuver);

    Register(m_oInputEmergencyBrake, "in_emergency_brake" , getStreamType(SIGNAL_VALUE)); //TODO: BOOL_SIGNAL_VALUE
    Register(m_oInputEgoState, "in_ego_state" , getStreamType(POINTER_VALUE));

    // ------ out
    Register(m_oOutputDriverState, "out_driverstate", getStreamType(DRIVER_STATE));

    Register(m_oOutputEmergencyBrake, "out_emergency_brake", getStreamType(SIGNAL_VALUE)); //TODO: BOOL_SIGNAL_VALUE
    Register(m_oOutputSpeed, "out_speed", getStreamType(SIGNAL_VALUE));
    Register(m_oOutputSteeringAngle, "out_steering_angle", getStreamType(SIGNAL_VALUE));

    // lights
    Register(m_oOutputLight_Head, "out_light_head", getStreamType(BOOL_SIGNAL_VALUE));
    Register(m_oOutputLight_TurnLeft, "out_light_turnleft", getStreamType(BOOL_SIGNAL_VALUE));
    Register(m_oOutputLight_TurnRight, "out_light_turnright", getStreamType(BOOL_SIGNAL_VALUE));
    Register(m_oOutputLight_Brake, "out_light_brake", getStreamType(BOOL_SIGNAL_VALUE));
    Register(m_oOutputLight_Hazard, "out_light_hazard", getStreamType(BOOL_SIGNAL_VALUE));
    Register(m_oOutputLight_Reverse, "out_light_reverse", getStreamType(BOOL_SIGNAL_VALUE));

    Register(m_oReaderOpenDriveMapAnalyzerInstanceSynchronization, "OpenDriveMapAnalyzerInstanceSynchronization", getStreamType(POINTER_VALUE));


    // default jury instruction
    m_currentJuryInstruction.i16ActionID = aadc::jury::action_stop;
    m_currentJuryInstruction.i16ManeuverEntry = 0;
    m_currentCarState = STARTUP;
}

tResult fcMissionController::Configure()
{
    //RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    RETURN_NOERROR;
}

tResult fcMissionController::Process(tTimeStamp tmTimeOfTrigger)  {


    // handle jury instruction, if new; if stop: stop!
    // as if task is finished, if so: finish task (gc) + fetch new task
    // execute task, hand taskResult to actuators

    CONSOLE_LOG_INFO(adtf_util::cString::Format("MissionController on data triggered ..."));

    receiveOpenDriveMapAnalyzerInstance();
    receiveEgoStateInstance();

    //static mutex m;
    //lock_guard<std::mutex> lock(m);

    // check for new maneuver list
    receiveManeuverList();
    //CONSOLE_LOG_INFO(adtf_util::cString::Format("MissionController on data triggered ..."));

    static mutex m;
    lock_guard<std::mutex> lock(m);

    // check for new emergencyBrake data at pin
    tSignalValue pEmergencyBrakeSample;
    if( IS_OK(readSignalValue(m_oInputEmergencyBrake, pEmergencyBrakeSample)) ) {
        m_emergencyBrakeSignal = pEmergencyBrakeSample;
        //CONSOLE_LOG_INFO(cString::Format("Received new emergency brake sample %f", m_emergencyBrakeSignal));
    }


    // check for received jury data
    receiveManeuverList();
    receiveJuryInstruction();


    switch(m_currentCarState) {
        case STARTUP: {
            tTaskResult taskResult = {tTrue, 0.0f, 0.0f, {tFalse, tFalse, tFalse, tFalse, tTrue, tFalse}, fhw::nothing};
            transmitTaskResult(taskResult, tmTimeOfTrigger);
            if (m_startTime == 0) {
                m_startTime = tmTimeOfTrigger;
                m_currentDriverState.i16ManeuverEntry = m_currentJuryInstruction.i16ManeuverEntry;
                m_currentDriverState.i16StateID = statecar_startup;
                sendDriverStateToJury(m_currentDriverState);

            }
            if (!m_timePassed && m_startTime + tTimeStamp(m_propMinimumStartupTime * 1e6) < tmTimeOfTrigger) {
                m_timePassed = tTrue;
            }
            LOG_INFO("Recieved Maneuevers: %s \t Recieved Map: %s \t Has Heading: %s \t Time has passed: %s", m_recievedManeuver ? "OK" : "⏱"
            , m_recievedMap ? "OK" : "⏱"
            , m_hasHeading ? "OK" : "⏱"
            , m_timePassed ? "OK" : "⏱" );
            if (m_recievedManeuver && m_recievedMap && m_hasHeading && m_timePassed) {
                m_currentCarState = READY;
                m_timePassed = tFalse;
                m_startTime = 0;
                m_currentDriverState.i16ManeuverEntry = m_currentJuryInstruction.i16ManeuverEntry;
                m_currentDriverState.i16StateID = statecar_ready;
                sendDriverStateToJury(m_currentDriverState);
            }
            break;
        }
        case READY: {
            tTaskResult taskResult = {tTrue, 0.0f, 0.0f, {tFalse, tFalse, tFalse, tFalse, tFalse, tFalse}, fhw::nothing};
            transmitTaskResult(taskResult, tmTimeOfTrigger);
            if (juryAction(m_currentJuryInstruction.i16ActionID) == action_start) {
                m_currentCarState = RUNNING;
                m_currentDriverState.i16ManeuverEntry = m_currentJuryInstruction.i16ManeuverEntry;
                m_currentDriverState.i16StateID = statecar_running;
                sendDriverStateToJury(m_currentDriverState);
                goalController.restartFromSection(m_currentJuryInstruction.i16ManeuverEntry);
            }
            break;
        }
        case RUNNING: {
            if (juryAction(m_currentJuryInstruction.i16ActionID) == action_stop) {
                m_currentCarState = STOPPED;
                break;
            }

            if (goalController.isDone()) {
                LOG_INFO("I'm done :-)");
                m_currentCarState = FINISHED;
                break; //o/o/ dance
            }

            tResult taskResult = handleTask();
            if (!IS_OK(taskResult)) {
                m_currentCarState = STOPPED;
                m_currentDriverState.i16ManeuverEntry = m_currentJuryInstruction.i16ManeuverEntry;
                m_currentDriverState.i16StateID = statecar_error;
                sendDriverStateToJury(m_currentDriverState);
                return taskResult;
            }
            transmitTaskResult(m_currentTaskResult, tmTimeOfTrigger);
            break;
        }
        case STOPPED: {
            tTaskResult taskResult = {tTrue, 0.0f, 0.0f, {tFalse, tFalse, tFalse, tFalse, tFalse, tFalse}, fhw::nothing};
            transmitTaskResult(taskResult, tmTimeOfTrigger);
            if (m_startTime == 0) {
                m_startTime = tmTimeOfTrigger;
                m_currentDriverState.i16ManeuverEntry = m_currentJuryInstruction.i16ManeuverEntry;
                m_currentDriverState.i16StateID = statecar_startup;
                sendDriverStateToJury(m_currentDriverState);
            }

            if (m_startTime + tTimeStamp(m_propMinimumStartupTime * 1e6) < tmTimeOfTrigger) {
                m_timePassed = tTrue;
            }

            if (m_timePassed) {
                m_currentCarState = READY;
                m_timePassed = tFalse;
                m_startTime = 0;
                m_currentDriverState.i16ManeuverEntry = m_currentJuryInstruction.i16ManeuverEntry;
                m_currentDriverState.i16StateID = statecar_ready;
                sendDriverStateToJury(m_currentDriverState);
            }
            break;
        }
        case FINISHED: {
            m_currentDriverState.i16ManeuverEntry = m_currentJuryInstruction.i16ManeuverEntry;
            m_currentDriverState.i16StateID = statecar_complete;
            sendDriverStateToJury(m_currentDriverState);
            if (m_timePassed) {
                m_currentCarState = READY;
                m_timePassed = tFalse;
                m_startTime = 0;
                m_currentDriverState.i16ManeuverEntry = m_currentJuryInstruction.i16ManeuverEntry;
                m_currentDriverState.i16StateID = statecar_ready;
                sendDriverStateToJury(m_currentDriverState);
            }
            break;
        }
    }

    RETURN_NOERROR;
}

tResult fcMissionController::handleTask() {
    //CONSOLE_LOG_INFO(adtf_util::cString::Format("MissionController: handleTask ..."));

    static environmentSituation  lastSituation;
    tBool situationHasChanged = cEgoState::singleton->getEnvironmentSituation() != lastSituation;

    if(m_currentCarState != eCarState::READY
       && m_currentCarState != eCarState::RUNNING) {
        if(cEgoState::singleton->getEnvironmentSituation() == obstacle_ahead) {
            cEgoState::singleton->setEnvironmentSituation(lanekeeping);
        }
    }

    // check if current task is finished, then get new task
    // TODO: Only let next Task take over control if the current one is interruptable
    if(currentTask.getTaskState() == fhw::taskState::finished
       || (currentTask.isInterruptable()
           && (goalController.isNextTaskReady() || situationHasChanged))) {
        goalController.completeCurrentTask();
        cEgoState::singleton->setYOLOState(true); // reactivate yolo if task is done
        cEgoState::singleton->setOverrideLaneDetectionAngleOfFielOfView(false);
        cEgoState::singleton->setLaneDetectionAngleOfFieldOfView(0);

        if(situationHasChanged && !goalController.isNextTaskReady()) {
            switch(cEgoState::singleton->getEnvironmentSituation()) {
                case environmentSituation::zebra_crossing :
                    goalController.addImmediateManeuver(fhw::extraManeuvers::crosswalk_handling);
                    break;
                case environmentSituation::lanekeeping :
                    goalController.addImmediateManeuver(fhw::extraManeuvers::nothing);
                    break;
                case environmentSituation::obstacle_ahead :
                    goalController.addImmediateManeuver(fhw::extraManeuvers::overtaking);
                    break;
                case environmentSituation::emergency_car :
                    goalController.addImmediateManeuver(emergency_vehicle);
                    break;
                default:
                    LOG_ERROR("Could not find appropriate maneuver");
                    break;
            }
            lastSituation = cEgoState::singleton->getEnvironmentSituation();
        }
        // if newly fetched task is first of maneuver
        if(currentTask.isFirstTaskOfManeuver()) {
            // send driverState to Jury
            m_currentDriverState.i16ManeuverEntry = goalController.getCurrentManeuverNumber();
            m_currentDriverState.i16StateID = stateCar::statecar_running;

            sendDriverStateToJury(m_currentDriverState);
        }
    }


    tTaskResult receivedTaskResult = {tFalse, 0.0f, 0.0f, {tFalse, tFalse, tFalse, tFalse, tFalse, tFalse}, fhw::nothing};
    RETURN_IF_FAILED(currentTask.execute(m_emergencyBrakeSignal.f32Value != 0.0f, receivedTaskResult));

    m_currentTaskResult = receivedTaskResult;
    RETURN_NOERROR;
}


tResult fcMissionController::transmitTaskResult(tTaskResult &taskResult, tTimeStamp timeOfTrigger) {

    //TODO check for new Tasks: tTaskResult.immediateManeuver

    tSignalValue emergencyBrake = {tUInt32(timeOfTrigger), (taskResult.bEmergencyBrake ? 1.0f : 0.0f)};
    RETURN_IF_FAILED(transmitSignalValue(emergencyBrake, m_oOutputEmergencyBrake));
    tSignalValue speed = {tUInt32(timeOfTrigger), taskResult.f32speed};
    RETURN_IF_FAILED(transmitSignalValue(speed, m_oOutputSpeed));
    tSignalValue steering = {tUInt32(timeOfTrigger), taskResult.f32steeringAngle};
    RETURN_IF_FAILED(transmitSignalValue(steering, m_oOutputSteeringAngle));

    if (m_bLight_Head != taskResult.lights.bHeadLight) {
        m_bLight_Head = taskResult.lights.bHeadLight;
        tBoolSignalValue HeadLight = {tUInt32(timeOfTrigger), m_bLight_Head};
        RETURN_IF_FAILED(transmitBoolSignalValue(HeadLight, m_oOutputLight_Head));
    }
    if (m_bLight_TurnLeft != taskResult.lights.bTurnSignalLeft) {
        m_bLight_TurnLeft = taskResult.lights.bTurnSignalLeft;
        tBoolSignalValue TurnSignalLeft = {tUInt32(timeOfTrigger), m_bLight_TurnLeft};
        RETURN_IF_FAILED(transmitBoolSignalValue(TurnSignalLeft, m_oOutputLight_TurnLeft));
    }
    if (m_bLight_TurnRight != taskResult.lights.bTurnSignalRight) {
        m_bLight_TurnRight = taskResult.lights.bTurnSignalRight;
        tBoolSignalValue TurnSignalRight = {tUInt32(timeOfTrigger), m_bLight_TurnRight};
        RETURN_IF_FAILED(transmitBoolSignalValue(TurnSignalRight, m_oOutputLight_TurnRight));
    }
    if (m_bLight_Brake != taskResult.lights.bBrakeLight) {
        m_bLight_Brake = taskResult.lights.bBrakeLight;
        tBoolSignalValue BrakeLight = {tUInt32(timeOfTrigger), m_bLight_Brake};
        RETURN_IF_FAILED(transmitBoolSignalValue(BrakeLight, m_oOutputLight_Brake));
    }
    if (m_bLight_Hazard != taskResult.lights.bHazardLight) {
        m_bLight_Hazard = taskResult.lights.bHazardLight;
        tBoolSignalValue HazardLight = {tUInt32(timeOfTrigger), m_bLight_Hazard};
        RETURN_IF_FAILED(transmitBoolSignalValue(HazardLight, m_oOutputLight_Hazard));
    }
    if (m_bLight_Reverse != taskResult.lights.bReverseLight) {
        m_bLight_Reverse = taskResult.lights.bReverseLight;
        tBoolSignalValue ReverseLight = {tUInt32(timeOfTrigger), m_bLight_Reverse};
        RETURN_IF_FAILED(transmitBoolSignalValue(ReverseLight, m_oOutputLight_Reverse));
    }

    RETURN_NOERROR;
}


// ------------ jury communication ------------

tResult fcMissionController::receiveJuryInstruction() {
    // check for new jury instruction data at pin
    tJuryStruct pJuryInstructionSample;
    if( IS_OK(readJuryInstructionData(m_oInputJuryStruct, pJuryInstructionSample)) ) {
        m_currentJuryInstruction = pJuryInstructionSample;
        CONSOLE_LOG_INFO(cString::Format("Received new jury instruction %s",
                                         fhw::juryActionToString(juryAction(m_currentJuryInstruction.i16ActionID))));
    }

    RETURN_NOERROR;
}

tResult fcMissionController::receiveManeuverList() {
    // check for new maneuverList data on maneuver list pin

    object_ptr<const ISample> pManeuverListData;
    if (IS_OK(m_oInputManeuverList.GetNextSample(pManeuverListData)))
    {
        CONSOLE_LOG_INFO(cString::Format("Received data on maneuverList pin"));
        std::vector<tChar> data;
        object_ptr_shared_locked<const ISampleBuffer> pSampleBuffer;
        RETURN_IF_FAILED(pManeuverListData->Lock(pSampleBuffer));
        data.resize(pSampleBuffer->GetSize());
        memcpy(data.data(), pSampleBuffer->GetPtr(), pSampleBuffer->GetSize());
        if (data.size() > 0)
        { //maneuverlist
            CONSOLE_LOG_INFO(cString::Format("Received ManeuverList data: %s", data.data()));
            cString  strManeuverList;
            strManeuverList.Set(data.data(), data.size());
            // send to goalController
            goalController.setManeuverList(data.data());
            m_recievedManeuver = tTrue;
        }
    }

    RETURN_NOERROR;
}

tResult fcMissionController::sendDriverStateToJury(tDriverStruct &theDriverState) {
    RETURN_IF_FAILED( transmitDriverStateData(theDriverState, m_oOutputDriverState));
    RETURN_NOERROR;
}

tResult fcMissionController::receiveOpenDriveMapAnalyzerInstance() {

    tPointerValue value;
    RETURN_IF_FAILED(cStdFilter::readPointerData(m_oReaderOpenDriveMapAnalyzerInstanceSynchronization, value));

    cOpenDriveMapAnalyzer::singleton = reinterpret_cast<cOpenDriveMapAnalyzer*>(value.address);
    if (!cOpenDriveMapAnalyzer::isInitialized()) {
        LOG_ERROR("Map: ERR_NOT_INITIALIZED.");
        //TODO: return cError(ERR_NOT_INITIALIZED);
        RETURN_NOERROR;
    }

    LOG_INFO("Received %u junction entries and %u casual lanes.", cOpenDriveMapAnalyzer::singleton->junctionEntries.size(), cOpenDriveMapAnalyzer::singleton->casualLanes.size());
    m_recievedMap = tTrue;

    RETURN_NOERROR;
}

tResult fcMissionController::receiveEgoStateInstance() {

    tPointerValue value;
    RETURN_IF_FAILED(cStdFilter::readPointerData(m_oInputEgoState, value));

    cEgoState::singleton = reinterpret_cast<cEgoState*>(value.address);
    if (!cEgoState::isInitialized()) {
        LOG_ERROR("Map: ERR_NOT_INITIALIZED.");
        //TODO: return cError(ERR_NOT_INITIALIZED);
        RETURN_NOERROR;
    }

    if (cEgoState::singleton->getVehiclePosition().f32heading != 0.0f) {
        m_hasHeading = tTrue;
    }
    RETURN_NOERROR;
}