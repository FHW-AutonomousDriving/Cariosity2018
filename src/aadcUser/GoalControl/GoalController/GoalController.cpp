#include "stdafx.h"
#include "GoalController.h"

using namespace aadc::jury;


// ------------ private  methods ------------

// ------ Constructor stuff ------

// private constructor
cGoalController::cGoalController() {}

cGoalController &cGoalController::getInstance() {
    static cGoalController instance;
    return instance;
}


// ------ Create task lists

tResult cGoalController::createTaskListFromSector(int sectorNumber) {
    LOG_INFO(cString::Format("Creating taskList from maneuverList for sector %d", sectorNumber));

    m_taskList.clear();

    tSector requestedSector = m_maneuverList.at(sectorNumber);
    for (auto const &currentManeuver: requestedSector.sector) {
        LOG_INFO(cString::Format("Maneuver #%d: %s", currentManeuver.id, aadc::jury::maneuverToString(aadc::jury::maneuver(currentManeuver.action)).c_str()));
        appendTaskListFromManeuver(currentManeuver, m_taskList);
    }

    // set maneuver and sector numbers
    m_iCurrentSectorNumber = sectorNumber;
    m_iCurrentManeuverNumber = m_maneuverList.at(sectorNumber).sector.at(0).id;

    RETURN_NOERROR;
}

tResult cGoalController::appendTaskListFromManeuver(fhw::extraManeuvers theManeuver, vector<cTask *> &theTaskList) {
    //TODO: implement appending required tasks to taskList
    tBool firstOfManeuver = tTrue;
    switch (theManeuver) {
        case fhw::extraManeuvers::nothing :
            theTaskList.insert(theTaskList.begin() + 0, new cTask_LaneFollowing);
            break;
        case fhw::extraManeuvers::crosswalk_handling :
            theTaskList.insert(theTaskList.begin() + 0, new cTask_ZebraCrossing);
            break;
        case fhw::extraManeuvers::emergency_vehicle :
            theTaskList.insert(theTaskList.begin() + 0, new cTask_EmergencyCar);
            break;
        case fhw::extraManeuvers::overtaking :
            theTaskList.insert(theTaskList.begin() + 0, new cTask_Overtaking);
            break;
    }
    RETURN_NOERROR;
}


tResult cGoalController::appendTaskListFromManeuver(aadc::jury::tManeuver theManeuver, vector<cTask *> &theTaskList) {
    //TODO: implement appending required tasks to taskList
    tBool firstOfManeuver = tTrue;
    LOG_INFO("Added action: %u", theManeuver.action);
    switch (theManeuver.action) {

        case maneuver::maneuver_right: case maneuver::maneuver_left: case maneuver::maneuver_straight:
            theTaskList.push_back(new cTask_LaneFollowing(firstOfManeuver));
            theTaskList.push_back(new cTask_IntersectionHandling(false, theManeuver.action));
            break;
        case maneuver::maneuver_merge_left:
            theTaskList.push_back(new cTask_LaneFollowing(firstOfManeuver));
            theTaskList.push_back(new cTask_LidarRamp(!firstOfManeuver));
            theTaskList.push_back(new cTask_LaneFollowing(!firstOfManeuver));
            theTaskList.push_back(new cTask_MergeScenario(false, theManeuver.action));
            break;
        case maneuver::maneuver_merge_right:
            theTaskList.push_back(new cTask_DoNothing()); //TODO
            break;
        case maneuver::maneuver_pull_out_left:
            theTaskList.push_back(new cTask_PullOutLeft(firstOfManeuver));
            break;
        case maneuver::maneuver_pull_out_right:
            theTaskList.push_back(new cTask_PullOutRight(firstOfManeuver));
            break;
        case maneuver::maneuver_cross_parking:
            theTaskList.push_back(new cTask_LaneFollowing(firstOfManeuver));
            theTaskList.push_back(new cTask_PullInRight(false, theManeuver.extra));
            break;
        case maneuver::maneuver_parallel_parking:
            theTaskList.push_back(new cTask_LaneFollowing()); //pure lanekeeping testing
            theTaskList.push_back(new cTask_LaneFollowing());
            break;
        case maneuver::manuever_undefined:
            theTaskList.push_back(new cTask_DoNothing()); //TODO
            break;
    }

    RETURN_NOERROR;
}

tBool cGoalController::isNextTaskReady() {
    return m_taskList.size() > 1 && m_taskList.at(1)->canTakeOverControl();
}

// ------  Maneuver getters ------

tInt16 cGoalController::getCurrentSectorNumber() {
    return m_iCurrentSectorNumber;
}

tInt16 cGoalController::getCurrentManeuverNumber() {
    return m_iCurrentManeuverNumber;
}


// ------ Task handling helpers ------

/*! removes first task from taskQueue */
void cGoalController::completeCurrentTask() {
    if (!m_taskList.empty()) {
        m_taskList.erase(m_taskList.begin());
    }
    if(m_taskList.empty()) {// task List is empty again, sector finished: start next sector if possible
        if (int(m_maneuverList.size() - 1) > m_iCurrentSectorNumber) {
            m_iCurrentSectorNumber++;
            createTaskListFromSector(m_iCurrentSectorNumber);
        }
    } else if(getCurrentTask().isFirstTaskOfManeuver()) {
        m_iCurrentManeuverNumber = m_iCurrentManeuverNumber + 1;
    }
}



// ................................

// ------  Maneuver helpers ------

/*! Get maneuver from absolute maneuver entry number from maneuverList */
aadc::jury::maneuver cGoalController::getManeuver(tInt16 maneuverEntry) {
    for (auto const &currentSector: m_maneuverList) {
        for (auto const &currentManeuver: currentSector.sector) {
            if (currentManeuver.id == maneuverEntry) {
                return currentManeuver.action;
            }
        }
    }
    // if not found: return default value
    return aadc::jury::maneuver_parallel_parking;
}

/*! Get next maneuver from maneuverList */
aadc::jury::maneuver cGoalController::setNextManeuver() {
    m_iCurrentManeuverNumber = m_iCurrentManeuverNumber + 1;
    return getManeuver(m_iCurrentManeuverNumber);
}


// ------ Task handling ------

cTask &cGoalController::getCurrentTask() {
    if(m_taskList.empty()) {
        return *new cTask_DoNothing;
    }
//    static cTask_EmergencyCar task = *new cTask_EmergencyCar;
//    return task;
    return *m_taskList.at(0);
}

tBool cGoalController::isDone() {
    return m_taskList.empty();
}

// adds immediate task to front of taskQueue (potentially list of tasks)
void cGoalController::addImmediateManeuver(fhw::extraManeuvers immediateManeuver) {
    appendTaskListFromManeuver(immediateManeuver, m_taskList);
}

// finds the corresponding sector for an maneuverid, or returns -1 when not found
tInt16 cGoalController::findSectorByManeueverID(tInt16 maneuverID) {
    for (unsigned int sector = 0; sector < m_maneuverList.size(); sector++) {
        tSector requestedSector = m_maneuverList.at(sector);
        for (auto const &currentManeuver: requestedSector.sector) {
            if (currentManeuver.id == maneuverID ) {
                return sector;
            }
        }
    }

    return -1;

}


void cGoalController::restartFromSection(tInt16 maneueverID) {
    // TODO (ask Audi maybe?) start from maneuver, not from section
    m_taskList.clear();

    tInt16 sector = findSectorByManeueverID(maneueverID);
    if (sector == -1) {
        LOG_ERROR("COULD NOT LOAD MANEUVER %f -- NOT FOUND. ABORTING.");
        return;
    }
    createTaskListFromSector(sector);
}


// ------ Tasklist from maneuverlist helpers ------

tResult cGoalController::setManeuverList(cString maneuverListString) {
    LOG_INFO("Received ManeuverList! -> %s", maneuverListString);

    // set and parse maneuver list
    m_strManeuverFileString = maneuverListString;
    parseManeuverList();

    // set current sector and maneuver
    m_iCurrentSectorNumber = 0;
    m_iCurrentManeuverNumber = 0;
    m_taskList.clear();

    RETURN_NOERROR;
}


// ------ Parse maneuver list

tResult cGoalController::parseManeuverList() {
    m_maneuverList.clear();
    // create dom from string received from pin
    adtf::util::cDOM oDOM;
    oDOM.FromString(m_strManeuverFileString);
    adtf::util::cDOMElementRefList oSectorElems;
    adtf::util::cDOMElementRefList oManeuverElems;

    //read first Sector Elem
    if (IS_OK(oDOM.FindNodes("AADC-Maneuver-List/AADC-Sector", oSectorElems))) {
        //iterate through sectors
        for (adtf::util::cDOMElementRefList::iterator itSectorElem = oSectorElems.begin(); itSectorElem != oSectorElems.end(); ++itSectorElem) {
            //if sector found
            aadc::jury::tSector sector;
            sector.id = (*itSectorElem)->GetAttributeInt("id");

            if (IS_OK((*itSectorElem)->FindNodes("AADC-Maneuver", oManeuverElems))) {
                //iterate through maneuvers
                for (adtf::util::cDOMElementRefList::iterator itManeuverElem = oManeuverElems.begin(); itManeuverElem != oManeuverElems.end(); ++itManeuverElem) {
                    aadc::jury::tManeuver man;
                    man.id = (*itManeuverElem)->GetAttributeInt("id");
                    man.action = aadc::jury::maneuverFromString((*itManeuverElem)->GetAttribute("action").GetPtr());
                    man.extra = (*itManeuverElem)->GetAttributeInt("extra", -1);
                    sector.sector.push_back(man);
                }
            }

            m_maneuverList.push_back(sector);
        }
    }

    LOG_INFO("------ Parsed Sector List");

    for (tSector sector : m_maneuverList) {
        LOG_INFO("\tSector #%d with %u items.", sector.id, sector.sector.size());
        for (tManeuver maneuver : sector.sector) {
            LOG_INFO("\t\tManeuver #%d: %s (%d) | Extra: %d", maneuver.id, aadc::jury::maneuverToString(aadc::jury::maneuver(maneuver.action)).c_str(), maneuver.action, maneuver.extra);
        }
    }

    LOG_INFO("------ Parsed Sector List");




    if (oSectorElems.size() > 0) {
        LOG_INFO("Loaded Maneuver file successfully.");
    } else {
        LOG_ERROR("No valid Maneuver Data found!");
        RETURN_ERROR(ERR_INVALID_FILE);
    }

    RETURN_NOERROR;
}