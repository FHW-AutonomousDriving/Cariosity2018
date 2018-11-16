#pragma once
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

#include "../Tasks/tasks.h"

using namespace std;

class cGoalController {

private: // fields

// ------ maneuver list

    /*! The maneuver file string */
    cString     m_strManeuverFileString;

    /*! this is the list with all the loaded sections from the maneuver list*/
    aadc::jury::maneuverList m_maneuverList;


// ------ current maneuver information

    int m_iCurrentSectorNumber = 0;
    int m_iCurrentManeuverNumber = 0;



// ------ executable task instances

    /*! List of state-machine task instances that generate actuator outputs */
    vector<cTask*> m_taskList;


private: // methods

// ------ Create task lists

    tResult createTaskListFromSector(int sectorNumber);
    tResult appendTaskListFromManeuver(aadc::jury::tManeuver theManeuver, vector<cTask*> &theTaskList);
    tResult appendTaskListFromManeuver(fhw::extraManeuvers theManeuver, vector<cTask*> &theTaskList);

// ------ Parse maneuver list

    tResult parseManeuverList();

// ------ helpers -----

    tInt16 findSectorByManeueverID(tInt16 maneuverID);

// ------  Maneuver getters ------

    /*! Get maneuver from absolute maneuver entry number from maneuverList */
    aadc::jury::maneuver getManeuver(tInt16 id);

    /*! Get next maneuver from maneuverList */
    aadc::jury::maneuver setNextManeuver();


// ------ Private constructor ------

    cGoalController();


public:
// ------ Constructor stuff ------

    // deleting copy constructor and assignment copy operation
    cGoalController(const cGoalController &) = delete;
    void operator=(cGoalController const &) = delete;

    /*! Get singleton instance of the GoalController */
    static cGoalController &getInstance();

    /*! Destructor. */
    virtual ~cGoalController() = default;


// ------ Jury communication handling ------

    /*! JuryMudule sets maneuverList and logs message READY */
    tResult setManeuverList(cString maneuverListString);


// ------ Positioning information in maneuver list ------

    tInt16 getCurrentSectorNumber();
    tInt16 getCurrentManeuverNumber();
    tBool isDone();
    tBool isNextTaskReady();

// ------ Task handling ------

    /*! returns current task */
    cTask& getCurrentTask();

    /*! removes first task from taskQueue */
    void completeCurrentTask();

    void restartFromSection(tInt16 maneuverID);

    /*! adds immediate task to front of taskQueue (potentially list of tasks) */
    void addImmediateManeuver(fhw::extraManeuvers immediateManeuver);


};