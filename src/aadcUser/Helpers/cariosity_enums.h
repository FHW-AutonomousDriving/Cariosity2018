#pragma once

#include <aadc_jury.h>

namespace fhw {

    enum extraManeuvers {
        nothing,
        crosswalk_handling,
        emergency_vehicle,
        overtaking
        // TODO ...
    };


    /*! the current processing state of a task */
    enum taskState {
        error = -1,
        in_progress = 0,
        finished = 1
    };

    /*! Situation of car (EgoState) */
    enum environmentSituation {
        lanekeeping = 0,
        crossing = 1,
        zebra_crossing = 2,
        ascending = 3,
        descending = 4,
        obstacle_ahead = 5,
        emergency_car = 6
        // ...
    };

    inline std::string juryActionToString(aadc::jury::juryAction theJuryAction) {
        switch(theJuryAction){
            case aadc::jury::action_stop :
                return std::string("STOP");
            case aadc::jury::action_getready :
                return std::string("GET READY");
            case aadc::jury::action_start :
                return std::string("START");
            default :
                return std::string("INVALID");
        }
    }


    /*! classification of lanes at junctions (from open drive)*/
    enum LaneTurnClassification {
        CASUAL,
        JUNCTION_STRAIGHT,
        JUNCTION_LEFT_TURN,
        JUNCTION_RIGHT_TURN
    };


    enum RoadSignType {
        RECHTS_VOR_LINKS = 0,
        STOP = 1,
        PARKING = 2,
        RAKETE = 3,
        STRAIGHT_ONLY = 4,
        GIVE_WAY = 5,
        PEDESTRIAN_CROSSING = 6,
        ROUND_ABOUT = 7,
        NO_OVERTAKING = 8,
        NO_ENTRY = 9,
        /* TEST SIGN is 10 */
        ONE_WAY = 11,
        ROAD_WORKS = 12,
        TEMPO_50 = 13,
        TEMPO_100 = 14
    };

}