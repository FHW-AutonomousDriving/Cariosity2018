#include "Task_DoNothing.h"

cTask_DoNothing::cTask_DoNothing(tBool firstOfManeuver) : cTask(firstOfManeuver)
{
    LOG_INFO("Constructor of Task_DoNothing...");
    // TODO..?
}

tResult cTask_DoNothing::execute(tTaskResult &taskResult)
{
    LOG_INFO("Task_DoNothing: Doing exactly.. nothing.");
    // TODO: assign task result
    RETURN_NOERROR;
}
