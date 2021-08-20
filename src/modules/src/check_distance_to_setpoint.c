//
// Created by quadcopteruser on 8/19/21.
//
#include "check_distance_to_setpoint.h"
#include "math3d.h"
#include "debug.h"



bool check_distance_to_setpoint(state_t *state_cf, setpoint_t *setpoint){
    struct vec setpointPos = mkvec(setpoint->position.x, setpoint->position.y, setpoint->position.z);
    struct vec statePos = mkvec(state_cf->position.x, state_cf->position.y, state_cf->position.z);

    struct vec r_error = vsub(setpointPos, statePos);

    double distance = sqrt(pow((double) r_error.x, 2) + pow((double) r_error.y, 2) +
            pow((double) r_error.z, 2));


    //DEBUG_PRINT("RUN MELLINGER -- CHECK DISTANCE: %f \n", distance);
    if (distance <= 0.01 && setpoint->position.z != 0)
        return true;
    else
        return false;
}