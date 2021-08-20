//
// Created by quadcopteruser on 8/19/21.
//

#ifndef CRAZYFLIE_FIRMWARE_CUSTOM_CHECK_DISTANCE_TO_SETPOINT_H
#define CRAZYFLIE_FIRMWARE_CUSTOM_CHECK_DISTANCE_TO_SETPOINT_H

#include "stabilizer_types.h"

//quick function to check if state is near setpoint
bool check_distance_to_setpoint(state_t *state_cf, setpoint_t *setpoint);

#endif //CRAZYFLIE_FIRMWARE_CUSTOM_CHECK_DISTANCE_TO_SETPOINT_H
