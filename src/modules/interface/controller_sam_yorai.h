//
// Created by chris on 7/29/21.
//

#ifndef CRAZYFLIE_FIRMWARE_CONTROLLER_SAM_YORAI_H
#define CRAZYFLIE_FIRMWARE_CONTROLLER_SAM_YORAI_H

#include "stabilizer_types.h"

void controllerSamYoraiInit(void);

bool controllerSamYoraiTest(void);

void controllerSamYorai(control_t *control, setpoint_t* setpoint,
                        const sensorData_t *sensors, const state_t* state,
                        const uint32_t tick);

#endif //CRAZYFLIE_FIRMWARE_CONTROLLER_SAM_YORAI_H
