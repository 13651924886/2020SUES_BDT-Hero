#ifndef REFEREE_API_H
#define REFEREE_API_H

#include "main.h"
#include "protocol.h"


extern void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer);

extern uint8_t get_robot_id(void);

extern void get_shoot_heat0_limit_and_heat0(uint16_t *heat0_limit, uint16_t *heat0);
extern void get_shoot_heat1_limit_and_heat1(uint16_t *heat1_limit, uint16_t *heat1);

extern const ext_power_heat_data_t *get_power_heat_data_Point(void);
extern const ext_game_robot_state_t *get_robot_state_data_Point(void);

#endif

