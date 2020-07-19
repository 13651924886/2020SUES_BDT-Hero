#include "referee_API.h"

extern ext_game_robot_state_t			  		robot_state;									//0x0201
extern ext_power_heat_data_t		  			power_heat_data_t;						//0x0202

void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer)
{
    *power = power_heat_data_t.chassis_power;
    *buffer = power_heat_data_t.chassis_power_buffer;

}

const ext_power_heat_data_t *get_power_heat_data_Point(void)
{
    return &power_heat_data_t;
}

const ext_game_robot_state_t *get_robot_state_data_Point(void)
{
		return &robot_state;
}

uint8_t get_robot_id(void)
{
    return robot_state.robot_id;
}

void get_shoot_heat0_limit_and_heat0(uint16_t *heat0_limit, uint16_t *heat0)
{
    *heat0_limit = robot_state.shooter_heat0_cooling_limit;
    *heat0 = power_heat_data_t.shooter_heat0;
}

void get_shoot_heat1_limit_and_heat1(uint16_t *heat1_limit, uint16_t *heat1)
{
    *heat1_limit = robot_state.shooter_heat1_cooling_limit;
    *heat1 = power_heat_data_t.shooter_heat1;
}
//≤‚ ‘