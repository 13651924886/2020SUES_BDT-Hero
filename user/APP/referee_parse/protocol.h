#ifndef ROBOMASTER_PROTOCOL_H
#define ROBOMASTER_PROTOCOL_H

//#include "struct_typedef.h"
#include "stm32f4xx.h"

#define HEADER_SOF 0xA5
#define REF_PROTOCOL_FRAME_MAX_SIZE         128
#define REF_PROTOCOL_DATA_MAX_SIZE         	113


#define REF_PROTOCOL_HEADER_SIZE            sizeof(frame_header_struct_t)
#define REF_PROTOCOL_CMD_SIZE               2
#define REF_PROTOCOL_CRC16_SIZE             2
#define REF_HEADER_CRC_LEN                  (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE)
#define REF_HEADER_CRC_CMDID_LEN            (REF_PROTOCOL_HEADER_SIZE + REF_PROTOCOL_CRC16_SIZE + sizeof(uint16_t))
#define REF_HEADER_CMDID_LEN                (REF_PROTOCOL_HEADER_SIZE + sizeof(uint16_t))

#pragma pack(push, 1)

//5字节帧头结构体
typedef __packed struct
{
  uint8_t SOF;
  uint16_t data_length;
  uint8_t seq;
  uint8_t CRC8;
} frame_header_struct_t;

//5字节帧头,偏移位置
typedef enum
{
	SOF          = 0,//起始位
	DATA_LENGTH  = 1,//帧内数据长度,根据这个来获取数据长度
	SEQ          = 3,//包序号
	CRC8         = 4 //CRC8
	
}FrameHeaderOffset;

typedef enum 
{
	FRAME_HEADER         = 0,
	CMD_ID               = 5,
	DATA                 = 7,
	
}JudgeFrameOffset;
/***************命令码ID********************/

/* 

	ID: 0x0001  Byte:  3    比赛状态数据       				发送频率 1HZ      
	ID: 0x0002  Byte:  1    比赛结果数据         			比赛结束后发送      
	ID: 0x0003  Byte:  32   比赛机器人存活数据   			1Hz发送
	ID: 0x0004  Byte:  3    飞镖发射状态							飞镖发射时发送
	ID: 0x0101  Byte:  4    场地事件数据   						1HZ
	ID: 0x0102  Byte:  4    场地补给站动作标识数据    动作改变后发送 
	ID: 0X0104  Byte:  2    裁判警告数据      				警告发生后发送
	ID: 0X0105  Byte:  1    飞镖发射口倒计时      		1HZ
	ID: 0X0201  Byte: 18    机器人状态数据        		10Hz
	ID: 0X0202  Byte: 16    实时功率热量数据   				50Hz       
	ID: 0x0203  Byte: 16    机器人位置数据           	10Hz
	ID: 0x0204  Byte:  1    机器人增益数据           	1HZ
	ID: 0x0205  Byte:  3    空中机器人能量状态数据    10Hz，只有空中机器人主控发送
	ID: 0x0206  Byte:  1    伤害状态数据           		伤害发生后发送
	ID: 0x0207  Byte:  6    实时射击数据           		子弹发射后发送
	ID: 0x0208  Byte:  2    弹丸剩余发射数           	1HZ，仅空中机器人发送
	ID: 0x0209  Byte:  4    机器人RFID状态          1HZ
	ID: 0x020A  Byte:  12   飞镖机器人客户端指令数据  10HZ
	ID: 0x0301  Byte:  n    机器人间交互数据          发送方触发发送
	
*/
typedef enum
{
    GAME_STATE_CMD_ID                 = 0x0001,	//比赛状态数据
    GAME_RESULT_CMD_ID                = 0x0002,	//比赛结果数据
    GAME_ROBOT_HP_CMD_ID              = 0x0003,	//比赛机器人存活数据
		MISSILE_STATUS										= 0x0004,	//飞镖发射状态
    FIELD_EVENTS_CMD_ID               = 0x0101,	//场地事件数据
    SUPPLY_PROJECTILE_ACTION_CMD_ID   = 0x0102,	//场地补给站动作标识数据
	//Q1:2020/5更新的裁判系统协议里没有0x0103这一项
		SUPPLY_PROJECTILE_BOOKING_CMD_ID  = 0x0103, 
    REFEREE_WARNING_CMD_ID            = 0x0104,	//裁判警告数据
		MISSILE_SILO_COUNT								= 0x0105,	//飞镖发射口倒计时
    ROBOT_STATE_CMD_ID                = 0x0201,	//机器人状态数据
    POWER_HEAT_DATA_CMD_ID            = 0x0202,	//实时功率热量数据
    ROBOT_POS_CMD_ID                  = 0x0203,	//机器人位置数据
    BUFF_MUSK_CMD_ID                  = 0x0204, //机器人增益数据
    AERIAL_ROBOT_ENERGY_CMD_ID        = 0x0205, //空中机器人能量状态数据
    ROBOT_HURT_CMD_ID                 = 0x0206, //伤害状态数据
    SHOOT_DATA_CMD_ID                 = 0x0207, //实时射击数据
    BULLET_REMAINING_CMD_ID           = 0x0208, //弹丸剩余发射数
		RFID_STATUS												= 0x0209, //机器人RFID状态
		MISSILE_LAUCNH_SIGNAL							= 0x020A, //飞镖机器人客户端指令数据
    STUDENT_INTERACTIVE_DATA_CMD_ID   = 0x0301, //机器人间交互数据
    IDCustomData,
}referee_cmd_id_t;

typedef enum
{
  STEP_HEADER_SOF  = 0,
  STEP_LENGTH_LOW  = 1,
  STEP_LENGTH_HIGH = 2,
  STEP_FRAME_SEQ   = 3,
  STEP_HEADER_CRC8 = 4,
  STEP_DATA_CRC16  = 5,
} unpack_step_e;

typedef enum
{
	
  DELETE_GRAPHIC  = 0x0100,
  DRAW_ONE  			= 0x0101,
  DRAW_TWO			  = 0x0102,
  DRAW_FIVE 		  = 0x0103,
	DRAW_SEVEN			= 0x0104,
	DRAW_WORDS 			= 0x0110,
	
} interactive_client_draw_data_cmd_id_e;

typedef struct
{
  frame_header_struct_t *p_header;
  uint16_t       data_len;
  uint8_t        protocol_packet[REF_PROTOCOL_FRAME_MAX_SIZE];
  unpack_step_e  unpack_step;
  uint16_t       index;
} unpack_data_t;


/* ID: 0x0001  Byte:  3    比赛状态数据 */
typedef enum
{
    RED_HERO        = 1,
    RED_ENGINEER    = 2,
    RED_STANDARD_1  = 3,
    RED_STANDARD_2  = 4,
    RED_STANDARD_3  = 5,
    RED_AERIAL      = 6,
    RED_SENTRY      = 7,
		RED_RADAR       = 9,
	
    BLUE_HERO       = 101,
    BLUE_ENGINEER   = 102,
    BLUE_STANDARD_1 = 103,
    BLUE_STANDARD_2 = 104,
    BLUE_STANDARD_3 = 105,
    BLUE_AERIAL     = 106,
    BLUE_SENTRY     = 107,
		BLUE_RADAR      = 109,
	
} robot_id_t;

typedef enum
{
    RED_HERO_Client				 = 0x0101,
    RED_ENGINEER_Client    = 0x0102,
    RED_STANDARD_1_Client  = 0x0103,
    RED_STANDARD_2_Client  = 0x0104,
    RED_STANDARD_3_Client  = 0x0105,
    RED_AERIAL_Client      = 0x0106,

    BLUE_HERO_Client       = 0x0165,
    BLUE_ENGINEER_Client   = 0x0166,
    BLUE_STANDARD_1_Client = 0x0167,
    BLUE_STANDARD_2_Client = 0x0168,
    BLUE_STANDARD_3_Client = 0x0169,
    BLUE_AERIAL_Client     = 0x016A,
	
} PC_client_id_t;

typedef enum
{
    PROGRESS_UNSTART        = 0,
    PROGRESS_PREPARE        = 1,
    PROGRESS_SELFCHECK      = 2,
    PROGRESS_5sCOUNTDOWN    = 3,
    PROGRESS_BATTLE         = 4,
    PROGRESS_CALCULATING    = 5,
} game_progress_t;


// 数据结构体
typedef __packed struct //0x0001 3字节，24位
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
} ext_game_state_t;

typedef __packed struct //0x0002 1字节
{
    uint8_t winner;
} ext_game_result_t;

typedef __packed struct//0x0003	32字节
{
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;
    uint16_t blue_base_HP;
} ext_game_robot_HP_t;

typedef __packed struct //0x0101 4字节
{
    uint32_t event_type;
} ext_event_data_t;

typedef __packed struct //0x0102 4字节
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_projectile_step;
    uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;


typedef __packed struct //0x0103 
{
    uint8_t supply_projectile_id;
    uint8_t supply_robot_id;
    uint8_t supply_num;
} ext_supply_projectile_booking_t;

typedef __packed struct //0x104 2字节
{
    uint8_t level;
    uint8_t foul_robot_id;
} ext_referee_warning_t;
typedef __packed struct //0x0201 18字节
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t remain_HP;
    uint16_t max_HP;
    uint16_t shooter_heat0_cooling_rate;
    uint16_t shooter_heat0_cooling_limit;
    uint16_t shooter_heat1_cooling_rate;
    uint16_t shooter_heat1_cooling_limit;
    uint8_t mains_power_gimbal_output : 1;
    uint8_t mains_power_chassis_output : 1;
    uint8_t mains_power_shooter_output : 1;
} ext_game_robot_state_t;

typedef __packed struct //0x0202
{
    uint16_t chassis_volt;
    uint16_t chassis_current;
    float chassis_power;
    uint16_t chassis_power_buffer;
    uint16_t shooter_heat0;
    uint16_t shooter_heat1;
} ext_power_heat_data_t;

typedef __packed struct //0x0203
{
    float x;
    float y;
    float z;
    float yaw;
} ext_game_robot_pos_t;

typedef __packed struct //0x0204
{
    uint8_t power_rune_buff;
} ext_buff_musk_t;

typedef __packed struct //0x0205
{
    uint8_t energy_point;
    uint8_t attack_time;
} aerial_robot_energy_t;

typedef __packed struct //0x0206
{
    uint8_t armor_type : 4;
    uint8_t hurt_type : 4;
} ext_robot_hurt_t;

typedef __packed struct //0x0207
{
    uint8_t bullet_type;
    uint8_t bullet_freq;
    float bullet_speed;
} ext_shoot_data_t;
typedef __packed struct
{
    uint8_t bullet_remaining_num;
} ext_bullet_remaining_t;

typedef __packed struct //0x0301
{
		uint16_t data_cmd_id;
    uint16_t send_ID;
    uint16_t receiver_ID;
    
} ext_student_interactive_header_data_t;

typedef __packed struct //0x0301
{
		ext_student_interactive_header_data_t header_data;
    uint16_t data_len;
    uint8_t *data;
} ext_student_interactive_data_t;

typedef __packed struct
{
	
		uint8_t data[REF_PROTOCOL_DATA_MAX_SIZE];
	
}robot_interactive_data_t;

typedef __packed struct
{
		uint8_t operate_type;	//0:空操作   1:删除图层    2:删除所有
		uint8_t layer;				//图层0 - 9
}ext_client_custom_graphic_delete_t;

typedef __packed struct
{
	uint8_t graphic_name[3];
	uint32_t operate_type:3;
	uint32_t graphic_type:3;
	uint32_t layer:4;
	uint32_t color:4;
	uint32_t start_angle:9;
	uint32_t end_angle:9;
	uint32_t width:10;
	uint32_t start_x:11;
	uint32_t start_y:11;
	uint32_t radius:10;
	uint32_t end_x:11;
	uint32_t end_y:11;
	
}graphic_data_struct_t;


typedef __packed struct
{
		graphic_data_struct_t graphic_data_struct;

}ext_client_custom_graphic_single_t;

typedef __packed struct
{
		graphic_data_struct_t graphic_data_struct[2];
}ext_client_custom_graphic_double_t;

typedef __packed struct
{
		graphic_data_struct_t graphic_data_struct[5];
}ext_client_custom_graphic_five_t;

typedef __packed struct
{
		graphic_data_struct_t graphic_data_struct[7];
}ext_client_custom_graphic_seven_t;

typedef __packed struct
{
		graphic_data_struct_t graphic_data_struct;
		uint8_t data[30];
}ext_client_custom_character_t;

//机器人交互信息 cmd_id = 0x301,内容id = 0x0200 - 0x02FF
typedef __packed struct
{
	frame_header_struct_t 									txFrameHeader;//帧头
	uint16_t																CmdID;//命令码
	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
	robot_interactive_data_t  	 						interactData;//数据段
	uint16_t		 														FrameTail;//帧尾
}ext_RobotCommunication_t;

//客户端交互信息 cmd_id = 0x301,内容id = 0x0100
typedef __packed struct
{
	frame_header_struct_t 									txFrameHeader;//帧头
	uint16_t																CmdID;//命令码
	ext_student_interactive_header_data_t   dataFrameHeader;//数据段头结构
	ext_client_custom_graphic_single_t  	  interactData;//数据段
	uint16_t		 														FrameTail;//帧尾
}ext_Client_Communication_custom_graphic_single_t;


typedef __packed struct
{
    float data1;
    float data2;
    float data3;
    uint8_t data4;
} custom_data_t;


typedef __packed struct
{
    uint8_t data[64];
} ext_up_stream_data_t;

typedef __packed struct
{
    uint8_t data[32];
} ext_download_stream_data_t;

#pragma pack(pop)

#endif //ROBOMASTER_PROTOCOL_H
