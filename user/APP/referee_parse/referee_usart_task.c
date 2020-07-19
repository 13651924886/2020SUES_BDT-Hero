/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       referee_usart_task.c/h
  * @brief      RM referee system data solve. RM裁判系统数据处理
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "main.h"
#include "stdbool.h" 	// 引入以使用bool类型
#include "string.h"

#include "uart8.h"
#include "usart6.h"
#include "detect_task.h"
#include "referee_usart_task.h"
#include "CRC8_CRC16.h"
#include "fifo.h"

#define USART_RX_BUF_LENGHT     512
#define REFEREE_FIFO_BUF_LENGTH 1024

#define    FALSE    0
#define    TRUE     1

#define    LEN_HEADER    5        //帧头长
#define    LEN_CMDID     2        //命令码长度
#define    LEN_TAIL      2	      //帧尾CRC16

//起始字节,协议固定为0xA5
#define    JUDGE_FRAME_HEADER         (0xA5)

uint8_t usart6_buf[2][USART_RX_BUF_LENGHT];
fifo_s_t referee_fifo;
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];

/*-------------------------2020--------------------------------*/

/*****************系统数据定义**********************/
ext_game_state_t       						game_state;										//0x0001
ext_game_result_t            			game_result;									//0x0002
ext_game_robot_HP_t          			game_robot_HP_t;							//0x0003
ext_event_data_t        					field_event;									//0x0101
ext_supply_projectile_action_t		supply_projectile_action_t;		//0x0102
ext_supply_projectile_booking_t		supply_projectile_booking_t;	//0x0103
ext_referee_warning_t							referee_warning_t;						//0x0104
//TODO2:添加0x0105,并在.h文件里新建0x0105的结构体
ext_game_robot_state_t			  		robot_state;									//0x0201
ext_power_heat_data_t		  				power_heat_data_t;						//0x0202
ext_game_robot_pos_t							game_robot_pos_t;							//0x0203
ext_buff_musk_t										buff_musk_t;									//0x0204
aerial_robot_energy_t							robot_energy_t;								//0x0205
ext_robot_hurt_t									robot_hurt_t;									//0x0206
ext_shoot_data_t									shoot_data_t;									//0x0207
ext_bullet_remaining_t 						bullet_remaining_t;						//0x0208
ext_student_interactive_data_t		student_interactive_data;

ext_RobotCommunication_t  				RobotCommunatianData;					//0x0301
ext_Client_Communication_custom_graphic_single_t client_com_graphic_single;
/****************************************************/

uint8_t Judge_Self_ID;//当前机器人的ID
uint16_t Judge_SelfClient_ID;//发送者机器人对应的客户端ID

unpack_data_t referee_unpack_obj;
unpack_data_t *judge = &referee_unpack_obj;

//裁判系统数据协议头
frame_header_struct_t referee_receive_header;
frame_header_struct_t referee_send_header;



/* static */
static void init_referee_struct_data(void);
static void Judge_Read_Data(void);
static void referee_data_solve(uint8_t *frame);


void Referee_Task_Init()
{
	init_referee_struct_data();	//初始化结构体数据对象
	fifo_s_init(&referee_fifo, referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);	//创建队列FIFO
	USART6_DMA_Init(usart6_buf[0], usart6_buf[1], USART_RX_BUF_LENGHT);	//初始化裁判系统串口6+DMA，开启串口空闲中断
}

void referee_usart_task(void)
{
	Judge_Read_Data();
}

//TODO:测试TeammateTxBuffer数组和RobotCommunatianData.interactData.data的区别
unsigned char TeammateTxBuffer[REF_PROTOCOL_FRAME_MAX_SIZE];

void referee_tx_task(void) //SOF:A5  data_l:15 00  seq:00  CRC8:BC   CMD_ID：0103  data:0101 0100 0101/ 01 00 00/ 49 10 1E 78 C8 D0 87 3E 64 A0 0F 7D    frame_tail:3A 40
//A5 00 15 00 D0 03 01 10 10 01 01 01 01 01 00 00 49 10 1E 78 C8 D0 87 3E 64 A0 0F 7D 8C 57
//49 10 1E 78 C8 D0 87 3E 64 A0 0F 7D B1 DF
{
	static u8 datalength,i;
	
	//Frame_Header
	client_com_graphic_single.txFrameHeader.SOF = 0xA5;
	client_com_graphic_single.txFrameHeader.data_length = sizeof(ext_student_interactive_header_data_t) + sizeof(ext_client_custom_graphic_single_t); //6字节+15字节
	client_com_graphic_single.txFrameHeader.seq = 0;
	memcpy(&TeammateTxBuffer, &client_com_graphic_single.txFrameHeader, sizeof(frame_header_struct_t));	
	append_CRC8_check_sum(TeammateTxBuffer, sizeof(frame_header_struct_t));

	//cmd_id
	client_com_graphic_single.CmdID = 0x301;
	
	//数据段头结构
	client_com_graphic_single.dataFrameHeader.send_ID 		 = RED_HERO;	
	client_com_graphic_single.dataFrameHeader.data_cmd_id = DRAW_ONE;
	client_com_graphic_single.dataFrameHeader.receiver_ID = RED_HERO_Client;
	
	client_com_graphic_single.interactData.graphic_data_struct.graphic_name[0] = 1;
	client_com_graphic_single.interactData.graphic_data_struct.operate_type = 1;
	client_com_graphic_single.interactData.graphic_data_struct.graphic_type = 2;
	client_com_graphic_single.interactData.graphic_data_struct.layer = 0;
	client_com_graphic_single.interactData.graphic_data_struct.color = 4;
	client_com_graphic_single.interactData.graphic_data_struct.start_angle = 0;
	client_com_graphic_single.interactData.graphic_data_struct.end_angle = 0;
	client_com_graphic_single.interactData.graphic_data_struct.width = 3;
	client_com_graphic_single.interactData.graphic_data_struct.start_x = 1060;
	client_com_graphic_single.interactData.graphic_data_struct.start_y = 640;
	client_com_graphic_single.interactData.graphic_data_struct.radius = 60;
	client_com_graphic_single.interactData.graphic_data_struct.end_x = 0;
	client_com_graphic_single.interactData.graphic_data_struct.end_y = 0;
	
	memcpy(TeammateTxBuffer+5,(uint8_t *)&client_com_graphic_single.CmdID,
	(sizeof(client_com_graphic_single.CmdID)+sizeof(client_com_graphic_single.dataFrameHeader)+sizeof(client_com_graphic_single.interactData)));	

	append_CRC16_check_sum(TeammateTxBuffer,sizeof(client_com_graphic_single));
	
	datalength = sizeof(client_com_graphic_single); 

//	if( First_Time_Send_Commu == TRUE )
//	{
		for(i = 0;i < datalength;i++)
		{
			USART_SendData(USART6,(uint16_t)TeammateTxBuffer[i]);
			while(USART_GetFlagStatus(USART6,USART_FLAG_TC)==RESET);
		}	 
//	}
}

static void init_referee_struct_data(void)
{
    memset(&referee_receive_header, 0, sizeof(frame_header_struct_t));
    memset(&referee_send_header, 0, sizeof(frame_header_struct_t));

    memset(&game_state, 0, sizeof(ext_game_state_t));
    memset(&game_result, 0, sizeof(ext_game_result_t));
    memset(&game_robot_HP_t, 0, sizeof(ext_game_robot_HP_t));


    memset(&field_event, 0, sizeof(ext_event_data_t));
    memset(&supply_projectile_action_t, 0, sizeof(ext_supply_projectile_action_t));
    memset(&supply_projectile_booking_t, 0, sizeof(ext_supply_projectile_booking_t));
    memset(&referee_warning_t, 0, sizeof(ext_referee_warning_t));


    memset(&robot_state, 0, sizeof(ext_game_robot_state_t));
    memset(&power_heat_data_t, 0, sizeof(ext_power_heat_data_t));
    memset(&game_robot_pos_t, 0, sizeof(ext_game_robot_pos_t));
    memset(&buff_musk_t, 0, sizeof(ext_buff_musk_t));
    memset(&robot_energy_t, 0, sizeof(aerial_robot_energy_t));
    memset(&robot_hurt_t, 0, sizeof(ext_robot_hurt_t));
    memset(&shoot_data_t, 0, sizeof(ext_shoot_data_t));
    memset(&bullet_remaining_t, 0, sizeof(ext_bullet_remaining_t));
		memset(&student_interactive_data, 0, sizeof(ext_student_interactive_data_t));


    memset(&client_com_graphic_single, 0, sizeof(ext_Client_Communication_custom_graphic_single_t));
}
// 裁判系统数据读取并送入解析
static void Judge_Read_Data(void)
{
	uint8_t G_byte = 0;
	uint8_t sof = HEADER_SOF; //0xA5

	while ( fifo_s_used(&referee_fifo) )	//Get FIFO the number of elements
	{
		    G_byte = fifo_s_get(&referee_fifo);

				switch(judge->unpack_step)	//一共6步
				{
					////////Step 1 判断帧头
					case STEP_HEADER_SOF:
          {
						//判断当前字节是否为帧头0xA5
            if(G_byte == sof)	// sof局部变量 0xA5
            {
              judge->unpack_step = STEP_LENGTH_LOW; // 2
              judge->protocol_packet[judge->index++] = G_byte;//将当前帧头按index顺序放入数据包中
             }
						//如果不是0xA5，则数据包序号清零（索引到数据包首位），继续等待0xA5出现
            else
            {
              judge->index = 0;
            }
          }break;
					////////Step 2 
          case STEP_LENGTH_LOW:
          {						
            judge->data_len = G_byte;
            judge->protocol_packet[judge->index++] = G_byte;
            judge->unpack_step = STEP_LENGTH_HIGH;
          }break;
      
          case STEP_LENGTH_HIGH:
          {
            judge->data_len |= (G_byte << 8);
            judge->protocol_packet[judge->index++] = G_byte;

               if(judge->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
               {
                 judge->unpack_step = STEP_FRAME_SEQ;
                }
               else
               {
                 judge->unpack_step = STEP_HEADER_SOF;
                 judge->index = 0;
                }
          }break;
		  
          case STEP_FRAME_SEQ:
          {
            judge->protocol_packet[judge->index++] = G_byte;
            judge->unpack_step = STEP_HEADER_CRC8;
          }break;

          case STEP_HEADER_CRC8:
          {
            judge->protocol_packet[judge->index++] = G_byte;
            if (judge->index == REF_PROTOCOL_HEADER_SIZE)
            {
              if (verify_CRC8_check_sum(judge->protocol_packet, REF_PROTOCOL_HEADER_SIZE) )
               {
                judge->unpack_step = STEP_DATA_CRC16;
               }
            else
              {
               judge->unpack_step = STEP_HEADER_SOF;
               judge->index = 0;
              }
		         }
					 }break;  
      
           case STEP_DATA_CRC16:
           {
							if (judge->index < (REF_HEADER_CRC_CMDID_LEN + judge->data_len))
							{
								judge->protocol_packet[judge->index++] = G_byte;  
							 }
							if (judge->index >= (REF_HEADER_CRC_CMDID_LEN + judge->data_len))
							{
								judge->unpack_step = STEP_HEADER_SOF;
								judge->index = 0;
								if (verify_CRC16_check_sum(judge->protocol_packet, REF_HEADER_CRC_CMDID_LEN + judge->data_len) )
								{
									referee_data_solve(judge->protocol_packet);
								}
							}
            }break;

					default:
					{
						judge->unpack_step = STEP_HEADER_SOF;
						judge->index = 0;
					}break;	  	
			}
		}
	}
static void referee_data_solve(uint8_t *frame)
{		
		uint8_t index1 = 0;
		uint16_t cmd_id = 0;
    memcpy(&referee_receive_header, frame, sizeof(frame_header_struct_t)); //size为5字节

    index1 += sizeof(frame_header_struct_t); //size为5字节

    memcpy(&cmd_id, frame + index1, sizeof(uint16_t)); //cmd_id 2字节 
    index1 += sizeof(uint16_t); // index1索引到数据
	
		// 判断读取到的命令码是对应哪种数据以及数据长度
    switch (cmd_id)
    {
        case GAME_STATE_CMD_ID:
        {
            memcpy(&game_state, frame + index1, sizeof(ext_game_state_t));
        }
        break;
        case GAME_RESULT_CMD_ID:
        {
            memcpy(&game_result, frame + index1, sizeof(game_result));
        }
        break;
        case GAME_ROBOT_HP_CMD_ID:
        {
            memcpy(&game_robot_HP_t, frame + index1, sizeof(ext_game_robot_HP_t));
        }
        break;


        case FIELD_EVENTS_CMD_ID:
        {
            memcpy(&field_event, frame + index1, sizeof(field_event));
        }
        break;
        case SUPPLY_PROJECTILE_ACTION_CMD_ID:
        {
            memcpy(&supply_projectile_action_t, frame + index1, sizeof(supply_projectile_action_t));
        }
        break;
        case SUPPLY_PROJECTILE_BOOKING_CMD_ID:
        {
            memcpy(&supply_projectile_booking_t, frame + index1, sizeof(supply_projectile_booking_t));
        }
        break;
        case REFEREE_WARNING_CMD_ID:
        {
            memcpy(&referee_warning_t, frame + index1, sizeof(ext_referee_warning_t));
        }
        break;

        case ROBOT_STATE_CMD_ID:
        {
            memcpy(&robot_state, frame + index1, sizeof(robot_state));
        }
        break;
        case POWER_HEAT_DATA_CMD_ID:
        {
            memcpy(&power_heat_data_t, frame + index1, sizeof(power_heat_data_t));
        }
        break;
        case ROBOT_POS_CMD_ID:
        {
            memcpy(&game_robot_pos_t, frame + index1, sizeof(game_robot_pos_t));
        }
        break;
        case BUFF_MUSK_CMD_ID:
        {
            memcpy(&buff_musk_t, frame + index1, sizeof(buff_musk_t));
        }
        break;
        case AERIAL_ROBOT_ENERGY_CMD_ID:
        {
            memcpy(&robot_energy_t, frame + index1, sizeof(robot_energy_t));
        }
        break;
        case ROBOT_HURT_CMD_ID:
        {
            memcpy(&robot_hurt_t, frame + index1, sizeof(robot_hurt_t));
        }
        break;
        case SHOOT_DATA_CMD_ID:
        {
            memcpy(&shoot_data_t, frame + index1, sizeof(shoot_data_t));
        }
        break;
        case BULLET_REMAINING_CMD_ID:
        {
            memcpy(&bullet_remaining_t, frame + index1, sizeof(ext_bullet_remaining_t));
        }
        break;
        case STUDENT_INTERACTIVE_DATA_CMD_ID:
        {
            memcpy(&student_interactive_data, frame + index1, sizeof(ext_student_interactive_data_t));
        }
        break;
        default:
        {
            break;
        }
    }
}





void USART6_IRQHandler(void)
{	
	if (USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)
    {
        USART_ReceiveData(USART6);
    }
    else if (USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)
    {
        static uint16_t this_time_rx_len = 0;
        USART_ReceiveData(USART6);

        if(DMA_GetCurrentMemoryTarget(DMA2_Stream1) == 0)
        {
            //重新设置DMA
            DMA_Cmd(DMA2_Stream1, DISABLE);
            this_time_rx_len = USART_RX_BUF_LENGHT - DMA_GetCurrDataCounter(DMA2_Stream1);
            DMA_SetCurrDataCounter(DMA2_Stream1, USART_RX_BUF_LENGHT);
            DMA2_Stream1->CR |= DMA_SxCR_CT;
            //清DMA中断标志
            DMA_ClearFlag(DMA2_Stream1, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);
            DMA_Cmd(DMA2_Stream1, ENABLE);
						fifo_s_puts(&referee_fifo, (char*)usart6_buf[0], this_time_rx_len);
        }
        else
        {
            //重新设置DMA
            DMA_Cmd(DMA2_Stream1, DISABLE);
						//
            this_time_rx_len = USART_RX_BUF_LENGHT - DMA_GetCurrDataCounter(DMA2_Stream1);
            DMA_SetCurrDataCounter(DMA2_Stream1, USART_RX_BUF_LENGHT);
            DMA2_Stream1->CR &= ~(DMA_SxCR_CT);
            //清DMA中断标志
            DMA_ClearFlag(DMA2_Stream1, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);
            DMA_Cmd(DMA2_Stream1, ENABLE);
            fifo_s_puts(&referee_fifo, (char*)usart6_buf[1], this_time_rx_len);


        }
    }
}
