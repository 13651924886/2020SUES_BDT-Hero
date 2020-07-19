#include "main.h"
#include <stm32f4xx.h> 
#include "usart6.h"

//void Referee_USART6_Configuration(u32 bound){
//		//GPIO端口设置
//		GPIO_InitTypeDef GPIO_InitStructure;
//		USART_InitTypeDef USART_InitStructure;
//		NVIC_InitTypeDef NVIC_InitStructure;

//		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE); //使能GPIOG时钟
//		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);//使能USART6时钟

//		//串口6对应引脚复用映射
//		GPIO_PinAFConfig(GPIOG,GPIO_PinSource9,GPIO_AF_USART6); //GPIOA9复用为USART6
//		GPIO_PinAFConfig(GPIOG,GPIO_PinSource14,GPIO_AF_USART6); //GPIOA10复用为USART6

//		//USART6端口配置
//		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_14; //GPIOG9与GPIOG14
//		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
//		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;        //速度50MHz
//		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
//		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
//		GPIO_Init(GPIOG,&GPIO_InitStructure); //初始化GPIOG9与GPIOG14

//		//USART6 初始化设置
//		USART_InitStructure.USART_BaudRate = bound;//波特率设置
//		USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
//		USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
//		USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
//		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
//		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;        //收发模式
//		USART_Init(USART6, &USART_InitStructure); //初始化串口6

//		USART_Cmd(USART6, ENABLE);  //使能串口6 

//		USART_ClearFlag(USART6, USART_FLAG_TC);

//		USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);//开启相关中断

//		//USART6 NVIC 配置
//		NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;//串口6中断通道
//		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
//		NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;                //子优先级3
//		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                        //IRQ通道使能
//		NVIC_Init(&NVIC_InitStructure);        //根据指定的参数初始化VIC寄存器、
// 
//        
//}
u8 kzsb = 0;
void USART6_DMA_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
	      GPIO_InitTypeDef GPIO_InitStructure;
        USART_InitTypeDef USART_InitStructure;
				NVIC_InitTypeDef NVIC_InitStructure;
        /* -------------- Enable Module Clock Source ----------------------------*/
				RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG | RCC_AHB1Periph_DMA2,ENABLE); //使能GPIOG时钟
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);//使能USART6时钟
				
        RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART6, ENABLE);
        RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART6, DISABLE);

          //串口6对应引脚复用映射
				GPIO_PinAFConfig(GPIOG,GPIO_PinSource9,GPIO_AF_USART6); //G9复用为USART6
				GPIO_PinAFConfig(GPIOG,GPIO_PinSource14,GPIO_AF_USART6); //G14复用为USART6                                                        /* -------------- Configure GPIO ---------------------------------------*/
        {


						//USART6端口配置
								GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_14; //GPIOG9与GPIOG14
								GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
								GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;        //速度50MHz
								GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
								GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
								GPIO_Init(GPIOG,&GPIO_InitStructure); //初始化GPIOG9与GPIOG14			
								//USART_DeInit(USART1);

                USART_InitStructure.USART_BaudRate = 115200;
                USART_InitStructure.USART_WordLength = USART_WordLength_8b;
                USART_InitStructure.USART_StopBits = USART_StopBits_1;
                USART_InitStructure.USART_Parity = USART_Parity_No;
                USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
                USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
                USART_Init(USART6, &USART_InitStructure);

                USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);

                USART_ClearFlag(USART6, USART_FLAG_IDLE);
                USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);

                USART_Cmd(USART6, ENABLE);
        }

        /* -------------- Configure NVIC ---------------------------------------*/
        {
                
                NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
                NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = USART6_NVIC;
                NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
                NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
                NVIC_Init(&NVIC_InitStructure);
        }
				
        //DMA2 stream1 ch5     !!!!!!! P206 of the datasheet
        /* -------------- Configure DMA -----------------------------------------*/
        {
                DMA_InitTypeDef DMA_InitStructure;
                DMA_DeInit(DMA2_Stream1);

                DMA_InitStructure.DMA_Channel = DMA_Channel_5;
                DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART6->DR);
                DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rx1_buf;
                DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
                DMA_InitStructure.DMA_BufferSize = 512;
                DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
                DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
                DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
                DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
                DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
                DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
                DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
                DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
                DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
                DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
                DMA_Init(DMA2_Stream1, &DMA_InitStructure);
                DMA_DoubleBufferModeConfig(DMA2_Stream1, (uint32_t)rx2_buf, DMA_Memory_0);
                DMA_DoubleBufferModeCmd(DMA2_Stream1, ENABLE);
                DMA_Cmd(DMA2_Stream1, DISABLE); //Add a disable
                DMA_Cmd(DMA2_Stream1, ENABLE);
        }
}
