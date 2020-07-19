#include "uart8.h"
#include <string.h>
#include <stdarg.h>
#include "LX_IMU.h"

#define UART_REC_LEN  			8  	//定义最大接收字节数 200
#define EN_UART8_RX 			1		//使能（1）/禁止（0）串口1接收  
#define UART8_Waiting 0
#define UART8_Receiving 1
#define UART8_Success 2 
#define UART8_Failed  3

u8 Tx8Buffer[256];
u8 Tx8Counter = 0;
u8 count8 = 0;

void LX_IMU_UART8_Configuration(u32 bound)
{
		//GPIO端口设置
		GPIO_InitTypeDef GPIO_InitStructure;
		USART_InitTypeDef USART_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE); //使能GPIOG时钟
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART8,ENABLE);//使能UART8时钟

		//串口8对应引脚复用映射
		GPIO_PinAFConfig(GPIOE,GPIO_PinSource0,GPIO_AF_UART8);
		GPIO_PinAFConfig(GPIOE,GPIO_PinSource1,GPIO_AF_UART8);

		//UART8端口配置
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;        //速度50MHz
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
		GPIO_Init(GPIOE,&GPIO_InitStructure); 

		//UART8 初始化设置
		USART_InitStructure.USART_BaudRate = bound;//波特率设置
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
		USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
		USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;        //收发模式
		USART_Init(UART8, &USART_InitStructure); //初始化串口8

		USART_Cmd(UART8, ENABLE);  //使能串口8

		USART_ClearFlag(UART8, USART_FLAG_TC);

		USART_ITConfig(UART8, USART_IT_RXNE, ENABLE);//开启相关中断

		//UART8 NVIC 配置
		NVIC_InitStructure.NVIC_IRQChannel = UART8_IRQn;//串口8中断通道
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = UART8_NVIC ;//抢占优先级3
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;                //子优先级3
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                        //IRQ通道使能
		NVIC_Init(&NVIC_InitStructure);        //根据指定的参数初始化VIC寄存器、

}

void UART8_IRQHandler(void)                        //串口6中断服务程序
{		
		u8 com_data;

		if ( UART8->SR & USART_SR_ORE ) //ORE中断
    {
        com_data = UART5->DR;
    }		
    //接收中断
    if ( USART_GetITStatus ( UART8, USART_IT_RXNE ) )
    {
        USART_ClearITPendingBit ( UART8, USART_IT_RXNE ); //清除中断标志
				//
        com_data = UART8->DR;
				//
				ANO_DT_LX_Data_Receive_Prepare(com_data);
				Usart_SendByte(UART8,com_data);
			
    }
//    //发送（进入移位）中断
//    if ( USART_GetITStatus ( UART8, USART_IT_TXE ) )
//    {
//        UART8->DR = Tx8Buffer[Tx8Counter++]; //写DR清除中断标志
//        if ( Tx8Counter == count8 )
//        {
//            UART8->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
//        }
//    }		
		
} 

void Uart8_Send ( unsigned char *DataToSend , u8 data_num )
{
    u8 i;
    for ( i = 0; i < data_num; i++ )
    {
        Tx8Buffer[count8++] = * ( DataToSend + i );
    }

    if ( ! ( UART8->CR1 & USART_CR1_TXEIE ) )
    {
        USART_ITConfig ( UART8, USART_IT_TXE, ENABLE ); //打开发送中断
    }

}

/*****************  发送一个字节 **********************/
void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch)
{
	/* 发送一个字节数据到USART */
	USART_SendData(pUSARTx,ch);
		
	/* 等待发送数据寄存器为空 */
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
}

/****************** 发送8位的数组 ************************/
void Usart_SendArray( USART_TypeDef * pUSARTx, uint8_t *array, uint16_t num)
{
  uint8_t i;
	
	for(i=0; i<num; i++)
  {
	    /* 发送一个字节数据到USART */
	    Usart_SendByte(pUSARTx,array[i]);	
  
  }
	/* 等待发送完成 */
	while(USART_GetFlagStatus(pUSARTx,USART_FLAG_TC)==RESET);
}

/*****************  发送字符串 **********************/
void Usart_SendString( USART_TypeDef * pUSARTx, char *str)
{
	unsigned int k=0;
  do 
  {
      Usart_SendByte( pUSARTx, *(str + k) );
      k++;
  } while(*(str + k)!='\0');
  
  /* 等待发送完成 */
  while(USART_GetFlagStatus(pUSARTx,USART_FLAG_TC)==RESET)
  {}
}

/*****************  发送一个16位数 **********************/
void Usart_SendHalfWord( USART_TypeDef * pUSARTx, uint16_t ch)
{
	uint8_t temp_h, temp_l;
	
	/* 取出高八位 */
	temp_h = (ch&0XFF00)>>8;
	/* 取出低八位 */
	temp_l = ch&0XFF;
	
	/* 发送高八位 */
	USART_SendData(pUSARTx,temp_h);	
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);
	
	/* 发送低八位 */
	USART_SendData(pUSARTx,temp_l);	
	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
}
