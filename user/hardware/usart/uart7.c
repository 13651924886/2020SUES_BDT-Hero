#include "uart7.h"

u8 TxBuffer[256];
u8 TxCounter = 0;
u8 count = 0;

u8 Rx_Buf[256];	//串口接收缓存

void ANO_DT_UART7_Configuration(u32 bound)
{
		//GPIO端口设置
		GPIO_InitTypeDef GPIO_InitStructure;
		USART_InitTypeDef USART_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE); //使能GPIOE时钟
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART7,ENABLE);//使能UART7时钟

		//串口8对应引脚复用映射
		GPIO_PinAFConfig(GPIOE,GPIO_PinSource7,GPIO_AF_UART8);
		GPIO_PinAFConfig(GPIOE,GPIO_PinSource8,GPIO_AF_UART8);

		//UART8端口配置
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8;
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
		USART_Init(UART7, &USART_InitStructure); //初始化串口7

		USART_Cmd(UART7, ENABLE);  //使能串口7

		USART_ClearFlag(UART7, USART_FLAG_TC);

		USART_ITConfig(UART7, USART_IT_RXNE, ENABLE);//开启相关中断

		//UART8 NVIC 配置
		NVIC_InitStructure.NVIC_IRQChannel = UART7_IRQn;//串口7中断通道
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = UART7_NVIC ;//抢占优先级
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;                //子优先级
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                        //IRQ通道使能
		NVIC_Init(&NVIC_InitStructure);        //根据指定的参数初始化VIC寄存器、

}


void Uart7_IRQ ( void )
{
    u8 com_data;

    if ( UART7->SR & USART_SR_ORE ) //ORE中断
    {
        //com_data = USART2->DR;
			USART_ReceiveData(UART7);
    }

    //接收中断
    if ( USART_GetITStatus ( UART7, USART_IT_RXNE ) )
    {
        USART_ClearITPendingBit ( UART7, USART_IT_RXNE ); //清除中断标志

        com_data = UART7->DR;
        //ANO_DT_Data_Receive_Prepare ( com_data );
				
			//UART7_Put_Char(com_data);//灵活格式帧
				//ANODT_GetByte(com_data);	//参数读写与返回
		}
    //发送（进入移位）中断
    if ( USART_GetITStatus ( UART7, USART_IT_TXE ) )
    {

        UART7->DR = TxBuffer[TxCounter++]; //写DR清除中断标志
        if ( TxCounter == count )
        {
            UART7->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
        }


        //USART_ClearITPendingBit(USART2,USART_IT_TXE);
    }



}
static void UART7_Put_Char(unsigned char DataToSend)
{
	TxBuffer[count++] = DataToSend;
	USART_ITConfig(UART7, USART_IT_TXE, ENABLE);
}

void UART7_Put_Buf( unsigned char *DataToSend, u8 data_num )
{
	for(u8 i = 0;i<data_num;i++)
	{
		TxBuffer[count++] = *(DataToSend+i);
	}
	if(!(UART7->CR1 & USART_CR1_TXEIE))
		USART_ITConfig(UART7, USART_IT_TXE, ENABLE);
}

//void UART7_Send( unsigned char *DataToSend , u8 data_num )
//{
//    u8 i;
//    for ( i = 0; i < data_num; i++ )
//    {
//        TxBuffer[count++] = * ( DataToSend + i );
//    }

//    if ( ! ( UART7->CR1 & USART_CR1_TXEIE ) )
//    {
//        USART_ITConfig ( UART7, USART_IT_TXE, ENABLE ); //打开发送中断
//    }

//}
void UART7_IRQHandler(void)                        //串口7中断服务程序
{
		Uart7_IRQ();
}
