/* GCS_Protocol.c file
编写者：lisn3188
网址：www.chiplab7.com
作者E-mail：lisn3188@163.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2012-11-12
测试： 本程序已在第七实验室的[Captain 飞控板]上完成测试

占用STM32 资源：
1. 使用UART1发送数据

------------------------------------
 */


//浮点 联合体
typedef union {
	float  value;
	unsigned char byte[4];
} f_bytes;

//整数 联合体
typedef union {
	int16_t  value;
	unsigned char byte[2];
} i_bytes;


/**************************实现函数********************************************
*函数原型:		void Initial_UART1(u32 baudrate)
*功　　能:		初始化UART
输入参数：
		u32 baudrate   设置RS232串口的波特率
输出参数：没有	
*******************************************************************************/
void Initial_UART1(u32 baudrate)
{
 	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* 使能 UART1 模块的时钟  使能 UART1对应的引脚端口PA的时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

  	 /* 配置UART1 的发送引脚
	 配置PA9 为复用输出  刷新频率50MHz
	  */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);    
  	/* 
	  配置UART1 的接收引脚
	  配置PA10为浮地输入 
	 */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);
	  
	/* 
	  UART1的配置:
	  1.波特率为调用程序指定的输入 baudrate;
	  2. 8位数据			  USART_WordLength_8b;
	  3.一个停止位			  USART_StopBits_1;
	  4. 无奇偶效验			  USART_Parity_No ;
	  5.不使用硬件流控制	  USART_HardwareFlowControl_None;
	  6.使能发送和接收功能	  USART_Mode_Rx | USART_Mode_Tx;
	 */
	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	//应用配置到UART1
	USART_Init(USART1, &USART_InitStructure); 
	//启动UART1
  	USART_Cmd(USART1, ENABLE);
}

/**************************实现函数********************************************
*函数原型:		void UART1_Put_Char(unsigned char DataToSend)
*功　　能:		RS232发送一个字节
输入参数：
		unsigned char DataToSend   要发送的字节数据
输出参数：没有	
*******************************************************************************/
void UART1_Put_Char(unsigned char DataToSend)
{
	//将要发送的字节写到UART1的发送缓冲区
	USART_SendData(USART1, (unsigned char) DataToSend);
	//等待发送完成
  	while (!(USART1->SR & USART_FLAG_TXE));
}


/**************************实现函数********************************************
*函数原型:		UART1_Report_PWMin
*功　　能:		将接收机输入的PWM值，发送到PC
输入参数：
int16_t PWM_Input_CH1,  PWM输出通道1的值，单位us   800-2200us
...
...
...
int16_t PWM_Input_CH6,  PWM输出通道6的值，单位us   800-2200us
输出参数：没有	
*******************************************************************************/
void UART1_Report_PWMin(int16_t PWM_Input_CH1,int16_t PWM_Input_CH2,int16_t PWM_Input_CH3,
						int16_t PWM_Input_CH4,int16_t PWM_Input_CH5,int16_t PWM_Input_CH6)
{
 	unsigned int temp=0x12+16;
	//uint16_t  temp;
	char ctemp;
	UART1_Put_Char(0xa5);
	UART1_Put_Char(0x5a);
	UART1_Put_Char(16);
	UART1_Put_Char(0x12);

	ctemp=PWM_Input_CH1>>8;	//通道1 高8位
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=PWM_Input_CH1;  //通道1 低8位
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	ctemp=PWM_Input_CH2>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=PWM_Input_CH2;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	ctemp=PWM_Input_CH3>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=PWM_Input_CH3;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	ctemp=PWM_Input_CH4>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=PWM_Input_CH4;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	ctemp=PWM_Input_CH5>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=PWM_Input_CH5;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
//-------------------------
	ctemp=PWM_Input_CH6>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=PWM_Input_CH6;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	UART1_Put_Char(temp%256); //校验
	UART1_Put_Char(0xaa);
}

/**************************实现函数********************************************
*函数原型:		UART1_Report_PWMout
*功　　能:		将PWM输出 发送到PC
输入参数：
int16_t PWM_Output_CH1,  PWM输出通道1的值，单位us   800-2200us
...
...
...
int16_t PWM_Output_CH8,  PWM输出通道8的值，单位us   800-2200us
输出参数：没有	
*******************************************************************************/
void UART1_Report_PWMout(int16_t PWM_Output_CH1,int16_t PWM_Output_CH2,int16_t PWM_Output_CH3,
						 int16_t PWM_Output_CH4,int16_t PWM_Output_CH5,int16_t PWM_Output_CH6,
						 int16_t PWM_Output_CH7,int16_t PWM_Output_CH8)
{
 	unsigned int temp=0x13+20;
	uint16_t  data;
	char ctemp;
	UART1_Put_Char(0xa5);
	UART1_Put_Char(0x5a);
	UART1_Put_Char(20);
	UART1_Put_Char(0x13);

	data = PWM_Output_CH1;//输出通道1 
	ctemp=data>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=data;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	data = PWM_Output_CH2;
	ctemp=data>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=data;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	data = PWM_Output_CH3;
	ctemp=data>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=data;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	data = PWM_Output_CH4;
	ctemp=data>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=data;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	data = PWM_Output_CH5;
	ctemp=data>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=data;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	data = PWM_Output_CH6;
	ctemp=data>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=data;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	data = PWM_Output_CH7;
	ctemp=data>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=data;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	data = PWM_Output_CH8;
	ctemp=data>>8;
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=data;
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	UART1_Put_Char(temp%256); //校验
	UART1_Put_Char(0xaa);
}


/**************************实现函数********************************************
*函数原型:		UART1_Report_PID
*功　　能:		向上位机发送 PID 控制器的当前值和目标还有控制输出
输入参数：
float target,   目标值，单位自定义
float current   当前值
float pidcon    PID的控制器输出值
输出参数：没有	
*******************************************************************************/
void UART1_Report_PID(float target,float current,float pidcon)
{
 	unsigned int temp=0x15+16;
	f_bytes data;
	char ctemp;
	UART1_Put_Char(0xa5);
	UART1_Put_Char(0x5a);
	UART1_Put_Char(16);
	UART1_Put_Char(0x15);

	data.value = target;
	ctemp=data.byte[0];
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=data.byte[1];
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=data.byte[2];
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=data.byte[3];
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	data.value = current;
	ctemp=data.byte[0];
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=data.byte[1];
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=data.byte[2];
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=data.byte[3];
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	data.value = pidcon;
	ctemp=data.byte[0];
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=data.byte[1];
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=data.byte[2];
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=data.byte[3];
	UART1_Put_Char(ctemp);
	temp+=ctemp;

	UART1_Put_Char(temp%256); //校验
	UART1_Put_Char(0xaa);
}


/**************************实现函数********************************************
*函数原型:		UART1_ReportSysteminfo
*功　　能:		向上位机发送控制器当前的系统信息
输入参数：
int8_t mode,  工作模式。
			  0x01 手动模式	
			  0x02  平衡模式
			  0x03  定点飞行
int16_t bat_vol, 动力电池电压值，单位0.1V  
int16_t sevor_vol, 舵机电源电压值，单位 0.1V
int8_t error_code, 错误代码   自己定义。0-255
int16_t I2C_error_count, I2C总线错误的次数，从上电时开始计
int16_t	system_time  系统时间，单位S 秒。从上电时开始计时 
输出参数：没有	
*******************************************************************************/
void UART1_ReportSysteminfo(
						int8_t mode,
						int16_t bat_vol,
						int16_t sevor_vol,
						int8_t error_code,
						int16_t I2C_error_count,
						int16_t	system_time   //S
						){
	char ctemp = 0,i;
	char data_to_send[21];
	data_to_send[0] = (0xa5);
	data_to_send[1] = (0x5a);
	data_to_send[2] = (14);  // 帧字节数，从这个字节开始到帧结束的总字节，不包含 A5 5A
	data_to_send[3] = (0xA6);  // 帧标识  帧关键字

	data_to_send[4] = mode;
	data_to_send[5] = bat_vol >> 8;
	data_to_send[6] = bat_vol;
	data_to_send[7] = sevor_vol >> 8;
	data_to_send[8] = sevor_vol;
	data_to_send[9] = error_code;
	data_to_send[10] = I2C_error_count >> 8;
	data_to_send[11] = I2C_error_count;

	data_to_send[12] = system_time >> 8;
	data_to_send[13] = system_time;
	ctemp = 0;
	for( i = 2;i<14;i++){
			ctemp += data_to_send[i];
			}
	data_to_send[14] = ctemp;
	data_to_send[15] = 0xaa;

	for( i = 0;i<16;i++){
			UART1_Put_Char(data_to_send[i]);
			}
}

/**************************实现函数********************************************
*函数原型:		UART1_ReportPosition
*功　　能:		向上位机发送控制器当前的位置信息  也就是GPS数据
输入参数：
int32_t lon,    经度值，单位0.0001度。当传送的值为 1234567  表示 123.4567度
int32_t lat,    纬度值，单位0.0001度。当传送的值为 123456  表示  12.4567度
int16_t hight,  GPS海拔高度值，单位0.1米。当传送值为 1623  表示当前海拔为 162.3米
int8_t  STnum,  锁定的卫星数量， 0 表示没有定位、
int16_t heading, GPS航向值，单位0.1度。当传送值为 125时，表示12.5度。
int16_t	speed    GPS速度，单位0.1米/S  当传送的值为 255时，表示 25.5M/S
输出参数：没有	
*******************************************************************************/
void UART1_ReportPosition(
						int32_t lon,
						int32_t lat,
						int16_t hight,
						int8_t  STnum,
						int16_t heading,
						int16_t	speed
						){
	char ctemp = 0,i;
	char data_to_send[21];
	data_to_send[0] = (0xa5);
	data_to_send[1] = (0x5a);
	data_to_send[2] = (19); // 帧字节数，从这个字节开始到帧结束的总字节，不包含 A5 5A
	data_to_send[3] = (0xA3); // 帧标识  帧关键字

	data_to_send[4] = lon >> 24;
	data_to_send[5] = lon >> 16;
	data_to_send[6] = lon >> 8;
	data_to_send[7] = lon >> 0;

	data_to_send[8] = lat >> 24;
	data_to_send[9] = lat >> 16;
	data_to_send[10] = lat >> 8;
	data_to_send[11] = lat >> 0;

	data_to_send[12] = hight >> 8;
	data_to_send[13] = hight;

	data_to_send[14] = STnum;

	data_to_send[15] = heading >> 8;
	data_to_send[16] = heading;

	data_to_send[17] = speed >> 8;
	data_to_send[18] = speed;
	ctemp = 0;
	for( i = 2;i<19;i++){
			ctemp += data_to_send[i];
			}
	data_to_send[19] = ctemp;
	data_to_send[20] = 0xaa;   //帧结束字节

	for( i = 0;i<21;i++){
			UART1_Put_Char(data_to_send[i]);
			}

}

/**************************实现函数********************************************
*函数原型:		UART1_ReportTarget
*功　　能:		向上位机发送控制器当前的目标航点信息  
输入参数：
int32_t lon,    经度值，单位0.0001度。当传送的值为 1234567  表示 123.4567度
int32_t lat,    纬度值，单位0.0001度。当传送的值为 123456  表示  12.4567度
int16_t hight,  GPS海拔高度值，单位0.1米。当传送值为 1623  表示当前海拔为 162.3米
int16_t Distance, 与目标点的距离 单位 0.1米 
int16_t Xspeed,   通过运算后控制器输出，在X轴上的控制量，单位0.1度 
int16_t	Yspeed    通过运算后控制器输出，四轴在Y轴上的控制量，单位0.1度 
输出参数：没有	
*******************************************************************************/
void UART1_ReportTarget(
						int32_t lon,
						int32_t lat,
						int16_t hight,
						int16_t Distance,
						int16_t Xspeed,
						int16_t	Yspeed
						){
	char ctemp = 0,i;
	char data_to_send[22];
	data_to_send[0] = (0xa5);
	data_to_send[1] = (0x5a);
	data_to_send[2] = (20);	  // 帧字节数，从这个字节开始到帧结束的总字节，不包含 A5 5A
	data_to_send[3] = (0xA5); // 帧标识  帧关键字

	data_to_send[4] = lon >> 24;
	data_to_send[5] = lon >> 16;
	data_to_send[6] = lon >> 8;
	data_to_send[7] = lon >> 0;

	data_to_send[8] = lat >> 24;
	data_to_send[9] = lat >> 16;
	data_to_send[10] = lat >> 8;
	data_to_send[11] = lat >> 0;

	data_to_send[12] = hight >> 8;
	data_to_send[13] = hight;

	data_to_send[14] = Distance >> 8;
	data_to_send[15] = Distance;

	data_to_send[16] = Xspeed >> 8;
	data_to_send[17] = Xspeed;

	data_to_send[18] = Yspeed >> 8;
	data_to_send[19] = Yspeed;

	ctemp = 0;
	for( i = 2;i<20;i++){
			ctemp += data_to_send[i];  //累加和校验
			}
	data_to_send[20] = ctemp;
	data_to_send[21] = 0xaa; //帧结束字节 

	for( i = 0;i<22;i++){
			UART1_Put_Char(data_to_send[i]);
			}

}


//------------------------------------------------------
void USART1_IRQHandler(void)
{
  
  if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
  {   
    /* Write one byte to the transmit data register */
    USART_SendData(USART1, TxBuffer[TxCounter++]);                    

    /* Clear the USART1 transmit interrupt */
    USART_ClearITPendingBit(USART1, USART_IT_TXE); 

    if(TxCounter == count)
    {
      /* Disable the USART1 Transmit interrupt */
      USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
    }    
  }

}

//------------------End of File----------------------------
