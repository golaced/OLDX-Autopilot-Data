/* GCS_Protocol.c file
��д�ߣ�lisn3188
��ַ��www.chiplab7.com
����E-mail��lisn3188@163.com
���뻷����MDK-Lite  Version: 4.23
����ʱ��: 2012-11-12
���ԣ� ���������ڵ���ʵ���ҵ�[Captain �ɿذ�]����ɲ���

ռ��STM32 ��Դ��
1. ʹ��UART1��������

------------------------------------
 */


//���� ������
typedef union {
	float  value;
	unsigned char byte[4];
} f_bytes;

//���� ������
typedef union {
	int16_t  value;
	unsigned char byte[2];
} i_bytes;


/**************************ʵ�ֺ���********************************************
*����ԭ��:		void Initial_UART1(u32 baudrate)
*��������:		��ʼ��UART
���������
		u32 baudrate   ����RS232���ڵĲ�����
���������û��	
*******************************************************************************/
void Initial_UART1(u32 baudrate)
{
 	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* ʹ�� UART1 ģ���ʱ��  ʹ�� UART1��Ӧ�����Ŷ˿�PA��ʱ��*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

  	 /* ����UART1 �ķ�������
	 ����PA9 Ϊ�������  ˢ��Ƶ��50MHz
	  */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);    
  	/* 
	  ����UART1 �Ľ�������
	  ����PA10Ϊ�������� 
	 */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);
	  
	/* 
	  UART1������:
	  1.������Ϊ���ó���ָ�������� baudrate;
	  2. 8λ����			  USART_WordLength_8b;
	  3.һ��ֹͣλ			  USART_StopBits_1;
	  4. ����żЧ��			  USART_Parity_No ;
	  5.��ʹ��Ӳ��������	  USART_HardwareFlowControl_None;
	  6.ʹ�ܷ��ͺͽ��չ���	  USART_Mode_Rx | USART_Mode_Tx;
	 */
	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	//Ӧ�����õ�UART1
	USART_Init(USART1, &USART_InitStructure); 
	//����UART1
  	USART_Cmd(USART1, ENABLE);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void UART1_Put_Char(unsigned char DataToSend)
*��������:		RS232����һ���ֽ�
���������
		unsigned char DataToSend   Ҫ���͵��ֽ�����
���������û��	
*******************************************************************************/
void UART1_Put_Char(unsigned char DataToSend)
{
	//��Ҫ���͵��ֽ�д��UART1�ķ��ͻ�����
	USART_SendData(USART1, (unsigned char) DataToSend);
	//�ȴ��������
  	while (!(USART1->SR & USART_FLAG_TXE));
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:		UART1_Report_PWMin
*��������:		�����ջ������PWMֵ�����͵�PC
���������
int16_t PWM_Input_CH1,  PWM���ͨ��1��ֵ����λus   800-2200us
...
...
...
int16_t PWM_Input_CH6,  PWM���ͨ��6��ֵ����λus   800-2200us
���������û��	
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

	ctemp=PWM_Input_CH1>>8;	//ͨ��1 ��8λ
	UART1_Put_Char(ctemp);
	temp+=ctemp;
	ctemp=PWM_Input_CH1;  //ͨ��1 ��8λ
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

	UART1_Put_Char(temp%256); //У��
	UART1_Put_Char(0xaa);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		UART1_Report_PWMout
*��������:		��PWM��� ���͵�PC
���������
int16_t PWM_Output_CH1,  PWM���ͨ��1��ֵ����λus   800-2200us
...
...
...
int16_t PWM_Output_CH8,  PWM���ͨ��8��ֵ����λus   800-2200us
���������û��	
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

	data = PWM_Output_CH1;//���ͨ��1 
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

	UART1_Put_Char(temp%256); //У��
	UART1_Put_Char(0xaa);
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:		UART1_Report_PID
*��������:		����λ������ PID �������ĵ�ǰֵ��Ŀ�껹�п������
���������
float target,   Ŀ��ֵ����λ�Զ���
float current   ��ǰֵ
float pidcon    PID�Ŀ��������ֵ
���������û��	
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

	UART1_Put_Char(temp%256); //У��
	UART1_Put_Char(0xaa);
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:		UART1_ReportSysteminfo
*��������:		����λ�����Ϳ�������ǰ��ϵͳ��Ϣ
���������
int8_t mode,  ����ģʽ��
			  0x01 �ֶ�ģʽ	
			  0x02  ƽ��ģʽ
			  0x03  �������
int16_t bat_vol, ������ص�ѹֵ����λ0.1V  
int16_t sevor_vol, �����Դ��ѹֵ����λ 0.1V
int8_t error_code, �������   �Լ����塣0-255
int16_t I2C_error_count, I2C���ߴ���Ĵ��������ϵ�ʱ��ʼ��
int16_t	system_time  ϵͳʱ�䣬��λS �롣���ϵ�ʱ��ʼ��ʱ 
���������û��	
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
	data_to_send[2] = (14);  // ֡�ֽ�����������ֽڿ�ʼ��֡���������ֽڣ������� A5 5A
	data_to_send[3] = (0xA6);  // ֡��ʶ  ֡�ؼ���

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

/**************************ʵ�ֺ���********************************************
*����ԭ��:		UART1_ReportPosition
*��������:		����λ�����Ϳ�������ǰ��λ����Ϣ  Ҳ����GPS����
���������
int32_t lon,    ����ֵ����λ0.0001�ȡ������͵�ֵΪ 1234567  ��ʾ 123.4567��
int32_t lat,    γ��ֵ����λ0.0001�ȡ������͵�ֵΪ 123456  ��ʾ  12.4567��
int16_t hight,  GPS���θ߶�ֵ����λ0.1�ס�������ֵΪ 1623  ��ʾ��ǰ����Ϊ 162.3��
int8_t  STnum,  ���������������� 0 ��ʾû�ж�λ��
int16_t heading, GPS����ֵ����λ0.1�ȡ�������ֵΪ 125ʱ����ʾ12.5�ȡ�
int16_t	speed    GPS�ٶȣ���λ0.1��/S  �����͵�ֵΪ 255ʱ����ʾ 25.5M/S
���������û��	
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
	data_to_send[2] = (19); // ֡�ֽ�����������ֽڿ�ʼ��֡���������ֽڣ������� A5 5A
	data_to_send[3] = (0xA3); // ֡��ʶ  ֡�ؼ���

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
	data_to_send[20] = 0xaa;   //֡�����ֽ�

	for( i = 0;i<21;i++){
			UART1_Put_Char(data_to_send[i]);
			}

}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		UART1_ReportTarget
*��������:		����λ�����Ϳ�������ǰ��Ŀ�꺽����Ϣ  
���������
int32_t lon,    ����ֵ����λ0.0001�ȡ������͵�ֵΪ 1234567  ��ʾ 123.4567��
int32_t lat,    γ��ֵ����λ0.0001�ȡ������͵�ֵΪ 123456  ��ʾ  12.4567��
int16_t hight,  GPS���θ߶�ֵ����λ0.1�ס�������ֵΪ 1623  ��ʾ��ǰ����Ϊ 162.3��
int16_t Distance, ��Ŀ���ľ��� ��λ 0.1�� 
int16_t Xspeed,   ͨ�������������������X���ϵĿ���������λ0.1�� 
int16_t	Yspeed    ͨ�����������������������Y���ϵĿ���������λ0.1�� 
���������û��	
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
	data_to_send[2] = (20);	  // ֡�ֽ�����������ֽڿ�ʼ��֡���������ֽڣ������� A5 5A
	data_to_send[3] = (0xA5); // ֡��ʶ  ֡�ؼ���

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
			ctemp += data_to_send[i];  //�ۼӺ�У��
			}
	data_to_send[20] = ctemp;
	data_to_send[21] = 0xaa; //֡�����ֽ� 

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
