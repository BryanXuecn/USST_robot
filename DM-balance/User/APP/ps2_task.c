/**
  *********************************************************************
  * @file      ps2_task.c/h
  * @brief     �������Ƕ�ȡ������ps2�ֱ�������ң�����ݣ�
	*            ��ң������ת��Ϊ�������ٶȡ�������ת�ǡ��������ȳ���
  * @note       
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================  
  @endverbatim
  *********************************************************************
  */
	
#include "ps2_task.h"
#include "cmsis_os.h"
#include "user_lib.h"
#include "dm4310_drv.h"
#include "fdcan.h"

extern FDCAN_HandleTypeDef hfdcan1;  // ȷ��CAN���߾������ȷ����
extern Joint_Motor_t motor;  // ȷ������ṹ�屻��ȷ��ʼ��


ps2data_t ps2data; // ���ڴ洢PS2�ֱ����ݵĽṹ��

uint16_t Handkey;	// ��ʱ�洢����ֵ
uint8_t Comd[2] = {0x01, 0x42}; // �ֱ�������������
uint8_t Data[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // �洢�ֱ���������
uint16_t MASK[] = {
    PSB_SELECT, 
	PSB_L3, 
	PSB_R3, 
	PSB_START, 
	PSB_PAD_UP, 
	PSB_PAD_RIGHT,
    PSB_PAD_DOWN, 
	PSB_PAD_LEFT, 
	PSB_L2, 
	PSB_R2, 
	PSB_L1, 
	PSB_R1,
    PSB_GREEN, 
	PSB_RED, 
	PSB_BLUE, 
	PSB_PINK
}; // ������Ӧ������ֵ

extern chassis_t chassis_move; // ���̿��Ʋ����ṹ��
extern INS_t INS; // ���Ե���ϵͳ����������
uint32_t PS2_TIME = 10; // PS2�ֱ���������Ϊ10ms

/**
 * @brief PS2�ֱ����񣬸����ȡ�ʹ���PS2�ֱ�������
 */
void pstwo_task(void)
{		
	PS2_SetInit(); // ��ʼ���ֱ�����

   while(1)
	 {
		 if(Data[1] != 0x73) // ��������쳣�����³�ʼ���ֱ�
		 {
		  PS2_SetInit();
		 }

	   PS2_data_read(&ps2data); // ��ȡ�ֱ�����
		 PS2_data_process(&ps2data, &chassis_move, (float)PS2_TIME / 1000.0f); // �����ֱ����ݣ����������ٶȡ�ת�ǵ�
	   osDelay(PS2_TIME); // �ȴ���������
	 }
}

/**
 * @brief ���ֱ���������
 * @param CMD Ҫ���͵�����
 */
void PS2_Cmd(uint8_t CMD)
{
	volatile uint16_t ref = 0x01;
	Data[1] = 0;
	for(ref = 0x01; ref < 0x0100; ref <<= 1)
	{
		// ��������λ
		if(ref & CMD)
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); // ����ߵ�ƽ
		else 
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET); // ����͵�ƽ

		// ����ʱ������
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
		DWT_Delay(0.000005f); // ��ʱ
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
		DWT_Delay(0.000005f);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);

		// ��ȡ��������λ
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))
			Data[1] = ref | Data[1];
	}
	DWT_Delay(0.000016f); // ��ʱȷ���ȶ�
}

uint8_t reve_flag = 0; // ������Ӧ��־λ

/**
 * @brief ��ȡPS2�ֱ����ݣ�����������ҡ������
 */
void PS2_data_read(ps2data_t *data)
{
  // ��ȡ������ֵ
	data->key = PS2_DataKey(); 

  // ��ȡ��ҡ��X�᷽������	
	data->lx = PS2_AnologData(PSS_LX);

  // ��ȡ��ҡ��Y�᷽������	
	data->ly = PS2_AnologData(PSS_LY);

  // ��ȡ��ҡ��X�᷽������  
	data->rx = PS2_AnologData(PSS_RX);

  // ��ȡ��ҡ��Y�᷽������  
	data->ry = PS2_AnologData(PSS_RY);

	// ������ҡ�˵�Y��X��λ��У������ֹ��������
	if((data->ry <= 255 && data->ry > 192) || (data->ry < 64 && data->ry >= 0))
	{
	  data->rx = 127;
	}
	if((data->rx <= 255 && data->rx > 192) || (data->rx < 64 && data->rx >= 0))
	{
	  data->ry = 128;
	}
}

extern vmc_leg_t right; // ������������ģ��			
extern vmc_leg_t left;	// ������������ģ��	
float acc_test = 0.005f; // ���ٶȲ��Ա���

/**
 * @brief �����ֱ����ݣ����µ��̵��ٶȡ�ת�ǵ�����ֵ
 */
void PS2_data_process(ps2data_t *data, chassis_t *chassis, float dt)
{   
	// ���Start����������ֹͣ����
	if(data->last_key != 4 && data->key == 4 && chassis->start_flag == 0) 
	{
		chassis->start_flag = 1; // ��������
		if(chassis->recover_flag == 0
			&& ((chassis->myPithR < (-3.1415926f / 4.0f) && chassis->myPithR > (-3.1415926f / 2.0f))
		  || (chassis->myPithR > (3.1415926f / 4.0f) && chassis->myPithR < (3.1415926f / 2.0f))))
		{
		  chassis->recover_flag = 1; // ��Ҫ��������
		}
	}
	else if(data->last_key != 4 && data->key == 4 && chassis->start_flag == 1) 
	{
		chassis->start_flag = 0; // ֹͣ����
		chassis->recover_flag = 0;
	}
	
	data->last_key = data->key;
	
	if(chassis->start_flag == 1)
	{ // ����������
		int button_control_active = 0;
		// ����Ŀ���ٶȣ����ڰ���״̬�ж�
		if (data->key == PSB_PAD_UP) // PSB_PAD_UP ��������
		{
			chassis->target_v = 0.5f; // ����Ŀ���ٶ�Ϊ��ֵ����ʾ��ǰ
			button_control_active = 1; // ���ð�������
		}
		else if (data->key == PSB_PAD_DOWN) // PSB_PAD_DOWN ��������
		{
			chassis->target_v = -0.5f; // ����Ŀ���ٶ�Ϊ��ֵ����ʾ���
			button_control_active = 1; // ���ð�������
		}
		else
		{
			chassis->target_v = 0.0f; // û�а���ʱֹͣ�ƶ�
		}

		// �����ƶ����ƣ�ͨ������ `turn_set` ʵ��
        if (data->key == PSB_PAD_RIGHT) // PSB_PAD_RIGHT ��������
        {
            chassis->turn_set = 0.2f; // ������ת����
            button_control_active = 1; // ���ð�������
        }
        if (data->key == PSB_PAD_LEFT) // PSB_PAD_LEFT ��������
        {
            chassis->turn_set = -0.2f; // ������ת����
            button_control_active = 1; // ���ð�������
        }
		if (data->key == PSB_GREEN)
			{
				// �����ƶ�����ʹ���ǰ����ָ������
				float TARGET_DISTANCE = 1.0f; // ���磬Ŀ�������Ϊ 1 ��
   				 move_to_distance(&hfdcan1, &motor, TARGET_DISTANCE);
   				 button_control_active = 1; // ���ð�������
			}
        else if (!button_control_active) 
        {
            // ������û�б�����ʱ������ҡ�˿���ǰ���ٶȺ�ת��
            chassis->target_v = ((float)(data->ry - 128)) * (-0.008f); // ҡ��Y�����ǰ���ٶ�
            chassis->turn_set += ((float)(data->rx - 127)) * (-0.00025f); // ҡ��X���������ת��
        }

		
			
			
	// 	// ������û�б�����ʱ������ҡ��Y������ٶ�
	// if (!button_control_active) 
	// {
	// 	chassis->target_v = ((float)(data->ry - 128)) * (-0.008f); // ����ҡ��Y������Ŀ���ٶ�
	// }
		slope_following(&chassis->target_v, &chassis->v_set, 0.005f);	// �¶ȸ���

		// ��������λ�ƺ�ת��
		chassis->x_set += chassis->v_set * dt;
		chassis->turn_set += (data->rx - 127) * (-0.00025f); // ��ҡ��X�����ת��
	  			
		// �����ȳ��仯����ҡ��Y�����
		chassis->leg_set += ((float)(data->ly - 128)) * (-0.000015f);
		chassis->roll_target = ((float)(data->lx - 127)) * (0.0025f); // ��ҡ��X����ƺ��Ŀ��

		slope_following(&chassis->roll_target, &chassis->roll_set, 0.0075f); // �������

		// �����Ծ����
		jump_key(chassis, data);

		// ���������ȵ�Ŀ�곤��
		chassis->leg_left_set = chassis->leg_set;
		chassis->leg_right_set = chassis->leg_set;

		// �����ȳ���0.065m��0.18m֮��
		mySaturate(&chassis->leg_left_set, 0.065f, 0.18f);
		mySaturate(&chassis->leg_right_set, 0.065f, 0.18f);
		

		if(fabsf(chassis->last_leg_left_set - chassis->leg_left_set) > 0.0001f || fabsf(chassis->last_leg_right_set - chassis->leg_right_set) > 0.0001f)
		{ // ���ֶ������ȳ��仯ʱ�����±�־
			right.leg_flag = 1;	// ����ȳ�������������
			left.leg_flag = 1;
		}
		
		chassis->last_leg_set = chassis->leg_set;
		chassis->last_leg_left_set = chassis->leg_left_set;
		chassis->last_leg_right_set = chassis->leg_right_set;
	} 
	else if(chassis->start_flag == 0)
	{ // ���̹ر�״̬
	  chassis->v_set = 0.0f; // �����ٶ�
		chassis->x_set = chassis->x_filter; // ���浱ǰλ��
	  chassis->turn_set = chassis->total_yaw; // ���浱ǰ�Ƕ�
	  chassis->leg_set = 0.08f; // �ָ��ȳ�����ʼ����
	}
}

/**
 * @brief �����Ծ�����Ƿ��£�������Ծ��־λ
 */
void jump_key (chassis_t *chassis, ps2data_t *data)
{
	if(data->key == 12)
	{
		if(++chassis->count_key > 10)
		{
			if(chassis->jump_flag == 0)
			{
				chassis->jump_flag = 1;
				chassis->jump_leg = chassis->leg_set; // ���浱ǰ�ȳ��Ա���Ծ�ָ�
			}
		}
	}
	else
	{
		chassis->count_key = 0; // δ������Ծ��ʱ���ü���
	}
}

/**
 * @brief ����ֱ��Ƿ�Ϊ���ģʽ
 * @return 0 ��ʾ���ģʽ, ��0 ��ʾ����ģʽ
 */
uint8_t PS2_RedLight(void)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
	PS2_Cmd(Comd[0]); // ��ʼ����
	PS2_Cmd(Comd[1]); // ��������
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
	if(Data[1] == 0X73) 
		return 0;
	else 
		return 1;
}

// ��ȡ�ֱ�����
void PS2_ReadData(void)
{
	volatile uint8_t byte = 0;
	volatile uint16_t ref = 0x01;
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET); // CS_L
	PS2_Cmd(Comd[0]); // ��ʼ����
	PS2_Cmd(Comd[1]); // ��������
	for(byte = 2; byte < 9; byte++) // ��ʼ��������
	{
		for(ref = 0x01; ref < 0x100; ref <<= 1)
		{
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET); // CLK_H
			DWT_Delay(0.000005f);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET); // CLK_L
			DWT_Delay(0.000005f);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET); // CLK_H
			if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)) // DI
		      Data[byte] = ref | Data[byte];
		}
        DWT_Delay(0.000016f); 
	}
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET); // CS_H
}

/* ����Ϊ�����á���ȡ�Ͳ��������ľ���ʵ�� */
/**
 * @brief ��ȡ�������ֱ��������ݣ�������������
 * @return ���µİ�������������0��ʾû�а����κΰ���
 */
uint8_t PS2_DataKey()
{
	uint8_t index;

	PS2_ClearData(); // ������ݻ�����
	PS2_ReadData();  // ��ȡ�ֱ�����

	// ���Data[3]��Data[4]�Եõ�16��������״̬������Ϊ0��δ����Ϊ1
	Handkey = (Data[4] << 8) | Data[3]; 
	for(index = 0; index < 16; index++)
	{	    
		if((Handkey & (1 << (MASK[index] - 1))) == 0)
			return index + 1; // ���ذ�������
	}
	return 0; // û���κΰ�������
}

/**
 * @brief ��ȡһ��ҡ�˵�ģ��ֵ����ΧΪ0~256
 * @param button ָ����ȡ��ҡ�˻򰴼�
 * @return ���ض�Ӧ��ģ��ֵ
 */
uint8_t PS2_AnologData(uint8_t button)
{
	return Data[button]; // ����ָ������/ҡ�˵�ģ��ֵ
}

/**
 * @brief ������ݻ���������Data��������
 */
void PS2_ClearData()
{
	for(uint8_t a = 0; a < 9; a++)
		Data[a] = 0x00;
}

/**
 * @brief �ֱ��𶯿��ƺ���
 * @param motor1 �����Ҳ�С�𶯵����0x00�رգ���������
 * @param motor2 ���������𶯵����0x40~0xFF��ֵԽ����Խǿ
 */
void PS2_Vibration(uint8_t motor1, uint8_t motor2)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET); // CS_L
	DWT_Delay(0.000016f);
	PS2_Cmd(0x01); // ��ʼ����
	PS2_Cmd(0x42); // ��������
	PS2_Cmd(0X00);
	PS2_Cmd(motor1); // �����Ҳ���
	PS2_Cmd(motor2); // ���������
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET); // CS_H
	DWT_Delay(0.000016f);
}

/**
 * @brief ����ѯ������ڿ��ټ���ֱ�״̬
 */
void PS2_ShortPoll(void)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET); // CS_L
	DWT_Delay(0.000016f); 
	PS2_Cmd(0x01);  
	PS2_Cmd(0x42);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0x00);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET); // CS_H
	DWT_Delay(0.000016f); 	
}

/**
 * @brief ��������ģʽ�������ֱ�����ģʽ
 */
void PS2_EnterConfing(void)
{
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET); // CS_L
	DWT_Delay(0.000016f); 
	PS2_Cmd(0x01);  
	PS2_Cmd(0x43);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x01); // ��������ģʽ
	PS2_Cmd(0x00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET); // CS_H
	DWT_Delay(0.000016f); 
}

/**
 * @brief �����ֱ�Ϊģ��ģʽ
 */
void PS2_TurnOnAnalogMode(void)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET); // CS_L
	PS2_Cmd(0x01);  
	PS2_Cmd(0x44);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x01); // ����Ϊģ��ģʽ
	PS2_Cmd(0x03); // 0x03��ʾģʽ�������޷�ͨ����������
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET); // CS_H
	DWT_Delay(0.000016f); 
}

/**
 * @brief �����ֱ���ģʽ
 */
void PS2_VibrationMode(void)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET); // CS_L
	DWT_Delay(0.000016f); 
	PS2_Cmd(0x01);  
	PS2_Cmd(0x4D);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0X01); // ������ģʽ
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET); // CS_H
	DWT_Delay(0.000016f); 
}

/**
 * @brief �˳�����ģʽ����������
 */
void PS2_ExitConfing(void)
{
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET); // CS_L
	DWT_Delay(0.000016f); 
	PS2_Cmd(0x01);  
	PS2_Cmd(0x43);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0x5A); // �˳�����������
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET); // CS_H
	DWT_Delay(0.000016f); 
}

/**
 * @brief PS2�ֱ����ó�ʼ�����������á�����ģ��ģʽ�����沢�˳�
 */
void PS2_SetInit(void)
{
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_EnterConfing();		// ��������ģʽ
	PS2_TurnOnAnalogMode();	// ����Ϊ�����̵ơ�ģ��ģʽ
	// PS2_VibrationMode();	// ��ѡ��������ģʽ
	PS2_ExitConfing();		// ��ɲ���������
}



