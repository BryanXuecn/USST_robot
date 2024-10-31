/**
  *********************************************************************
  * @file      ps2_task.c/h
  * @brief     该任务是读取并处理ps2手柄传来的遥控数据，
	*            将遥控数据转化为期望的速度、期望的转角、期望的腿长等
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

extern FDCAN_HandleTypeDef hfdcan1;  // 确保CAN总线句柄被正确声明
extern Joint_Motor_t motor;  // 确保电机结构体被正确初始化


ps2data_t ps2data; // 用于存储PS2手柄数据的结构体

uint16_t Handkey;	// 临时存储按键值
uint8_t Comd[2] = {0x01, 0x42}; // 手柄数据请求命令
uint8_t Data[9] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 存储手柄返回数据
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
}; // 按键对应的掩码值

extern chassis_t chassis_move; // 底盘控制参数结构体
extern INS_t INS; // 惯性导航系统传感器数据
uint32_t PS2_TIME = 10; // PS2手柄任务周期为10ms

/**
 * @brief PS2手柄任务，负责读取和处理PS2手柄的数据
 */
void pstwo_task(void)
{		
	PS2_SetInit(); // 初始化手柄配置

   while(1)
	 {
		 if(Data[1] != 0x73) // 如果数据异常，重新初始化手柄
		 {
		  PS2_SetInit();
		 }

	   PS2_data_read(&ps2data); // 读取手柄数据
		 PS2_data_process(&ps2data, &chassis_move, (float)PS2_TIME / 1000.0f); // 处理手柄数据，更新期望速度、转角等
	   osDelay(PS2_TIME); // 等待任务周期
	 }
}

/**
 * @brief 向手柄发送命令
 * @param CMD 要发送的命令
 */
void PS2_Cmd(uint8_t CMD)
{
	volatile uint16_t ref = 0x01;
	Data[1] = 0;
	for(ref = 0x01; ref < 0x0100; ref <<= 1)
	{
		// 设置命令位
		if(ref & CMD)
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET); // 输出高电平
		else 
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET); // 输出低电平

		// 产生时钟脉冲
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
		DWT_Delay(0.000005f); // 延时
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
		DWT_Delay(0.000005f);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);

		// 读取返回数据位
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))
			Data[1] = ref | Data[1];
	}
	DWT_Delay(0.000016f); // 延时确保稳定
}

uint8_t reve_flag = 0; // 按键响应标志位

/**
 * @brief 读取PS2手柄数据，包括按键和摇杆数据
 */
void PS2_data_read(ps2data_t *data)
{
  // 读取按键键值
	data->key = PS2_DataKey(); 

  // 读取左摇杆X轴方向数据	
	data->lx = PS2_AnologData(PSS_LX);

  // 读取左摇杆Y轴方向数据	
	data->ly = PS2_AnologData(PSS_LY);

  // 读取右摇杆X轴方向数据  
	data->rx = PS2_AnologData(PSS_RX);

  // 读取右摇杆Y轴方向数据  
	data->ry = PS2_AnologData(PSS_RY);

	// 根据右摇杆的Y和X轴位置校正，防止数据跳变
	if((data->ry <= 255 && data->ry > 192) || (data->ry < 64 && data->ry >= 0))
	{
	  data->rx = 127;
	}
	if((data->rx <= 255 && data->rx > 192) || (data->rx < 64 && data->rx >= 0))
	{
	  data->ry = 128;
	}
}

extern vmc_leg_t right; // 右腿虚拟主控模型			
extern vmc_leg_t left;	// 左腿虚拟主控模型	
float acc_test = 0.005f; // 加速度测试变量

/**
 * @brief 处理手柄数据，更新底盘的速度、转角等期望值
 */
void PS2_data_process(ps2data_t *data, chassis_t *chassis, float dt)
{   
	// 检测Start键，启动或停止底盘
	if(data->last_key != 4 && data->key == 4 && chassis->start_flag == 0) 
	{
		chassis->start_flag = 1; // 启动底盘
		if(chassis->recover_flag == 0
			&& ((chassis->myPithR < (-3.1415926f / 4.0f) && chassis->myPithR > (-3.1415926f / 2.0f))
		  || (chassis->myPithR > (3.1415926f / 4.0f) && chassis->myPithR < (3.1415926f / 2.0f))))
		{
		  chassis->recover_flag = 1; // 需要倒地自起
		}
	}
	else if(data->last_key != 4 && data->key == 4 && chassis->start_flag == 1) 
	{
		chassis->start_flag = 0; // 停止底盘
		chassis->recover_flag = 0;
	}
	
	data->last_key = data->key;
	
	if(chassis->start_flag == 1)
	{ // 底盘已启动
		int button_control_active = 0;
		// 更新目标速度，基于按键状态判断
		if (data->key == PSB_PAD_UP) // PSB_PAD_UP 按键按下
		{
			chassis->target_v = 0.5f; // 设置目标速度为正值，表示向前
			button_control_active = 1; // 启用按键控制
		}
		else if (data->key == PSB_PAD_DOWN) // PSB_PAD_DOWN 按键按下
		{
			chassis->target_v = -0.5f; // 设置目标速度为负值，表示向后
			button_control_active = 1; // 启用按键控制
		}
		else
		{
			chassis->target_v = 0.0f; // 没有按下时停止移动
		}

		// 左右移动控制，通过控制 `turn_set` 实现
        if (data->key == PSB_PAD_RIGHT) // PSB_PAD_RIGHT 按键按下
        {
            chassis->turn_set = 0.2f; // 设置右转弧度
            button_control_active = 1; // 启用按键控制
        }
        if (data->key == PSB_PAD_LEFT) // PSB_PAD_LEFT 按键按下
        {
            chassis->turn_set = -0.2f; // 设置左转弧度
            button_control_active = 1; // 启用按键控制
        }
		if (data->key == PSB_GREEN)
			{
				// 调用移动函数使电机前进到指定距离
				float TARGET_DISTANCE = 1.0f; // 例如，目标距离设为 1 米
   				 move_to_distance(&hfdcan1, &motor, TARGET_DISTANCE);
   				 button_control_active = 1; // 启用按键控制
			}
        else if (!button_control_active) 
        {
            // 当按键没有被按下时，允许摇杆控制前后速度和转角
            chassis->target_v = ((float)(data->ry - 128)) * (-0.008f); // 摇杆Y轴控制前后速度
            chassis->turn_set += ((float)(data->rx - 127)) * (-0.00025f); // 摇杆X轴控制左右转角
        }

		
			
			
	// 	// 当按键没有被按下时，允许摇杆Y轴控制速度
	// if (!button_control_active) 
	// {
	// 	chassis->target_v = ((float)(data->ry - 128)) * (-0.008f); // 根据摇杆Y轴设置目标速度
	// }
		slope_following(&chassis->target_v, &chassis->v_set, 0.005f);	// 坡度跟随

		// 更新期望位移和转角
		chassis->x_set += chassis->v_set * dt;
		chassis->turn_set += (data->rx - 127) * (-0.00025f); // 右摇杆X轴控制转角
	  			
		// 计算腿长变化，左摇杆Y轴控制
		chassis->leg_set += ((float)(data->ly - 128)) * (-0.000015f);
		chassis->roll_target = ((float)(data->lx - 127)) * (0.0025f); // 左摇杆X轴控制横滚目标

		slope_following(&chassis->roll_target, &chassis->roll_set, 0.0075f); // 横滚跟随

		// 检测跳跃按键
		jump_key(chassis, data);

		// 设置左右腿的目标长度
		chassis->leg_left_set = chassis->leg_set;
		chassis->leg_right_set = chassis->leg_set;

		// 限制腿长在0.065m到0.18m之间
		mySaturate(&chassis->leg_left_set, 0.065f, 0.18f);
		mySaturate(&chassis->leg_right_set, 0.065f, 0.18f);
		

		if(fabsf(chassis->last_leg_left_set - chassis->leg_left_set) > 0.0001f || fabsf(chassis->last_leg_right_set - chassis->leg_right_set) > 0.0001f)
		{ // 当手动控制腿长变化时，更新标志
			right.leg_flag = 1;	// 标记腿长正在主动伸缩
			left.leg_flag = 1;
		}
		
		chassis->last_leg_set = chassis->leg_set;
		chassis->last_leg_left_set = chassis->leg_left_set;
		chassis->last_leg_right_set = chassis->leg_right_set;
	} 
	else if(chassis->start_flag == 0)
	{ // 底盘关闭状态
	  chassis->v_set = 0.0f; // 清零速度
		chassis->x_set = chassis->x_filter; // 保存当前位置
	  chassis->turn_set = chassis->total_yaw; // 保存当前角度
	  chassis->leg_set = 0.08f; // 恢复腿长至初始长度
	}
}

/**
 * @brief 检测跳跃按键是否按下，控制跳跃标志位
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
				chassis->jump_leg = chassis->leg_set; // 保存当前腿长以便跳跃恢复
			}
		}
	}
	else
	{
		chassis->count_key = 0; // 未按下跳跃键时重置计数
	}
}

/**
 * @brief 检查手柄是否为红灯模式
 * @return 0 表示红灯模式, 非0 表示其他模式
 */
uint8_t PS2_RedLight(void)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
	PS2_Cmd(Comd[0]); // 开始命令
	PS2_Cmd(Comd[1]); // 请求数据
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET);
	if(Data[1] == 0X73) 
		return 0;
	else 
		return 1;
}

// 读取手柄数据
void PS2_ReadData(void)
{
	volatile uint8_t byte = 0;
	volatile uint16_t ref = 0x01;
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET); // CS_L
	PS2_Cmd(Comd[0]); // 开始命令
	PS2_Cmd(Comd[1]); // 请求数据
	for(byte = 2; byte < 9; byte++) // 开始接受数据
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

/* 以下为各配置、读取和操作函数的具体实现 */
/**
 * @brief 读取并处理手柄按键数据，仅处理按键部分
 * @return 按下的按键索引，返回0表示没有按下任何按键
 */
uint8_t PS2_DataKey()
{
	uint8_t index;

	PS2_ClearData(); // 清除数据缓冲区
	PS2_ReadData();  // 读取手柄数据

	// 组合Data[3]和Data[4]以得到16个按键的状态，按下为0，未按下为1
	Handkey = (Data[4] << 8) | Data[3]; 
	for(index = 0; index < 16; index++)
	{	    
		if((Handkey & (1 << (MASK[index] - 1))) == 0)
			return index + 1; // 返回按键索引
	}
	return 0; // 没有任何按键按下
}

/**
 * @brief 获取一个摇杆的模拟值，范围为0~256
 * @param button 指定读取的摇杆或按键
 * @return 返回对应的模拟值
 */
uint8_t PS2_AnologData(uint8_t button)
{
	return Data[button]; // 返回指定按键/摇杆的模拟值
}

/**
 * @brief 清除数据缓冲区，将Data数组清零
 */
void PS2_ClearData()
{
	for(uint8_t a = 0; a < 9; a++)
		Data[a] = 0x00;
}

/**
 * @brief 手柄震动控制函数
 * @param motor1 控制右侧小震动电机：0x00关闭，其他开启
 * @param motor2 控制左侧大震动电机：0x40~0xFF，值越大震动越强
 */
void PS2_Vibration(uint8_t motor1, uint8_t motor2)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET); // CS_L
	DWT_Delay(0.000016f);
	PS2_Cmd(0x01); // 开始命令
	PS2_Cmd(0x42); // 请求数据
	PS2_Cmd(0X00);
	PS2_Cmd(motor1); // 控制右侧电机
	PS2_Cmd(motor2); // 控制左侧电机
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET); // CS_H
	DWT_Delay(0.000016f);
}

/**
 * @brief 短轮询命令，用于快速检查手柄状态
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
 * @brief 进入配置模式，设置手柄工作模式
 */
void PS2_EnterConfing(void)
{
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET); // CS_L
	DWT_Delay(0.000016f); 
	PS2_Cmd(0x01);  
	PS2_Cmd(0x43);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x01); // 进入配置模式
	PS2_Cmd(0x00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET); // CS_H
	DWT_Delay(0.000016f); 
}

/**
 * @brief 设置手柄为模拟模式
 */
void PS2_TurnOnAnalogMode(void)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET); // CS_L
	PS2_Cmd(0x01);  
	PS2_Cmd(0x44);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x01); // 设置为模拟模式
	PS2_Cmd(0x03); // 0x03表示模式锁定，无法通过按键更改
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	PS2_Cmd(0X00);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET); // CS_H
	DWT_Delay(0.000016f); 
}

/**
 * @brief 开启手柄震动模式
 */
void PS2_VibrationMode(void)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET); // CS_L
	DWT_Delay(0.000016f); 
	PS2_Cmd(0x01);  
	PS2_Cmd(0x4D);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0X01); // 启用震动模式
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET); // CS_H
	DWT_Delay(0.000016f); 
}

/**
 * @brief 退出配置模式并保存设置
 */
void PS2_ExitConfing(void)
{
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET); // CS_L
	DWT_Delay(0.000016f); 
	PS2_Cmd(0x01);  
	PS2_Cmd(0x43);  
	PS2_Cmd(0X00);
	PS2_Cmd(0x00);
	PS2_Cmd(0x5A); // 退出并保存配置
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	PS2_Cmd(0x5A);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET); // CS_H
	DWT_Delay(0.000016f); 
}

/**
 * @brief PS2手柄配置初始化，进入配置、设置模拟模式、保存并退出
 */
void PS2_SetInit(void)
{
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_ShortPoll();
	PS2_EnterConfing();		// 进入配置模式
	PS2_TurnOnAnalogMode();	// 设置为“红绿灯”模拟模式
	// PS2_VibrationMode();	// 可选：开启震动模式
	PS2_ExitConfing();		// 完成并保存配置
}



