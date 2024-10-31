/**
  *********************************************************************
  * @file      chassisR_task.c/h
  * @brief     该任务控制右半部分的电机，分别是两个DM4310和一个DM6215，这三个电机挂载在can1总线上
	*						 从底盘上往下看，右上角的DM4310发送id为6、接收id为3，
	*						 右下角的DM4310发送id为8、接收id为4，
	*						 右边DM轮毂电机发送id为1、接收id为0。
  * @note       
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================  
  @endverbatim
  *********************************************************************
  */
  
#include "chassisR_task.h"
#include "fdcan.h"
#include "cmsis_os.h"

// LQR控制增益矩阵，用于稳定控制
float LQR_K_R[12]={ 
   -2.1954,   -0.2044,  -0.8826,   -1.3245,   1.2784,  0.1112,
    2.5538,   0.2718,  1.5728,    2.2893,   12.1973,   0.4578};

// 三次多项式拟合系数，用于右腿关节角度与力矩关系的拟合计算
float Poly_Coefficient[12][4]={
    {-88.3079710751263,  68.9068310796955, -30.0003802287502, -0.197774178106864},
    {1.52414598059982, -1.09343038036609, -2.82688593867512, 0.0281973842051861},
    {-21.8700750609220, 12.7421672466682, -2.58779676995074, -0.750848242540331},
    {-29.3271263750692, 17.6067629457167, -4.23484645974363, -1.08976980288501},
    {-147.771748892911, 94.0665615939814, -22.5139626085997, 2.53224765312440},
    {-6.72857056332562, 4.46216499907277, -1.14328671767927, 0.176775242328476},
    {-43.1495035855057, 35.1427890165576, -12.7617044245710, 3.36940801739176},
    {4.14428184617563, -2.56933858132474, 0.479050092243477, 0.248175261724735},
    {-229.898177881547, 144.949258291255, -33.9196587052128, 3.44291788865558},
    {-329.509693153293, 207.219295206736, -48.3799707459102, 4.952560575479143},
    {380.589246401548, -223.660017597103, 46.1696952431268, 9.82308882692083},
    {26.1010681824798, -15.7241310513153, 3.39175554658673, 0.278568898146322}
};

vmc_leg_t right; // 右腿虚拟主控模型

extern INS_t INS; // 惯性导航系统传感器数据
extern vmc_leg_t left; // 左腿虚拟主控模型
chassis_t chassis_move; // 底盘控制参数结构体

PidTypeDef LegR_Pid; // 右腿长度PID控制
PidTypeDef Tp_Pid; // 防劈叉补偿PID控制
PidTypeDef Turn_Pid; // 转向PID控制
PidTypeDef RollR_Pid; // 横滚PID控制

uint32_t CHASSR_TIME=1; // 右侧底盘控制时间间隔

/**
 * @brief 右侧底盘控制任务，主要负责初始化和实时控制右侧电机和腿部
 */
void ChassisR_task(void)
{
	while(INS.ins_flag==0)
	{// 等待加速度数据收敛
	  osDelay(1);	
	}

    // 初始化右侧底盘和腿部
    ChassisR_init(&chassis_move, &right, &LegR_Pid);
    Pensation_init(&Tp_Pid, &Turn_Pid); // 初始化补偿PID
	roll_pid_init(&RollR_Pid); // 初始化横滚PID

	while(1)
	{	
		chassisR_feedback_update(&chassis_move, &right, &INS); // 更新传感器和反馈数据
	    chassisR_control_loop(&chassis_move, &right, &INS, LQR_K_R, &LegR_Pid); // 控制计算
		
		// 根据启动标志位控制电机输出
		if(chassis_move.start_flag==1)	
		{
			mit_ctrl(&hfdcan1, chassis_move.joint_motor[1].para.id, 0.0f, 0.0f, 0.0f, 0.0f, right.torque_set[1]);
			osDelay(CHASSR_TIME);
			mit_ctrl(&hfdcan1, chassis_move.joint_motor[0].para.id, 0.0f, 0.0f, 0.0f, 0.0f, right.torque_set[0]);
			osDelay(CHASSR_TIME);
			mit_ctrl2(&hfdcan1, chassis_move.wheel_motor[0].para.id, 0.0f, 0.0f, 0.0f, 0.0f, chassis_move.wheel_motor[0].wheel_T);
			osDelay(CHASSR_TIME);
		}
		else if(chassis_move.start_flag==0)	
		{
			mit_ctrl(&hfdcan1, chassis_move.joint_motor[1].para.id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
			osDelay(CHASSR_TIME);
			mit_ctrl(&hfdcan1, chassis_move.joint_motor[0].para.id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
			osDelay(CHASSR_TIME);
			mit_ctrl2(&hfdcan1, chassis_move.wheel_motor[0].para.id, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
			osDelay(CHASSR_TIME);
		}
	}
}

/**
 * @brief 右侧底盘初始化，设置电机模式和控制参数
 */
void ChassisR_init(chassis_t *chassis, vmc_leg_t *vmc, PidTypeDef *legr)
{
  const static float legr_pid[3] = {LEG_PID_KP, LEG_PID_KI, LEG_PID_KD};

	joint_motor_init(&chassis->joint_motor[0], 6, MIT_MODE); // 初始化发送id为6的电机
	joint_motor_init(&chassis->joint_motor[1], 8, MIT_MODE); // 初始化发送id为8的电机
	
	wheel_motor_init(&chassis->wheel_motor[0], 1, MIT_MODE); // 初始化发送id为1的轮毂电机
	
	VMC_init(vmc); // 初始化虚拟主控
	PID_init(legr, PID_POSITION, legr_pid, LEG_PID_MAX_OUT, LEG_PID_MAX_IOUT); // 初始化腿长PID

	for(int j = 0; j < 10; j++)
	{
	  enable_motor_mode(&hfdcan1, chassis->joint_motor[1].para.id, chassis->joint_motor[1].mode); // 启动右下角关节电机
	  osDelay(1);
	}
	for(int j = 0; j < 10; j++)
	{
	  enable_motor_mode(&hfdcan1, chassis->joint_motor[0].para.id, chassis->joint_motor[0].mode); // 启动右上角关节电机
	  osDelay(1);
	}
	for(int j = 0; j < 10; j++)
	{
    enable_motor_mode(&hfdcan1, chassis->wheel_motor[0].para.id, chassis->wheel_motor[0].mode); // 启动轮毂电机
	  osDelay(1);
	}
}

/**
 * @brief 补偿控制PID初始化，用于防劈叉和转向补偿
 */
void Pensation_init(PidTypeDef *Tp, PidTypeDef *turn)
{
	const static float tp_pid[3] = {TP_PID_KP, TP_PID_KI, TP_PID_KD};
	const static float turn_pid[3] = {TURN_PID_KP, TURN_PID_KI, TURN_PID_KD};
	
	PID_init(Tp, PID_POSITION, tp_pid, TP_PID_MAX_OUT, TP_PID_MAX_IOUT);
	PID_init(turn, PID_POSITION, turn_pid, TURN_PID_MAX_OUT, TURN_PID_MAX_IOUT);
}

/**
 * @brief 横滚控制PID初始化
 */
void roll_pid_init(PidTypeDef *roll_pid)
{
	const static float roll[3] = {ROLL_PID_KP, ROLL_PID_KI, ROLL_PID_KD};
	
	PID_init(roll_pid, PID_POSITION, roll, ROLL_PID_MAX_OUT, ROLL_PID_MAX_IOUT);
}

/**
 * @brief 更新右侧底盘反馈数据，获取当前角度、角速度等信息
 */
void chassisR_feedback_update(chassis_t *chassis, vmc_leg_t *vmc, INS_t *ins)
{
  // 更新右腿角度 phi1 和 phi4
  vmc->phi1 = pi / 2.0f + chassis->joint_motor[0].para.pos;
	vmc->phi4 = pi / 2.0f + chassis->joint_motor[1].para.pos;
	
	// 更新右侧底盘的俯仰角、俯仰角速度、偏航角和滚转角
	chassis->myPithR = ins->Pitch;
	chassis->myPithGyroR = ins->Gyro[0];
	chassis->total_yaw = ins->YawTotalAngle;
	chassis->roll = ins->Roll;
	
	// 计算横向角度误差（期望值为0）
	chassis->theta_err = 0.0f - (vmc->theta + left.theta);
	
	// 判断是否完成倒地自起，根据 Pitch 角度范围
	if (ins->Pitch < (3.1415926f / 6.0f) && ins->Pitch > (-3.1415926f / 6.0f))
	{
		chassis->recover_flag = 0; // 倒地自起完成标志位清零
	}
}

uint32_t count_roll = 0; 
uint8_t right_flag = 0; // 右腿离地标志位
extern uint8_t left_flag; // 左腿离地标志位

/**
 * @brief 控制循环，计算控制输出并将其应用于右腿和底盘
 */
void chassisR_control_loop(chassis_t *chassis, vmc_leg_t *vmcr, INS_t *ins, float *LQR_K, PidTypeDef *leg)
{
	// 计算右腿关节角度和角速度，用于LQR控制
	VMC_calc_1_right(vmcr, ins, ((float)CHASSR_TIME) * 3.0f / 1000.0f);

	for (int i = 0; i < 12; i++)
	{
		// 使用多项式拟合系数，基于腿长L0计算LQR控制增益
		LQR_K[i] = LQR_K_calc(&Poly_Coefficient[i][0], vmcr->L0);
	}
	
	// 转向力矩的PID控制，基于总偏航角和角速度
	chassis->turn_T = Turn_Pid.Kp * (chassis->turn_set - chassis->total_yaw) - Turn_Pid.Kd * ins->Gyro[2];

	// 防劈叉补偿力矩的PID计算，基于theta_err
	chassis->leg_tp = PID_Calc(&Tp_Pid, chassis->theta_err, 0.0f);
	
	// 计算轮毂电机的输出力矩，使用LQR控制器的线性组合
	chassis->wheel_motor[0].wheel_T = 
		(LQR_K[0] * (vmcr->theta - 0.0f) +
		 LQR_K[1] * (vmcr->d_theta - 0.0f) +
		 LQR_K[2] * (chassis->x_filter - chassis->x_set) +
		 LQR_K[3] * (chassis->v_filter - chassis->v_set) +
		 LQR_K[4] * (chassis->myPithR - 0.0f) +
		 LQR_K[5] * (chassis->myPithGyroR - 0.0f));

	// 右边髋关节力矩输出
	vmcr->Tp = (LQR_K[6] * (vmcr->theta - 0.0f) +
	            LQR_K[7] * (vmcr->d_theta - 0.0f) +
	            LQR_K[8] * (chassis->x_filter - chassis->x_set) +
	            LQR_K[9] * (chassis->v_filter - chassis->v_set) +
	            LQR_K[10] * (chassis->myPithR - 0.0f) +
	            LQR_K[11] * (chassis->myPithGyroR - 0.0f));
	
	// 轮毂电机力矩减去转向力矩
	chassis->wheel_motor[0].wheel_T -= chassis->turn_T;
	mySaturate(&chassis->wheel_motor[0].wheel_T, -1.0f, 1.0f); // 限幅
	
	// 髋关节输出力矩增加防劈叉力矩
	vmcr->Tp += chassis->leg_tp;

	// 横滚力矩的PID控制，基于当前滚转角度
	chassis->now_roll_set = PID_Calc(&RollR_Pid, chassis->roll, chassis->roll_set);

	// 跳跃状态更新
	jump_loop_r(chassis, vmcr, leg);
	
	// 右腿离地检测
	right_flag = ground_detectionR(vmcr, ins);
	
	if (chassis->recover_flag == 0)
	{ // 当倒地自起完成时，检查左右腿是否同时离地
		if (right_flag == 1 && left_flag == 1 && vmcr->leg_flag == 0)
		{
			// 当两腿同时离地且未被控制时，清零相关参数
			chassis->wheel_motor[0].wheel_T = 0.0f;
			vmcr->Tp = LQR_K[6] * (vmcr->theta - 0.0f) + LQR_K[7] * (vmcr->d_theta - 0.0f);
			chassis->x_filter = 0.0f;
			chassis->x_set = chassis->x_filter;
			chassis->turn_set = chassis->total_yaw;
			vmcr->Tp += chassis->leg_tp;
		}
		else
		{ // 否则将leg_flag置为0
			vmcr->leg_flag = 0;
		}
	}
	else if (chassis->recover_flag == 1)
	{
		vmcr->Tp = 0.0f; // 若倒地自起未完成，清零力矩
	}
	
	// 限幅髋关节输出力矩
	mySaturate(&vmcr->F0, -100.0f, 100.0f);
	
	// 计算最终的关节输出力矩
	VMC_calc_2(vmcr);

	// 限幅右腿关节力矩
    mySaturate(&vmcr->torque_set[1], -3.0f, 3.0f);
	mySaturate(&vmcr->torque_set[0], -3.0f, 3.0f);
}

/**
 * @brief 将输入值限制在指定范围内
 */
void mySaturate(float *in, float min, float max)
{
  if (*in < min)
  {
    *in = min;
  }
  else if (*in > max)
  {
    *in = max;
  }
}

/**
 * @brief 跳跃控制逻辑，更新跳跃状态和跳跃力矩
 */
void jump_loop_r(chassis_t *chassis, vmc_leg_t *vmcr, PidTypeDef *leg)
{
	if (chassis->jump_flag == 1)
	{
		if (chassis->jump_status_r == 0)
		{
			vmcr->F0 = Mg / arm_cos_f32(vmcr->theta) + PID_Calc(leg, vmcr->L0, 0.07f); // 前馈+PD控制
			if (vmcr->L0 < 0.1f)
			{
				chassis->jump_time_r++;
			}
			if (chassis->jump_time_r >= 10 && chassis->jump_time_l >= 10)
			{
				chassis->jump_time_r = 0;
				chassis->jump_status_r = 1;
				chassis->jump_time_l = 0;
				chassis->jump_status_l = 1;
			}
		}
		else if (chassis->jump_status_r == 1)
		{
			vmcr->F0 = Mg / arm_cos_f32(vmcr->theta) + PID_Calc(leg, vmcr->L0, 0.4f);
			if (vmcr->L0 > 0.16f)
			{
				chassis->jump_time_r++;
			}
			if (chassis->jump_time_r >= 10 && chassis->jump_time_l >= 10)
			{
				chassis->jump_time_r = 0;
				chassis->jump_status_r = 2;
				chassis->jump_time_l = 0;
				chassis->jump_status_l = 2;
			}
		}
		else if (chassis->jump_status_r == 2)
		{
			vmcr->F0 = Mg / arm_cos_f32(vmcr->theta) + PID_Calc(leg, vmcr->L0, chassis->leg_right_set);
			if (vmcr->L0 < (chassis->leg_right_set + 0.01f))
			{
				chassis->jump_time_r++;
			}
			if (chassis->jump_time_r >= 10 && chassis->jump_time_l >= 10)
			{
				chassis->jump_time_r = 0;
				chassis->jump_status_r = 3;
				chassis->jump_time_l = 0;
				chassis->jump_status_l = 3;
			}
		}
		else
		{
			vmcr->F0 = Mg / arm_cos_f32(vmcr->theta) + PID_Calc(leg, vmcr->L0, chassis->leg_right_set);
		}

		if (chassis->jump_status_r == 3 && chassis->jump_status_l == 3)
		{
			// 完成跳跃后重置跳跃状态
			chassis->jump_flag = 0;
			chassis->jump_time_r = 0;
			chassis->jump_status_r = 0;
			chassis->jump_time_l = 0;
			chassis->jump_status_l = 0;
		}
	}
	else
	{
		// 若未跳跃，使用前馈+PD控制力矩
		vmcr->F0 = Mg / arm_cos_f32(vmcr->theta) + PID_Calc(leg, vmcr->L0, chassis->leg_right_set) - chassis->now_roll_set;
	}
}
