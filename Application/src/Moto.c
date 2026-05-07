#include "Moto.h"
#include "USER_Moto.h"
#include "bsp_can.h"
#include "super_cap.h"
#include "RLS.h"
#include "arm_math.h"
#include "USER_B2B.h"

ChassisMotor_t chassis = {0}; //所有底盘电机结构体 包括四个轮电机 四个舵电机 一个大yaw电机
GimbalMotor_t gimbal = {0};

float sumPowTorque =0;
float sumPowSpeed = 0;
float powerPredict;
float effetive_power;
float toque_coefficient=1;
float measure_power = 0.0;

float p_des = 0;
float p_des_yaw = 0;


void Supercap_Update(SuperCap* this, uint8_t* rxdata)
{
	  // 状态变量 
    this->receive_data.power_ctrl_mode = rxdata[0];
    // 状态变量 dcdcmode.autoMode.stage
    this->receive_data.automode_stage = rxdata[1];
    // 电容组电压 CapVoltage
    this->receive_data.cap_voltage = (int16_t)(rxdata[2] | (rxdata[3] << 8));    
	//电感电流
	this->receive_data.L_current = (int16_t)(rxdata[4]|rxdata[5]<<8);
    // 总线电压 Buspower
    this->receive_data.bus_power = (int16_t)(rxdata[6] | (rxdata[7] << 8));
}

void Motor_ClearErr(FDCAN_HandleTypeDef *hfdcan, uint8_t id, uint8_t state)
{
    if(state == 0)
        enable_motor_mode(hfdcan, id, MIT_MODE);
    else if(state != 1)
        clear_err(hfdcan, id, MIT_MODE);
}

/**底盘**/

void Chassis_InitPID()
{
	for (uint8_t i = 0; i < 4; i++)
	{
		PID_Init(&chassis.M3508[i].speedPID, 10, 0, 6, 8000, 16000);
		PD_Init(&chassis.J4310[i].PD_Ctrl,8,0.25,7);
	}
	//		PID_Init(&chassis.move.buffer_pid, 2.5, 0.1, 0, 10, 40);		// 缓冲能量pid 暂时不加
		PID_SetDeadzone(&chassis.M3508[0].speedPID, 0);
		PID_SetDeadzone(&chassis.M3508[1].speedPID, 0);
		PID_SetDeadzone(&chassis.M3508[2].speedPID, 0);
		PID_SetDeadzone(&chassis.M3508[3].speedPID, 0);
}

void Task_CANMotors_Callback()
{
//		Motor_CalcAngle_J4310(&chassis.J4310_yaw);  //积累多圈角度
		Motor_CalcAngle_J4310(&chassis.J4310[0]);
		Motor_CalcAngle_J4310(&chassis.J4310[1]);
		Motor_CalcAngle_J4310(&chassis.J4310[2]);
		Motor_CalcAngle_J4310(&chassis.J4310[3]);
		for (uint8_t i = 0; i < 4; i++)
		{
			PD_ParallelCalc(&chassis.J4310[i].PD_Ctrl,chassis.J4310[i].targetTurnAngle * PI / 180.0f ,0,chassis.J4310[i].totalAngle * PI / 180.0f,chassis.J4310[i].para.vel);//仿mid PD控制电机位置
			PID_SingleCalc(&chassis.M3508[i].speedPID,chassis.M3508[i].targetSpeed,chassis.M3508[i].speed);
		}
		PowerCtrl();
		mit_ctrl(&hfdcan2,0x03,0.0f,0.0f,0.0f,0.0f,chassis.J4310[2].PD_Ctrl.outputTorque);//
		mit_ctrl(&hfdcan3,0x05,0,gimbal.J4310_yaw.targetTorque,0,2,0);
		mit_ctrl(&hfdcan2,0x04,0.0f,0.0f,0.0f,0.0f,chassis.J4310[3].PD_Ctrl.outputTorque);//
		mit_ctrl(&hfdcan2,0x01,0.0f,0.0f,0.0f,0.0f,chassis.J4310[0].PD_Ctrl.outputTorque);//
		mit_ctrl(&hfdcan2,0x02,0.0f,0.0f,0.0f,0.0f,chassis.J4310[1].PD_Ctrl.outputTorque);//

//		mit_ctrl(&hfdcan3,0x05,0.0f,0.0f,0.0f,0.0f,gimbal.J4310_yaw.targetTorque);//
//		mit_ctrl(&hfdcan3,0x05,0.0f,0.0f,0.0f,0.0f,0);//
//		mit_ctrl(&hfdcan2,0x01,0.0f,0.0f,0.0f,0.0f,0);//
//		mit_ctrl(&hfdcan2,0x02,0.0f,0.0f,0.0f,0.0f,0);//
//		mit_ctrl(&hfdcan2,0x03,0.0f,0.0f,0.0f,0.0f,0);//
//		mit_ctrl(&hfdcan2,0x04,0.0f,0.0f,0.0f,0.0f,0);//
		

    USER_CAN_SetMotorCurrent(&hfdcan1, 0x200, chassis.M3508[0].speedPID.output,chassis.M3508[1].speedPID.output,chassis.M3508[2].speedPID.output,chassis.M3508[3].speedPID.output);
//		measure_power = cap.receive_data.bus_power*0.01;
} 

void Task_ClearError_Callback()
{
    Motor_ClearErr(&hfdcan2, 0x01, chassis.J4310[0].para.state);
    Motor_ClearErr(&hfdcan2, 0x02, chassis.J4310[1].para.state);
    Motor_ClearErr(&hfdcan2, 0x03, chassis.J4310[2].para.state);
	Motor_ClearErr(&hfdcan3, 0x05, gimbal.J4310_yaw.para.state);
    Motor_ClearErr(&hfdcan2, 0x04, chassis.J4310[3].para.state);
}

/************************freertos任务*******************  *********/
void OS_MotorCallback(void const * argument)
{
	osDelay(1500);
	Chassis_InitPID();
	enable_motor_mode(&hfdcan2,0x01,MIT_MODE);
	Motor_StartCalcAngle_J4310(&chassis.J4310[0]);
	enable_motor_mode(&hfdcan2,0x02,MIT_MODE);
	Motor_StartCalcAngle_J4310(&chassis.J4310[1]);
	enable_motor_mode(&hfdcan2,0x03,MIT_MODE);
	Motor_StartCalcAngle_J4310(&chassis.J4310[2]);
	enable_motor_mode(&hfdcan3,0x05,MIT_MODE);
	Motor_StartCalcAngle_J4310(&chassis.J4310[3]);
	enable_motor_mode(&hfdcan2,0x04,MIT_MODE);
	PowerCtralInit();
//		PowerControl_AutoUpdateParamInit();
    for(;;)
    {	
		if(rs485_isvalid){
			Task_CANMotors_Callback();
			rs485_isvalid = 0;
		}
		static uint32_t clear_error_tick = 0;
		if (HAL_GetTick() - clear_error_tick >= 50)
		{
			clear_error_tick = HAL_GetTick();
			Task_ClearError_Callback();
		}
		osDelay(1);
    }
}
