#include "PowerCtrl.h"
#include "main.h"
#include "arm_math.h"
#include "math.h"
#include "RLS.h"
#include "moto.h"
#include "Judge.h"

uint16_t SET_WHEELSPEED_MAX = 8000;

ChassisPower turn_power;
ChassisPower whell_power;
void PowerInit4310()
{
	turn_power.toque_coefficient = 1.0;
	turn_power.paramVector[0][0] = 5.42751688e-04; // RPM
	turn_power.paramVector[1][0] = 4.630179e-01;  // Torque
	turn_power.paramVector[2][0] = 4.24808025;      // constant
	//experience points
	turn_power.transVector[0][0] = 1e-11;   // RPM
	turn_power.transVector[1][1] = 1e-11;   // Torque
	turn_power.transVector[2][2] = 2.5e-5;  // constant
	turn_power.moto_type = 4310;
	turn_power.UserPowerLimit = 100; 	//60
}

void PowerInit3508()
{
	whell_power.toque_coefficient =  2.4324e-6f;//(20/16384)*(0.3)*(187/3591)/9.55
	whell_power.paramVector[0][0] = 1.2158888e-7; // RPM
	whell_power.paramVector[1][0] = 1.5822148e-7;  // Torque
	whell_power.paramVector[2][0] = 3.04855824;      // constant
	//experience points
	whell_power.transVector[0][0] = 2.5e-15;   // RPM
	whell_power.transVector[1][1] = 2.5e-15;   // Torque
	whell_power.transVector[2][2] = 0.000025;  // constant
	whell_power.moto_type = 3508;
	whell_power.UserPowerLimit = 50;	//120
}

void PowerCtralInit()
{
	PowerInit4310();
	PowerControl_AutoUpdateParamInit(&turn_power);//动态拟合初始化
	PowerInit3508();
//	PowerControl_AutoUpdateParamInit(&chassis_power->Dj3508Power);
}

void TurnPowerCtral()
{
	turn_power.a = fmaxf(1e-7, turn_power.paramVector[1][0]);
	turn_power.k2 = fmaxf(turn_power.paramVector[0][0], 1e-7);
	turn_power.constant = fmaxf(0.7f, turn_power.paramVector[2][0]);
	
	//数据清零
	turn_power.SumPowerSpeed = 0;
	turn_power.SumPowerTorque = 0;
	turn_power.EffetivePower = 0;
	turn_power.InitialTotalPower = 0;
	turn_power.scaleFactor = 0;
	turn_power.PredictPower = 0;
	turn_power.output = 0;
	
	//拟合曲线用
	for (uint8_t i = 0;i<4;i++)
	{
			turn_power.SumPowerSpeed += chassis.J4310[i].para.vel*chassis.J4310[i].para.vel;
			turn_power.SumPowerTorque+= chassis.J4310[i].para.tor*chassis.J4310[i].para.tor;
			turn_power.EffetivePower += turn_power.toque_coefficient*chassis.J4310[i].para.tor*chassis.J4310[i].para.vel;
	}
	turn_power.PredictPower = turn_power.paramVector[1][0] * turn_power.SumPowerTorque + turn_power.paramVector[0][0] * turn_power.SumPowerSpeed + 4 * turn_power.paramVector[2][0] + turn_power.EffetivePower;
	//power.MeasurePower = cap.receive_data.bus_power / 100.0f;
		if (turn_power.EffetivePower > 10 && turn_power.MeasurePower>10 )//&& chassis.move.fastMode == 0)
	{
		PowerControl_AutoUpdateParam(turn_power.SumPowerSpeed / 4.0f, turn_power.SumPowerTorque / 4.0f, 1, (turn_power.MeasurePower-turn_power.EffetivePower) / 4.0f,turn_power);
	}
	//预测控制前功率
	for(uint8_t i=0;i<4;i++)
	{
		turn_power.InitialGivePower[i] = turn_power.toque_coefficient * chassis.J4310[i].PD_Ctrl.outputTorque * chassis.J4310[i].para.vel +
								turn_power.k2 * chassis.J4310[i].para.vel * chassis.J4310[i].para.vel +
								turn_power.a * chassis.J4310[i].PD_Ctrl.outputTorque * chassis.J4310[i].PD_Ctrl.outputTorque + turn_power.constant;
		turn_power.InitialTotalPower += turn_power.InitialGivePower[i];
	}
	//判断是否是超出允许的功率上限
	if(turn_power.InitialTotalPower > turn_power.UserPowerLimit)
	{
		//计算pid输出放缩系数
		float a0 = 0;
		float b0 = 0;
		float c0 = 0;
		for (uint8_t i = 0;i<4;i++)
		{
			a0 += turn_power.a*chassis.J4310[i].PD_Ctrl.outputTorque * chassis.J4310[i].PD_Ctrl.outputTorque;
			b0 += turn_power.toque_coefficient * chassis.J4310[i].PD_Ctrl.outputTorque * chassis.J4310[i].para.vel;
			c0 += turn_power.k2 * chassis.J4310[i].para.vel * chassis.J4310[i].para.vel+turn_power.constant;
		}
		c0 -= turn_power.UserPowerLimit;
		//判断是否有解
		float delta = b0*b0-4*a0*c0;
		if(delta < 0)
			turn_power.scaleFactor = 0;
		else 
		{
			float x1 = (-b0+sqrt(delta))/(2*a0);
			float x2 = (-b0-sqrt(delta))/(2*a0);
			float s1 = (x1 >= 0.0f && x1 <= 1.0f) ? x1 : 0.0f;
			float s2 = (x2 >= 0.0f && x2 <= 1.0f) ? x2 : 0.0f;
			turn_power.scaleFactor = fmaxf(s1, s2);
		}
		if(turn_power.scaleFactor >= 0.0f && turn_power.scaleFactor <= 1.0f)
		{
			for(uint8_t i = 0;i<4;i++)
			{
				chassis.J4310[i].PD_Ctrl.outputTorque = chassis.J4310[i].PD_Ctrl.outputTorque*turn_power.scaleFactor;
				LIMIT(chassis.J4310[i].PD_Ctrl.outputTorque,-7,7);
			}
		}
		else
		{
			for(uint8_t i = 0;i<4;i++)
			{
				chassis.J4310[i].PD_Ctrl.outputTorque*=0.0001f;
			}
		}
	}
	for(uint8_t i=0;i<4;i++)
	{
		LIMIT(chassis.J4310[i].PD_Ctrl.outputTorque,-7,7);
		turn_power.output += chassis.J4310[i].PD_Ctrl.outputTorque;
	}
	
}

//轮电机功率控制
void WhellPowerCtral()
{
	whell_power.a = fmaxf(1e-7, whell_power.paramVector[1][0]);
	whell_power.k2 = fmaxf(whell_power.paramVector[0][0], 1e-7);
	whell_power.constant = fmaxf(0.7f, whell_power.paramVector[2][0]);
	
	//数据清零
	whell_power.SumPowerSpeed = 0;
	whell_power.SumPowerTorque = 0;
	whell_power.EffetivePower = 0;
	whell_power.InitialTotalPower = 0;
	whell_power.scaleFactor = 0;
	whell_power.PredictPower = 0;
	
	//读取最大功率
	//whell_power.MaxPowerLimit = JUDGE_GetChassisPowerLimit();
	//chassis_power_buffer = JUDGE_GetPowerBuffer();
//	if (whell_power.MaxPowerLimit < 15 || whell_power.MaxPowerLimit > 200)
//	{
		whell_power.MaxPowerLimit = whell_power.UserPowerLimit;
	//}
	
	//拟合曲线用
	for (uint8_t i = 0;i<4;i++)
	{
			whell_power.SumPowerSpeed += chassis.M3508[i].speed*chassis.M3508[i].speed;
			whell_power.SumPowerTorque+= whell_power.LastOutput[i]*whell_power.LastOutput[i];
			whell_power.EffetivePower += whell_power.toque_coefficient*chassis.M3508[i].speed*whell_power.LastOutput[i];
	}
	whell_power.PredictPower = whell_power.paramVector[1][0] * whell_power.SumPowerTorque + whell_power.paramVector[0][0] * whell_power.SumPowerSpeed + 4 * whell_power.paramVector[2][0] + whell_power.EffetivePower;
	//power.MeasurePower = cap.receive_data.bus_power / 100.0f;
		if (whell_power.EffetivePower > 10 && whell_power.MeasurePower>10 )//&& chassis.move.fastMode == 0)
	{
	//	PowerControl_AutoUpdateParam(whell_power.SumPowerSpeed / 4.0f, whell_power.SumPowerTorque / 4.0f, 1, (whell_power.MeasurePower-whell_power.EffetivePower) / 4.0f,whell_power);
	}
	//预测控制前功率
	for(uint8_t i=0;i<4;i++)
	{
		whell_power.InitialGivePower[i] = whell_power.toque_coefficient * chassis.M3508[i].speed*chassis.M3508[i].speedPID.output +
																			whell_power.k2 * chassis.M3508[i].speed*chassis.M3508[i].speed +
																			whell_power.a * chassis.M3508[i].speedPID.output*chassis.M3508[i].speedPID.output + whell_power.constant;
		whell_power.InitialTotalPower += whell_power.InitialGivePower[i];
	}
	//轮电机最大功率
	whell_power.InputPower = whell_power.MaxPowerLimit - turn_power.output;
	LIMIT(whell_power.InputPower,5, whell_power.MaxPowerLimit+20);
	float modelPower = fmax(whell_power.MaxPowerLimit,whell_power.InputPower);
	float delta = TOQUE_CONST * whell_power.toque_coefficient * TOQUE_CONST * whell_power.toque_coefficient - 4 * whell_power.k2 * whell_power.constant + whell_power.k2 * modelPower - 4 * whell_power.a * whell_power.k2 * TOQUE_CONST;
	if (delta < 0) delta = 0;
	SET_WHEELSPEED_MAX = 1000.0f + ((sqrtf(delta) - whell_power.toque_coefficient * TOQUE_CONST) / (2.0f * whell_power.k2));

	//判断是否是超出允许的功率上限
	if(whell_power.InitialTotalPower > whell_power.InputPower)
	{
		//计算pid输出放缩系数
		float a0 = 0;
		float b0 = 0;
		float c0 = 0;
		for (uint8_t i = 0;i<4;i++)
		{
			a0 += whell_power.a * chassis.M3508[i].speedPID.output*chassis.M3508[i].speedPID.output;
			b0 += whell_power.toque_coefficient * chassis.M3508[i].speed*chassis.M3508[i].speedPID.output;
			c0 += whell_power.k2 * chassis.M3508[i].speed*chassis.M3508[i].speed+whell_power.constant;
		}
		c0 -= whell_power.InputPower;
		//判断是否有解
		float delta = b0*b0-4*a0*c0;
		if(delta < 0)
			whell_power.scaleFactor = 0;
		else 
		{
			float x1 = (-b0+sqrt(delta))/(2*a0);
			float x2 = (-b0-sqrt(delta))/(2*a0);
			float s1 = (x1 >= 0.0f && x1 <= 1.0f) ? x1 : 0.0f;
			float s2 = (x2 >= 0.0f && x2 <= 1.0f) ? x2 : 0.0f;
			whell_power.scaleFactor = fmaxf(s1, s2);
		}
		if(whell_power.scaleFactor >= 0.0f && whell_power.scaleFactor <= 1.0f)
		{
			for(uint8_t i = 0;i<4;i++)
			{
				chassis.M3508[i].speedPID.output =chassis.M3508[i].speedPID.output*whell_power.scaleFactor;
				LIMIT(chassis.M3508[i].speedPID.output,-16000,16000);
			}
		}
		else
		{
			for(uint8_t i = 0;i<4;i++)
			{
				chassis.M3508[i].speedPID.output*=0.0001f;
			}
		}
	}
	for(uint8_t i=0;i<4;i++)
	{
		LIMIT(chassis.M3508[i].speedPID.output,-16000,16000);
		whell_power.LastOutput[i] = chassis.M3508[i].speedPID.output;
	}
	
}

void PowerCtrl()
{
	TurnPowerCtral();
	switch (USER_SentryCmd.sentry_mode)
	{
		case 1: //进攻
			whell_power.UserPowerLimit = 50 - turn_power.PredictPower;
			break;
		case 2: //防守
			whell_power.UserPowerLimit = 50 - turn_power.PredictPower;
			break;
		case 3: //移动
			whell_power.UserPowerLimit = 150 - turn_power.PredictPower;
			break;
		default:
			whell_power.UserPowerLimit = GameRobotState.chassis_power_limit - turn_power.PredictPower;
			break;
	}
	WhellPowerCtral();
}
