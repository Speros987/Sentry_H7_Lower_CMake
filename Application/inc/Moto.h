#ifndef _MOTO_H_
#define _MOTO_H_

#include "USER_Moto.h"
#include "PID.h"
#include "super_cap.h"


typedef struct CHASSIS
{
	DJI_Motor_t M3508[4];
	DM_motor_t J4310[4];
} ChassisMotor_t;

typedef struct GIMBAL
{
	DM_motor_t J4310_yaw;
	
}GimbalMotor_t;

extern ChassisMotor_t chassis;
extern GimbalMotor_t gimbal;
void Supercap_Update(SuperCap* this, uint8_t* rxdata);

#endif
