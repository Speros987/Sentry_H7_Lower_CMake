#include "USER_B2B.h"
#include "usart.h"
#include "cmsis_os.h"
#include "UserFreertos.h"
#include "Moto.h"
#include "judge.h"
#include "USER_Detcet.h"
#include "imu_temp_ctrl.h"
#include <string.h>


extern DMA_HandleTypeDef hdma_usart2_rx;
uint8_t rs485_isvalid = 0;

#define B2B_FRAME_LEN 64U


/* 需要用到的接收变量*/
uint8_t usart2RxBuf[256]; // 串口2缓冲区
uint8_t STOPFLAG = 0;
uint8_t FEEDBACK = 0;


uint32_t receive_times;

/* 需要用到的发送变量*/
uint8_t txbuffer[64] = {0};

void B2B_Init()
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, usart2RxBuf, sizeof(usart2RxBuf));
	__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
}


void B2B_ParseUsart() // 先发低字节
{
	if (usart2RxBuf[0] == 0xAA && usart2RxBuf[B2B_FRAME_LEN - 1U] == 0xFE)
	{
		/*先解析接收内容*/
		for(uint8_t i = 0; i < 4; i++)
		{
				float v;
				uint8_t *p = (uint8_t *)&v;
				p[0] = usart2RxBuf[1 + 4*i];
				p[1] = usart2RxBuf[2 + 4*i];
				p[2] = usart2RxBuf[3 + 4*i];
				p[3] = usart2RxBuf[4 + 4*i];
				chassis.J4310[i].targetTurnAngle = v;
		} //解析舵电机目标角度（多圈）1-16
		for (uint8_t i = 0; i < 4; i++)
		{
				chassis.M3508[i].targetSpeed = (int16_t)usart2RxBuf[17 + i * 2] | (int16_t)usart2RxBuf[17 + i * 2 + 1] << 8;
		} //轮电机目标速度	17-24
		{
				float v;
				uint8_t *p = (uint8_t *)&v;
				p[0] = usart2RxBuf[25];
				p[1] = usart2RxBuf[26];
				p[2] = usart2RxBuf[27];
				p[3] = usart2RxBuf[28];
				gimbal.J4310_yaw.targetTorque = v;
		}	//大yaw电机目标转矩 25-28

		memcpy(&USER_SentryCmd, &usart2RxBuf[29], sizeof(USER_SentryCmd_t));
		
		STOPFLAG = usart2RxBuf[62];
		
		
		/* 发送    */
		txbuffer[0] = 0xAB;
		for(uint8_t i = 0; i < 4; i++)
		{
				float v = chassis.J4310[i].nowAngle;
				uint8_t *p = (uint8_t *)&v;
				txbuffer[1 + 4*i] = p[0];
				txbuffer[2 + 4*i] = p[1];
				txbuffer[3 + 4*i] = p[2];
				txbuffer[4 + 4*i] = p[3];
		}//1-16 舵电机当前角度（单圈）单位为°
		for (uint8_t i = 0; i < 4; i++)
		{
				txbuffer[17 + i * 2] = chassis.M3508[i].speed;
				txbuffer[17 + i * 2 + 1] = chassis.M3508[i].speed >> 8;
		} // 17-24 轮电机当前速度
		{
				float v = gimbal.J4310_yaw.nowAngle;   // 要发送的那个 float
				uint8_t *p = (uint8_t *)&v;
				txbuffer[25] = p[0];
				txbuffer[26] = p[1];
				txbuffer[27] = p[2];
				txbuffer[28] = p[3];
		}// 25-28 yaw电机当前角度(单圈)单位为°

		// {
		// 		float v = gimbal.J4310_yaw.para.vel;   // 要发送的那个 float
		// 		uint8_t *p = (uint8_t *)&v;
		// 		txbuffer[29] = p[0];
		// 		txbuffer[30] = p[1];
		// 		txbuffer[31] = p[2];
		// 		txbuffer[32] = p[3];
		// }	//29-32 yaw电机当前速度(单圈)单位为°  //先不用

		memcpy(&txbuffer[29], &USER_JudgeData, sizeof(JudgeData_t)); //29-58 裁判系统数据

		txbuffer[62] = FEEDBACK;
		txbuffer[B2B_FRAME_LEN - 1U] = 0xFD;	
		rs485_isvalid = 1;
		HAL_UART_Transmit_DMA(&huart2, txbuffer, sizeof(txbuffer));
	}
}
	
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart == &huart2)
	{
		if (Size == B2B_FRAME_LEN)
		{
			receive_times ++;
			B2B_ParseUsart();
			Detect_Update(DeviceID_B2B);
			detectList[DeviceID_B2B].isLost = 0;
		}
	}
	if(huart == &huart1)
	{
			Judge_Receive();
			Detect_Update(DeviceID_Judge);	
	}
}


void Task_B2B_Callback()
{
	/**********特殊情况处理*********************/
	if (STOPFLAG == 1)
	{
		FEEDBACK = 1;
		B2B_ParseUsart();
		osThreadResume(ErrorTaskHandle); // 恢复错误任务 饿死其他任务
	}
}
int times;
/************************freertos任务****************************/

void OS_Board2BoardCallback(void const *argument)
{
	B2B_Init();
	for (;;)
	{
		times ++;
		if(times >= 200)
		{
			times = 0;
			if(receive_times <= 10)
			{
				HAL_UARTEx_ReceiveToIdle_DMA(&huart2,usart2RxBuf,sizeof(usart2RxBuf));
				__HAL_DMA_DISABLE_IT(&hdma_usart2_rx,DMA_IT_HT); 
			}
			receive_times = 0;
		}
		Task_B2B_Callback();
		osDelay(1);
	}
}

