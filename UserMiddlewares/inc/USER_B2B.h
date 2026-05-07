#ifndef _USER_B2B_H_
#define _USER_B2B_H_

#include "main.h"

extern uint8_t STOPFLAG;
extern uint8_t FEEDBACK;

extern uint8_t rs485_isvalid;
extern uint32_t rs485_cnt;

void B2B_ParseUsart(void);
//RC初始化
void B2B_Init(void);
void B2B_LostCallback(void);


#endif
