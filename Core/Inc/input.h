#ifndef INPUT_H_
#define INPUT_H_

#include "shared_data.h"

void Input_Init(void);           // 初始化按键模块
void Input_HandleEvent(BtnEvent_t *evt); // 处理来自 ISR 的事件
void Input_PeriodicTask(void);   // 定时处理按住连续动作 (repeat)
#endif
