// ad_can.h
#ifndef AD_CAN_H
#define AD_CAN_H

#include <stdint.h>

extern volatile uint8_t ADcan_data_received; // CAN 數據接收標誌
extern uint8_t rx_data[8]; // CAN 接收數據緩衝區
extern float ch6_value; // 第6路模擬量值 (V 或 A)
extern float ch8_value; // 第8路模擬量值 (V 或 A)

#endif // AD_CAN_H
