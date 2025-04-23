// ad_can.h
#ifndef AD_CAN_H
#define AD_CAN_H

#include <stdint.h>
#include "stm32h7xx_hal.h" // 包含 HAL 頭文件以使用 FDCAN_HandleTypeDef

extern volatile uint8_t ADcan_data_received; // CAN 數據接收標誌
extern uint8_t rx_data[8]; // CAN 接收數據緩衝區
//extern float ch6_value; // 激光距離值 (m)
extern float ch8_value; // 保留，第8路模擬量值 (V 或 A)
extern float laser_ema; // EMA 平滑值
extern float motor_pos; // 電機累計角度 (度)
extern float window_data[1200]; // 方差計算窗口數據 (60s / 0.05s = 1200)
extern uint32_t window_idx; // 窗口索引
extern float variance; // 窗口方差
extern uint8_t is_stopped; // 停止狀態標誌
extern FDCAN_HandleTypeDef hfdcan1; // 激光 CAN
extern FDCAN_HandleTypeDef hfdcan2; // 電機 CAN

#endif // AD_CAN_H