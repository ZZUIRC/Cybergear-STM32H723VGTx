#ifndef CYBERGEAR_CAN_INTERFACE_STM32_H
#define CYBERGEAR_CAN_INTERFACE_STM32_H

#include "stm32h7xx_hal.h"
#include "cybergear_can_interface.h"
#include "string.h"

class CybergearCanInterfaceStm32 : public CybergearCanInterface
{
public:
  CybergearCanInterfaceStm32();
  virtual ~CybergearCanInterfaceStm32();
  bool initCan(FDCAN_HandleTypeDef* hfdcan);
  virtual bool send_message(uint32_t id, const uint8_t * data, uint8_t len, bool ext);
  virtual bool read_message(unsigned long & id, uint8_t * data, uint8_t & len);
  virtual bool available();
  virtual bool support_interrupt();

private:
  FDCAN_HandleTypeDef* hfdcan_;
  FDCAN_TxHeaderTypeDef tx_header_;
  uint8_t tx_data_[8];
};

#endif  // CYBERGEAR_CAN_INTERFACE_STM32_HH
