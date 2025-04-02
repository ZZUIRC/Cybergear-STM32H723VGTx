#include "cybergear_can_interface_stm32.h"

CybergearCanInterfaceStm32::CybergearCanInterfaceStm32() : CybergearCanInterface() {}

CybergearCanInterfaceStm32::~CybergearCanInterfaceStm32() {};

bool CybergearCanInterfaceStm32::initCan(FDCAN_HandleTypeDef* hfdcan)
{
  hfdcan_ = hfdcan;

  /* setting for rx */
  FDCAN_FilterTypeDef sFilterConfig;
  sFilterConfig.IdType = FDCAN_EXTENDED_ID;
  sFilterConfig.FilterIndex = 1;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x0;
  sFilterConfig.FilterID2 = 0x0;
  HAL_FDCAN_ConfigFilter(hfdcan_, &sFilterConfig);
  HAL_FDCAN_ConfigGlobalFilter(hfdcan_, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
  HAL_FDCAN_ConfigFifoWatermark(hfdcan_, FDCAN_CFG_RX_FIFO0, 1);

  /* setting for tx 
  tx_header_.IdType = FDCAN_EXTENDED_ID;
  tx_header_.TxFrameType = FDCAN_DATA_FRAME;
	tx_header_.DataLength =  FDCAN_DLC_BYTES_8;	
  tx_header_.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_header_.BitRateSwitch = FDCAN_BRS_OFF; // FDCAN_BRS_OFF;
  tx_header_.FDFormat = FDCAN_CLASSIC_CAN;															// ÆÕÍ¨CAN¸ñÊ½ 
  tx_header_.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx_header_.MessageMarker = 0x00;*/
	

 // HAL_FDCAN_ConfigTxDelayCompensation(hfdcan_, 7, 0); //Configure hfdcan1
 // HAL_FDCAN_EnableTxDelayCompensation(hfdcan_); //Enable TDC of hfdcan1

  /* start rx */
  HAL_FDCAN_Start(hfdcan_);
  HAL_FDCAN_ActivateNotification(hfdcan_, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  return true;
}

bool CybergearCanInterfaceStm32::send_message(uint32_t id, const uint8_t* data, uint8_t len, bool ext)
{
    const uint32_t timeout = 1000; // ???? 1000ms

    // ?? ID ???
    if (ext)
    {
        if (id > 0x1FFFFFFF) // ?? ID ???
        {
            return false;
        }
        tx_header_.IdType = FDCAN_EXTENDED_ID;
    }
    else
    {
        if (id > 0x7FF) // ?? ID ???
        {
            return false;
        }
        tx_header_.IdType = FDCAN_STANDARD_ID;
    }

    // ???? ID ????
    tx_header_.Identifier = id;
    tx_header_.FDFormat = FDCAN_CLASSIC_CAN;

    // ?? len ??????
    if (len > 8) len = 8; // ?? CAN ?? 8 ??
    switch (len)
    {
        case 0: tx_header_.DataLength = FDCAN_DLC_BYTES_0; break;
        case 1: tx_header_.DataLength = FDCAN_DLC_BYTES_1; break;
        case 2: tx_header_.DataLength = FDCAN_DLC_BYTES_2; break;
        case 3: tx_header_.DataLength = FDCAN_DLC_BYTES_3; break;
        case 4: tx_header_.DataLength = FDCAN_DLC_BYTES_4; break;
        case 5: tx_header_.DataLength = FDCAN_DLC_BYTES_5; break;
        case 6: tx_header_.DataLength = FDCAN_DLC_BYTES_6; break;
        case 7: tx_header_.DataLength = FDCAN_DLC_BYTES_7; break;
        default: tx_header_.DataLength = FDCAN_DLC_BYTES_8; break;
    }

    // ???????????
    memcpy(tx_data_, data, len);
    if (len < 8)
    {
        memset(tx_data_ + len, 0, 8 - len); // ??????
    }

    // ????
    uint32_t tickstart = HAL_GetTick();
    while (true)
    {
        if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_, &tx_header_, tx_data_) == HAL_OK)
        {
            return true;
        }

        // ??????
        if (hfdcan_->ErrorCode & HAL_FDCAN_ERROR_FIFO_FULL)
        {
            // Tx FIFO/Queue ??,?? 1ms ???
            HAL_Delay(1);
        }
        else
        {
            // ????,????
            return false;
        }

        // ????
        if ((timeout == 0U) || ((HAL_GetTick() - tickstart) > timeout))
        {
            return false;
        }
    }
}
bool CybergearCanInterfaceStm32::read_message(unsigned long & id, uint8_t * data, uint8_t & len)
{
  
  return true;
}

bool CybergearCanInterfaceStm32::available()
{
  return true;
}

bool CybergearCanInterfaceStm32::support_interrupt()
{
  return true;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs)
{
  FDCAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8] = {0x00};

  HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);
}


