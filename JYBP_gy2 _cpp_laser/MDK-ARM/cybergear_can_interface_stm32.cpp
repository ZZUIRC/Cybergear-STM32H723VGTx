#include "cybergear_can_interface_stm32.h"
#include "ad_can.h" // 新增：包含共享頭文件

extern UART_HandleTypeDef huart1; // ??????


CybergearCanInterfaceStm32::CybergearCanInterfaceStm32() : CybergearCanInterface() {
		hfdcan_ = nullptr;
    tx_header_.Identifier = 0;
    tx_header_.IdType = FDCAN_EXTENDED_ID;
    tx_header_.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header_.DataLength = FDCAN_DLC_BYTES_0;
    tx_header_.TxFrameType = FDCAN_DATA_FRAME;
    tx_header_.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header_.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header_.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header_.MessageMarker = 0;
    memset(tx_data_, 0, sizeof(tx_data_));
}

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
  sFilterConfig.FilterID1 = 0xAA0601;
  sFilterConfig.FilterID2 = 0x00FFFFFF;
	
	//sFilterConfig.FilterID1 = 0x00;
 // sFilterConfig.FilterID2 = 0x00;
  HAL_FDCAN_ConfigFilter(hfdcan_, &sFilterConfig);
  HAL_FDCAN_ConfigGlobalFilter(hfdcan_, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
  HAL_FDCAN_ConfigFifoWatermark(hfdcan_, FDCAN_CFG_RX_FIFO0, 1);

  /* setting for tx 
  tx_header_.IdType = FDCAN_EXTENDED_ID;
  tx_header_.TxFrameType = FDCAN_DATA_FRAME;
	tx_header_.DataLength =  FDCAN_DLC_BYTES_8;	
  tx_header_.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_header_.BitRateSwitch = FDCAN_BRS_OFF; // FDCAN_BRS_OFF;
  tx_header_.FDFormat = FDCAN_CLASSIC_CAN;															// ��ͨCAN��ʽ 
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
  FDCAN_RxHeaderTypeDef rx_header;
// 检查 Rx FIFO0 是否有消息
    if (HAL_FDCAN_GetRxFifoFillLevel(hfdcan_, FDCAN_RX_FIFO0) == 0)
    {
        // 没有消息可读
        return false;
    }

    // 从 Rx FIFO0 中读取消息
    uint8_t rx_data[8] = {0}; // 用于存储接收到的数据
    if (HAL_FDCAN_GetRxMessage(hfdcan_, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
    {
        // 读取消息失败
        return false;
    }

    id = rx_header.Identifier; // 提取消息 ID
    len = rx_header.DataLength; // 提取消息数据长度
 // 将数据长度转换为字节数
    switch (len)
    {
        case FDCAN_DLC_BYTES_0: len = 0; break;
        case FDCAN_DLC_BYTES_1: len = 1; break;
        case FDCAN_DLC_BYTES_2: len = 2; break;
        case FDCAN_DLC_BYTES_3: len = 3; break;
        case FDCAN_DLC_BYTES_4: len = 4; break;
        case FDCAN_DLC_BYTES_5: len = 5; break;
        case FDCAN_DLC_BYTES_6: len = 6; break;
        case FDCAN_DLC_BYTES_7: len = 7; break;
        case FDCAN_DLC_BYTES_8: len = 8; break;
        default: len = 0; break;
    }
    memcpy(data, rx_data, len);
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
  

  HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);
  ADcan_data_received = 1;
	
//		FDCAN_RxHeaderTypeDef rx_header;
//    uint8_t rx_data[8] = {0x00};
//    char buffer[50]; // 定义足够大的缓冲区

//    HAL_StatusTypeDef status = HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);

//    if (status == HAL_OK)
//    {
//        // 使用 sprintf 格式化字符串
//        int len = sprintf(buffer,
//                          "Received message:\n"
//                          "  ID: 0x%lX\n"
//                          "  DLC: %d\n"
//                          "  Data: %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
//                          rx_header.Identifier,
//                          rx_header.DataLength,
//                          rx_data[0], rx_data[1], rx_data[2], rx_data[3],
//                          rx_data[4], rx_data[5], rx_data[6], rx_data[7]);

//        // 使用 HAL_UART_Transmit 发送字符串
//        HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, HAL_MAX_DELAY);
//    }
//    else
//    {
//        // 使用 sprintf 格式化错误信息
//        int len = sprintf(buffer, "Error reading message from FIFO0. Status: %d\r\n", status);

//        // 使用 HAL_UART_Transmit 发送错误信息
//        HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, HAL_MAX_DELAY);
//    }

//    ADcan_data_received = 1; // 标记消息已接收
}


