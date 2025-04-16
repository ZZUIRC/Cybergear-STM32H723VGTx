#include "cybergeartx.h"
/**
************************************************************************
* @brief:      	can_bsp_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN ʹ��
************************************************************************
**/
void can_bsp_init(void)
{
	can_filter_init();
	HAL_FDCAN_Start(&hfdcan1);                               //����FDCAN
	HAL_FDCAN_Start(&hfdcan2);
	HAL_FDCAN_Start(&hfdcan3);
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}
/**
************************************************************************
* @brief:      	can_filter_init(void)
* @param:       void
* @retval:     	void
* @details:    	CAN�˲�����ʼ��
************************************************************************
**/
void can_filter_init(void)
{
		FDCAN_FilterTypeDef fdcan_filter;

	// FDCAN1 
	//fdcan_filter.IdType = FDCAN_STANDARD_ID;                       //��׼ID
    fdcan_filter.IdType = FDCAN_EXTENDED_ID;                       // 29???ID
    fdcan_filter.FilterIndex = 0;                                  // ?????
    fdcan_filter.FilterType = FDCAN_FILTER_MASK;                   // ??????
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;           // ???FIFO0
		fdcan_filter.FilterID1 = 0x111;                               //����ID1
		fdcan_filter.FilterID2 =0x1FFFFFFF;                        // ??(????)
    HAL_FDCAN_ConfigFilter(&hfdcan1, &fdcan_filter);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1);
	
	
		// FDCAN 2
	//fdcan_filter.IdType = FDCAN_STANDARD_ID;                       //��׼ID
    fdcan_filter.IdType = FDCAN_EXTENDED_ID;                       // 29???ID
    fdcan_filter.FilterIndex = 1;                                  // ?????
    fdcan_filter.FilterType = FDCAN_FILTER_MASK;                   // ??????
    fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;           // ???FIFO0
		fdcan_filter.FilterID1 = 0x111;                               //����ID1
		fdcan_filter.FilterID2 = 0x1FFFFFFF ;                       // ??(????)
    HAL_FDCAN_ConfigFilter(&hfdcan2, &fdcan_filter);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_ConfigFifoWatermark(&hfdcan2, FDCAN_CFG_RX_FIFO0, 1);

}
/**
************************************************************************
* @brief:      	fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
* @param:       hfdcan��FDCAN���
* @param:       id��CAN�豸ID
* @param:       data�����͵�����
* @param:       len�����͵����ݳ���
* @retval:     	void
* @details:    	��������
************************************************************************
**/
uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t *data, uint32_t len)
{	
	FDCAN_TxHeaderTypeDef TxHeader;
	
  TxHeader.Identifier = id;
  //TxHeader.IdType = FDCAN_STANDARD_ID;																// ��׼ID 
	TxHeader.IdType = FDCAN_EXTENDED_ID;																// ��׼ID 
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;														// ����֡ 
  TxHeader.DataLength =  FDCAN_DLC_BYTES_8;																		// �������ݳ��� 
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;										// ���ô���״ָ̬ʾ 								
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;															// �������ɱ䲨���� 
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;															// ��ͨCAN��ʽ 
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;										// ���ڷ����¼�FIFO����, ���洢 
  TxHeader.MessageMarker = 0x00; 			// ���ڸ��Ƶ�TX EVENT FIFO����ϢMaker��ʶ����Ϣ״̬����Χ0��0xFF                
    
  if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, data)!=HAL_OK) 
		return 1;//����
	return 0;	
}
/**
************************************************************************
* @brief:      	fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf)
* @param:       hfdcan��FDCAN���
* @param:       buf���������ݻ���
* @retval:     	���յ����ݳ���
* @details:    	��������
************************************************************************
**/
uint8_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint8_t *buf)
{	
		FDCAN_RxHeaderTypeDef fdcan_RxHeader;
    HAL_StatusTypeDef status = HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &fdcan_RxHeader, buf);
	switch (status)
	{
		case HAL_OK:
		return (uint8_t)(fdcan_RxHeader.DataLength >> 16);
	break;		
		case	HAL_ERROR:
		return 0;
	break;	
		case	HAL_TIMEOUT:
		return 0;
	break;	
		case	HAL_BUSY:
			return 0;
	default:
		return 0;
						}

}
/**
************************************************************************
* @brief:      	HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
* @param:       hfdcan��FDCAN���
* @param:       RxFifo0ITs���жϱ�־λ
* @retval:     	void
* @details:    	HAL���FDCAN�жϻص�����
************************************************************************
**/
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
  {
		if(hfdcan == &hfdcan1)
		{
			fdcan1_rx_callback();
		}
		if(hfdcan == &hfdcan2)
		{
			fdcan2_rx_callback();
		}
		if(hfdcan == &hfdcan3)
		{
			fdcan3_rx_callback();
		}
	}
}
/**
************************************************************************
* @brief:      	fdcan_rx_callback(void)
* @param:       void
* @retval:     	void
* @details:    	���û����õĽ���������
************************************************************************
**/
uint8_t rx_data1[8] = {0};
void fdcan1_rx_callback(void)
{
	fdcanx_receive(&hfdcan1, rx_data1);
}
uint8_t rx_data2[8] = {0};
void fdcan2_rx_callback(void)
{
	fdcanx_receive(&hfdcan2, rx_data2);
}
uint8_t rx_data3[8] = {0};
void fdcan3_rx_callback(void)
{
	fdcanx_receive(&hfdcan3, rx_data3);
}
