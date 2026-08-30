#include "MIT_FDCAN.h"

#include "MIT_Protocol.h"
#include "UserData_Config.h"

#if MIT_PROTOCOL_ENABLE && MIT_FDCAN_HAL_ENABLE

#include MIT_FDCAN_HAL_HEADER

extern FDCAN_HandleTypeDef MIT_FDCAN_HANDLE;

typedef struct{
    uint32_t identifier;
    uint8_t data[8];
}MIT_FDCAN_RX_FRAME_STRUCT;

static MIT_FDCAN_RX_FRAME_STRUCT MIT_RxQueue[MIT_FDCAN_RX_QUEUE_SIZE];
static volatile uint8_t MIT_RxWrite = 0U;
static volatile uint8_t MIT_RxRead = 0U;
static uint8_t MIT_TxPending = 0U;
static uint8_t MIT_TxData[8];

static uint8_t MIT_FDCAN_QueuePush(uint32_t identifier,
                                  const uint8_t data[8]){
    uint8_t next = (uint8_t)((MIT_RxWrite + 1U) % MIT_FDCAN_RX_QUEUE_SIZE);
    if (next == MIT_RxRead){
        return 0U;
    }
    MIT_RxQueue[MIT_RxWrite].identifier = identifier;
    for (uint8_t i = 0; i < 8U; i++){
        MIT_RxQueue[MIT_RxWrite].data[i] = data[i];
    }
    MIT_RxWrite = next;
    return 1U;
}

static uint8_t MIT_FDCAN_QueuePop(MIT_FDCAN_RX_FRAME_STRUCT *frame){
    if (MIT_RxRead == MIT_RxWrite){
        return 0U;
    }
    frame->identifier = MIT_RxQueue[MIT_RxRead].identifier;
    for (uint8_t i = 0; i < 8U; i++){
        frame->data[i] = MIT_RxQueue[MIT_RxRead].data[i];
    }
    MIT_RxRead = (uint8_t)((MIT_RxRead + 1U) % MIT_FDCAN_RX_QUEUE_SIZE);
    return 1U;
}

static uint8_t MIT_FDCAN_Transmit(const uint8_t data[8]){
    FDCAN_TxHeaderTypeDef header = {0};
    header.Identifier = MIT_MASTER_ID;
    header.IdType = FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = FDCAN_DLC_BYTES_8;
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = FDCAN_BRS_OFF;
    header.FDFormat = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    header.MessageMarker = 0U;
    return HAL_FDCAN_AddMessageToTxFifoQ(&MIT_FDCAN_HANDLE,
                                         &header,
                                         (uint8_t *)data) == HAL_OK;
}

void MIT_FDCAN_Init(void){
    FDCAN_FilterTypeDef filter = {0};
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = MIT_FDCAN_FILTER_INDEX;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = MIT_CAN_ID & MIT_MODE_ID_MASK;
    filter.FilterID2 = MIT_MODE_ID_MASK;

    (void)HAL_FDCAN_ConfigFilter(&MIT_FDCAN_HANDLE, &filter);
    (void)HAL_FDCAN_ConfigGlobalFilter(&MIT_FDCAN_HANDLE,
                                        FDCAN_REJECT,
                                        FDCAN_REJECT,
                                        FDCAN_REJECT_REMOTE,
                                        FDCAN_REJECT_REMOTE);
    (void)HAL_FDCAN_Start(&MIT_FDCAN_HANDLE);
    (void)HAL_FDCAN_ActivateNotification(&MIT_FDCAN_HANDLE,
                                          FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                          0U);
}

void MIT_FDCAN_RxFifo0Callback(void *hfdcan,
                               uint32_t rx_fifo0_interrupts){
    FDCAN_HandleTypeDef *handle = (FDCAN_HandleTypeDef *)hfdcan;
    if ((handle != &MIT_FDCAN_HANDLE) ||
        (!(rx_fifo0_interrupts & FDCAN_IT_RX_FIFO0_NEW_MESSAGE))){
        return;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(handle, FDCAN_RX_FIFO0) > 0U){
        FDCAN_RxHeaderTypeDef header = {0};
        uint8_t data[8] = {0};
        if (HAL_FDCAN_GetRxMessage(handle,
                                   FDCAN_RX_FIFO0,
                                   &header,
                                   data) != HAL_OK){
            break;
        }
        if ((header.IdType == FDCAN_STANDARD_ID) &&
            (header.RxFrameType == FDCAN_DATA_FRAME) &&
            (header.DataLength == FDCAN_DLC_BYTES_8) &&
            MIT_Protocol_IsSupportedIdentifier(header.Identifier)){
            (void)MIT_FDCAN_QueuePush(header.Identifier, data);
        }
    }
}

void MIT_FDCAN_MainLoop(void){
    MIT_FDCAN_RX_FRAME_STRUCT frame;

    if (MIT_TxPending){
        if (!MIT_FDCAN_Transmit(MIT_TxData)){
            return;
        }
        MIT_TxPending = 0U;
    }

    while (MIT_FDCAN_QueuePop(&frame)){
        if (MIT_Protocol_Receive(frame.identifier, frame.data, 8U)){
            MIT_Protocol_PackFeedback(MIT_TxData);
            if (!MIT_FDCAN_Transmit(MIT_TxData)){
                MIT_TxPending = 1U;
                return;
            }
        }
    }
}

#else

void MIT_FDCAN_Init(void){
}

void MIT_FDCAN_MainLoop(void){
}

void MIT_FDCAN_RxFifo0Callback(void *hfdcan,
                               uint32_t rx_fifo0_interrupts){
    (void)hfdcan;
    (void)rx_fifo0_interrupts;
}

#endif
