#ifndef __MIT_FDCAN_H
#define __MIT_FDCAN_H

#include <stdint.h>

void MIT_FDCAN_Init(void);
void MIT_FDCAN_MainLoop(void);

/*
 * 在HAL_FDCAN_RxFifo0Callback()中调用此函数：
 * MIT_FDCAN_RxFifo0Callback(hfdcan, RxFifo0ITs);
 */
void MIT_FDCAN_RxFifo0Callback(void *hfdcan, uint32_t rx_fifo0_interrupts);

#endif // MIT_FDCAN_H
