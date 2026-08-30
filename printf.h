#ifndef __PRINTF_H
#define __PRINTF_H

#include "Config.h"
#include <stdint.h>

#define CH_COUNT             16
#define MODE_COMMAND_MAX_LEN 32

typedef struct {
    float fdata[CH_COUNT];
    uint8_t tail[4];
} PRINTF_STRUCT;

extern uint8_t PrintfBuff[200];

void Printf_TX_Init(PRINTF_STRUCT *str);
void Printf_TX_Loop(PRINTF_STRUCT *str);
void Printf_RX_Init(void);
void Printf_RX_Loop(const uint8_t *data, uint16_t length);


#endif // PRINTF_H
