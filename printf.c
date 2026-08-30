#include "FOC.h"

#include <string.h>

uint8_t PrintfBuff[200];
static uint8_t Printf_BufferIndex = 0U;
static uint8_t Printf_DiscardFrame = 0U;

static uint8_t Printf_ToUpper(uint8_t value){
    if ((value >= (uint8_t)'a') && (value <= (uint8_t)'z')){
        return (uint8_t)(value - ((uint8_t)'a' - (uint8_t)'A'));
    }
    return value;
}

static uint8_t Printf_TextEquals(const char *text){
    return strcmp((const char *)PrintfBuff, text) == 0;
}

static void Printf_ModeSwitch(void){
    uint8_t mode = 0U;

    if (Printf_TextEquals("MODE=1") || Printf_TextEquals("MODE=MIT")){
        mode = MIT_MODE_MIT;
    }
    else if (Printf_TextEquals("MODE=2") ||
             Printf_TextEquals("MODE=POS_VEL") ||
             Printf_TextEquals("MODE=POS_SPEED") ||
             Printf_TextEquals("MODE=POSITION_SPEED")){
        mode = MIT_MODE_POSITION_SPEED;
    }
    else if (Printf_TextEquals("MODE=3") ||
             Printf_TextEquals("MODE=VEL") ||
             Printf_TextEquals("MODE=SPEED")){
        mode = MIT_MODE_SPEED;
    }
    else if (Printf_TextEquals("MODE=4") ||
             Printf_TextEquals("MODE=POS_FORCE") ||
             Printf_TextEquals("MODE=POSITION_FORCE")){
        mode = MIT_MODE_POSITION_FORCE;
    }

    if (mode != 0U){
        (void)MIT_Protocol_SetControlMode(mode);
    }
}

void Printf_TX_Init(PRINTF_STRUCT *str){
    str->tail[0] = 0x00U;
    str->tail[1] = 0x00U;
    str->tail[2] = 0x80U;
    str->tail[3] = 0x7FU;
}

void Printf_TX_Loop(PRINTF_STRUCT *str){
    (void)str;
}

void Printf_RX_Init(void){
    memset(PrintfBuff, 0, sizeof(PrintfBuff));
    Printf_BufferIndex = 0U;
    Printf_DiscardFrame = 0U;
}

void Printf_RX_Loop(const uint8_t *data, uint16_t length){
    if (data == 0){
        return;
    }

    for (uint16_t i = 0U; i < length; i++){
        uint8_t value = data[i];
        if ((value == (uint8_t)'\r') ||
            (value == (uint8_t)'\n') ||
            (value == (uint8_t)' ') ||
            (value == (uint8_t)'\t')){
            continue;
        }

        if (value == (uint8_t)'?'){
            if (!Printf_DiscardFrame){
                PrintfBuff[Printf_BufferIndex] = '\0';
                Printf_ModeSwitch();
            }
            memset(PrintfBuff, 0, sizeof(PrintfBuff));
            Printf_BufferIndex = 0U;
            Printf_DiscardFrame = 0U;
            continue;
        }

        if (Printf_DiscardFrame){
            continue;
        }

        if (Printf_BufferIndex >= (MODE_COMMAND_MAX_LEN - 1U)){
            memset(PrintfBuff, 0, sizeof(PrintfBuff));
            Printf_BufferIndex = 0U;
            Printf_DiscardFrame = 1U;
            continue;
        }

        PrintfBuff[Printf_BufferIndex++] = Printf_ToUpper(value);
    }
}
