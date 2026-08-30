#ifndef __MIT_PROTOCOL_H
#define __MIT_PROTOCOL_H

#include <stdint.h>

#include "UserData_Config.h"

#define MIT_SPECIAL_CLEAR_ERROR 0xFBU
#define MIT_SPECIAL_ENABLE      0xFCU
#define MIT_SPECIAL_DISABLE     0xFDU
#define MIT_SPECIAL_SAVE_ZERO   0xFEU

#define MIT_ERROR_NONE          0x0U
#define MIT_ERROR_OVERVOLTAGE   0x8U
#define MIT_ERROR_UNDERVOLTAGE  0x9U
#define MIT_ERROR_OVERCURRENT   0xAU
#define MIT_ERROR_MOS_OVERHEAT  0xBU
#define MIT_ERROR_COIL_OVERHEAT 0xCU
#define MIT_ERROR_COMM_LOST     0xDU
#define MIT_ERROR_OVERLOAD      0xEU

#define MIT_MODE_MIT             1U
#define MIT_MODE_POSITION_SPEED  2U
#define MIT_MODE_SPEED           3U
#define MIT_MODE_POSITION_FORCE  4U

#define MIT_POSITION_SPEED_ID    (0x100U + MIT_CAN_ID)
#define MIT_SPEED_ID             (0x200U + MIT_CAN_ID)
#define MIT_POSITION_FORCE_ID    (0x300U + MIT_CAN_ID)
#define MIT_MODE_ID_MASK         0x4FFU

typedef struct{
    float Position;
    float Velocity;
    float Kp;
    float Kd;
    float Torque;
    float Current_Limit;
}MIT_COMMAND_STRUCT;

typedef struct{
    MIT_COMMAND_STRUCT command;

    float Real_Position;
    float Real_Velocity;
    float Real_Torque;
    float Target_Torque;
    float Target_Iq;
    float Torque_Constant;

    float Position_Zero;
    float Position_Unwrapped;
    float Position_Last;

    volatile uint32_t Timeout_Count;
    volatile uint8_t Control_Mode;
    volatile uint8_t Enabled;
    volatile uint8_t Command_Valid;
    volatile uint8_t Timed_Out;
    uint8_t Position_Initialized;
    uint16_t Outer_Loop_Count;
}MIT_PROTOCOL_STRUCT;

extern MIT_PROTOCOL_STRUCT MIT;

void MIT_Protocol_Init(void);
uint8_t MIT_Protocol_SetControlMode(uint8_t mode);
uint32_t MIT_Protocol_GetCommandIdentifier(void);
uint8_t MIT_Protocol_IsSupportedIdentifier(uint32_t identifier);
uint8_t MIT_Protocol_Receive(uint32_t identifier,
                            const uint8_t data[8],
                            uint8_t length);
void MIT_Protocol_FeedbackUpdate(void);
void MIT_Protocol_ControlLoop(void);
void MIT_Protocol_1msLoop(void);
void MIT_Protocol_PackFeedback(uint8_t data[8]);
uint8_t MIT_Protocol_GetErrorCode(void);

uint32_t MIT_FloatToUint(float value,
                         float value_min,
                         float value_max,
                         uint8_t bits);
float MIT_UintToFloat(uint32_t value,
                      float value_min,
                      float value_max,
                      uint8_t bits);

#endif // MIT_PROTOCOL_H
