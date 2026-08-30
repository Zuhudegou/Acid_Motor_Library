/*
 * 控制端接入示例。SDK 只生成/解析数据，Controller_* 函数由用户
 * 用 STM32 HAL、SocketCAN、USB-CAN 或其他底层驱动实现。
 */
#include "MotorSDK.h"

#include <string.h>

static MotorSDK_Motor Motor1;

void Controller_CAN_Send(uint32_t identifier,
                         const uint8_t data[8],
                         uint8_t length);

static void Controller_SendFrame(const MotorSDK_CANFrame *frame){
    Controller_CAN_Send(frame->identifier, frame->data, frame->length);
}

void Controller_MotorInit(void){
    MotorSDK_Config config;
    MotorSDK_ConfigDefault(&config, 1U, 0x11U);
    if (MotorSDK_Init(&Motor1, &config) != MOTOR_SDK_OK){
        return;
    }
}

void Controller_EnableMotor(void){
    MotorSDK_CANFrame enable;
    if (MotorSDK_BuildSpecialFrame(&Motor1,
                                   MOTOR_SDK_SPECIAL_ENABLE,
                                   &enable) == MOTOR_SDK_OK){
        Controller_SendFrame(&enable);
    }
}

void Controller_SetMotorSpeed(float velocity_rad_s){
    MotorSDK_SpeedCommand command = {
        .velocity_rad_s = velocity_rad_s
    };
    MotorSDK_CANFrame frame;
    MotorSDK_Result result = MotorSDK_BuildSpeedFrame(&Motor1,
                                                       &command,
                                                       &frame);
    if (result >= MOTOR_SDK_OK){
        Controller_SendFrame(&frame);
    }
}

void Controller_CANReceive(uint32_t identifier,
                           const uint8_t data[8],
                           uint8_t length){
    MotorSDK_CANFrame frame = {
        .identifier = identifier,
        .length = length
    };
    if ((data == 0) || (length != MOTOR_SDK_CAN_PAYLOAD_LENGTH)){
        return;
    }
    memcpy(frame.data, data, sizeof(frame.data));

    MotorSDK_Feedback feedback;
    if (MotorSDK_DecodeFeedback(&Motor1, &frame, &feedback) == MOTOR_SDK_OK){
        /* 在此使用 feedback.position_rad / velocity_rad_s / error。 */
    }
}
