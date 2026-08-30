#ifndef MOTOR_SDK_H
#define MOTOR_SDK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_SDK_VERSION_MAJOR          1U
#define MOTOR_SDK_VERSION_MINOR          0U
#define MOTOR_SDK_VERSION_PATCH          0U
#define MOTOR_SDK_CAN_PAYLOAD_LENGTH     8U
#define MOTOR_SDK_INVALID_IDENTIFIER     0xFFFFFFFFUL

typedef enum {
    MOTOR_SDK_OK                 = 0,
    MOTOR_SDK_CLAMPED            = 1,
    MOTOR_SDK_NOT_FOR_THIS_MOTOR = 2,
    MOTOR_SDK_ERROR_ARGUMENT     = -1,
    MOTOR_SDK_ERROR_CONFIG       = -2,
    MOTOR_SDK_ERROR_NOT_INITIALIZED = -3,
    MOTOR_SDK_ERROR_FRAME        = -4,
    MOTOR_SDK_ERROR_NOT_FINITE   = -5
} MotorSDK_Result;

/* 数值与达妙 CTRL_MODE 保持一致。 */
typedef enum {
    MOTOR_SDK_MODE_INVALID        = 0,
    MOTOR_SDK_MODE_MIT            = 1,
    MOTOR_SDK_MODE_POSITION_SPEED = 2,
    MOTOR_SDK_MODE_SPEED          = 3,
    MOTOR_SDK_MODE_POSITION_FORCE = 4
} MotorSDK_ControlMode;

typedef enum {
    MOTOR_SDK_SPECIAL_CLEAR_ERROR = 0xFB,
    MOTOR_SDK_SPECIAL_ENABLE      = 0xFC,
    MOTOR_SDK_SPECIAL_DISABLE     = 0xFD,
    MOTOR_SDK_SPECIAL_SAVE_ZERO   = 0xFE
} MotorSDK_SpecialCommand;

typedef enum {
    MOTOR_SDK_PROTOCOL_ERROR_NONE          = 0x0,
    MOTOR_SDK_PROTOCOL_ERROR_OVERVOLTAGE   = 0x8,
    MOTOR_SDK_PROTOCOL_ERROR_UNDERVOLTAGE  = 0x9,
    MOTOR_SDK_PROTOCOL_ERROR_OVERCURRENT   = 0xA,
    MOTOR_SDK_PROTOCOL_ERROR_MOS_OVERHEAT  = 0xB,
    MOTOR_SDK_PROTOCOL_ERROR_COIL_OVERHEAT = 0xC,
    MOTOR_SDK_PROTOCOL_ERROR_COMM_LOST     = 0xD,
    MOTOR_SDK_PROTOCOL_ERROR_OVERLOAD      = 0xE
} MotorSDK_ProtocolError;

typedef struct {
    uint8_t motor_id;
    uint16_t master_id;

    float position_min_rad;
    float position_max_rad;
    float velocity_min_rad_s;
    float velocity_max_rad_s;
    float torque_min_nm;
    float torque_max_nm;
    float kp_min;
    float kp_max;
    float kd_min;
    float kd_max;
} MotorSDK_Config;

typedef struct {
    uint32_t identifier;
    uint8_t length;
    uint8_t data[MOTOR_SDK_CAN_PAYLOAD_LENGTH];
} MotorSDK_CANFrame;

typedef struct {
    float position_rad;
    float velocity_rad_s;
    float kp;
    float kd;
    float torque_feedforward_nm;
} MotorSDK_MITCommand;

typedef struct {
    float position_rad;
    float velocity_limit_rad_s;
} MotorSDK_PositionSpeedCommand;

typedef struct {
    float velocity_rad_s;
} MotorSDK_SpeedCommand;

typedef struct {
    float position_rad;
    float velocity_limit_rad_s;
    float current_limit_ratio;
} MotorSDK_PositionForceCommand;

typedef struct {
    uint8_t motor_id;
    MotorSDK_ProtocolError error;
    float position_rad;
    float velocity_rad_s;
    float torque_nm;
    uint8_t mos_temperature_c;
    uint8_t rotor_temperature_c;
} MotorSDK_Feedback;

typedef struct {
    MotorSDK_Config config;
    MotorSDK_Feedback feedback;
    uint8_t initialized;
    uint8_t feedback_valid;
} MotorSDK_Motor;

/* 默认使用 DM4310 量程：P=12.5, V=30, T=10, Kp=500, Kd=5。 */
void MotorSDK_ConfigDefault(MotorSDK_Config *config,
                            uint8_t motor_id,
                            uint16_t master_id);
MotorSDK_Result MotorSDK_Init(MotorSDK_Motor *motor,
                              const MotorSDK_Config *config);
const char *MotorSDK_GetVersion(void);

uint32_t MotorSDK_GetCommandIdentifier(const MotorSDK_Motor *motor,
                                       MotorSDK_ControlMode mode);
uint32_t MotorSDK_GetFeedbackIdentifier(const MotorSDK_Motor *motor);

/* 特殊帧始终发送到基础 motor_id。 */
MotorSDK_Result MotorSDK_BuildSpecialFrame(const MotorSDK_Motor *motor,
                                           MotorSDK_SpecialCommand command,
                                           MotorSDK_CANFrame *frame);

MotorSDK_Result MotorSDK_BuildMITFrame(const MotorSDK_Motor *motor,
                                       const MotorSDK_MITCommand *command,
                                       MotorSDK_CANFrame *frame);
MotorSDK_Result MotorSDK_BuildPositionSpeedFrame(
    const MotorSDK_Motor *motor,
    const MotorSDK_PositionSpeedCommand *command,
    MotorSDK_CANFrame *frame);
MotorSDK_Result MotorSDK_BuildSpeedFrame(const MotorSDK_Motor *motor,
                                         const MotorSDK_SpeedCommand *command,
                                         MotorSDK_CANFrame *frame);
MotorSDK_Result MotorSDK_BuildPositionForceFrame(
    const MotorSDK_Motor *motor,
    const MotorSDK_PositionForceCommand *command,
    MotorSDK_CANFrame *frame);

/* 解析电机发回 master_id 的达妙 8 字节反馈帧。 */
MotorSDK_Result MotorSDK_DecodeFeedback(MotorSDK_Motor *motor,
                                        const MotorSDK_CANFrame *frame,
                                        MotorSDK_Feedback *feedback);
const MotorSDK_Feedback *MotorSDK_GetFeedback(const MotorSDK_Motor *motor);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_SDK_H */
