#include "../include/MotorSDK.h"

#include <string.h>

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(sizeof(float) == 4U,
               "MotorSDK requires 32-bit IEEE-754 float");
#endif

static uint8_t MotorSDK_IsFinite(float value){
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return (bits & 0x7F800000UL) != 0x7F800000UL;
}

static float MotorSDK_Abs(float value){
    return value < 0.0f ? -value : value;
}

static float MotorSDK_Clamp(float value,
                            float minimum,
                            float maximum,
                            uint8_t *clamped){
    if (value > maximum){
        *clamped = 1U;
        return maximum;
    }
    if (value < minimum){
        *clamped = 1U;
        return minimum;
    }
    return value;
}

static uint8_t MotorSDK_ModeValid(MotorSDK_ControlMode mode){
    return (mode >= MOTOR_SDK_MODE_MIT) &&
           (mode <= MOTOR_SDK_MODE_POSITION_FORCE);
}

static uint8_t MotorSDK_ConfigValid(const MotorSDK_Config *config){
    if ((config == 0) ||
        (config->motor_id > 0x0FU) ||
        (config->master_id > 0x7FFU)){
        return 0U;
    }

    const float values[] = {
        config->position_min_rad,
        config->position_max_rad,
        config->velocity_min_rad_s,
        config->velocity_max_rad_s,
        config->torque_min_nm,
        config->torque_max_nm,
        config->kp_min,
        config->kp_max,
        config->kd_min,
        config->kd_max
    };
    for (uint8_t i = 0U; i < (uint8_t)(sizeof(values) / sizeof(values[0])); i++){
        if (!MotorSDK_IsFinite(values[i])){
            return 0U;
        }
    }

    return (config->position_min_rad < config->position_max_rad) &&
           (config->velocity_min_rad_s < config->velocity_max_rad_s) &&
           (config->torque_min_nm < config->torque_max_nm) &&
           (config->kp_min < config->kp_max) &&
           (config->kd_min < config->kd_max);
}

static MotorSDK_Result MotorSDK_CheckMotor(const MotorSDK_Motor *motor){
    if (motor == 0){
        return MOTOR_SDK_ERROR_ARGUMENT;
    }
    if (!motor->initialized){
        return MOTOR_SDK_ERROR_NOT_INITIALIZED;
    }
    return MOTOR_SDK_OK;
}

static void MotorSDK_ClearFrame(MotorSDK_CANFrame *frame,
                                uint32_t identifier){
    frame->identifier = identifier;
    frame->length = MOTOR_SDK_CAN_PAYLOAD_LENGTH;
    memset(frame->data, 0, sizeof(frame->data));
}

static void MotorSDK_PackFloatLE(uint8_t data[4], float value){
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    data[0] = (uint8_t)bits;
    data[1] = (uint8_t)(bits >> 8);
    data[2] = (uint8_t)(bits >> 16);
    data[3] = (uint8_t)(bits >> 24);
}

static uint32_t MotorSDK_FloatToUint(float value,
                                     float minimum,
                                     float maximum,
                                     uint8_t bits){
    uint32_t integer_max = (1UL << bits) - 1UL;
    return (uint32_t)(((value - minimum) * (float)integer_max) /
                      (maximum - minimum));
}

static float MotorSDK_UintToFloat(uint32_t value,
                                  float minimum,
                                  float maximum,
                                  uint8_t bits){
    uint32_t integer_max = (1UL << bits) - 1UL;
    return ((float)value * (maximum - minimum) /
            (float)integer_max) + minimum;
}

static float MotorSDK_VelocityLimitMaximum(const MotorSDK_Config *config){
    float negative = MotorSDK_Abs(config->velocity_min_rad_s);
    float positive = MotorSDK_Abs(config->velocity_max_rad_s);
    return negative > positive ? negative : positive;
}

void MotorSDK_ConfigDefault(MotorSDK_Config *config,
                            uint8_t motor_id,
                            uint16_t master_id){
    if (config == 0){
        return;
    }

    config->motor_id = motor_id;
    config->master_id = master_id;
    config->position_min_rad = -12.5f;
    config->position_max_rad = 12.5f;
    config->velocity_min_rad_s = -30.0f;
    config->velocity_max_rad_s = 30.0f;
    config->torque_min_nm = -10.0f;
    config->torque_max_nm = 10.0f;
    config->kp_min = 0.0f;
    config->kp_max = 500.0f;
    config->kd_min = 0.0f;
    config->kd_max = 5.0f;
}

MotorSDK_Result MotorSDK_Init(MotorSDK_Motor *motor,
                              const MotorSDK_Config *config){
    if (motor == 0){
        return MOTOR_SDK_ERROR_ARGUMENT;
    }
    if (!MotorSDK_ConfigValid(config)){
        return MOTOR_SDK_ERROR_CONFIG;
    }

    *motor = (MotorSDK_Motor){0};
    motor->config = *config;
    motor->feedback.motor_id = config->motor_id;
    motor->initialized = 1U;
    return MOTOR_SDK_OK;
}

const char *MotorSDK_GetVersion(void){
    return "1.0.0";
}

uint32_t MotorSDK_GetCommandIdentifier(const MotorSDK_Motor *motor,
                                       MotorSDK_ControlMode mode){
    if ((MotorSDK_CheckMotor(motor) != MOTOR_SDK_OK) ||
        !MotorSDK_ModeValid(mode)){
        return MOTOR_SDK_INVALID_IDENTIFIER;
    }

    switch (mode){
    case MOTOR_SDK_MODE_POSITION_SPEED:
        return 0x100U + motor->config.motor_id;
    case MOTOR_SDK_MODE_SPEED:
        return 0x200U + motor->config.motor_id;
    case MOTOR_SDK_MODE_POSITION_FORCE:
        return 0x300U + motor->config.motor_id;
    default:
        return motor->config.motor_id;
    }
}

uint32_t MotorSDK_GetFeedbackIdentifier(const MotorSDK_Motor *motor){
    if ((motor == 0) || !motor->initialized){
        return MOTOR_SDK_INVALID_IDENTIFIER;
    }
    return motor->config.master_id;
}

MotorSDK_Result MotorSDK_BuildSpecialFrame(const MotorSDK_Motor *motor,
                                           MotorSDK_SpecialCommand command,
                                           MotorSDK_CANFrame *frame){
    MotorSDK_Result result = MotorSDK_CheckMotor(motor);
    if (result != MOTOR_SDK_OK){
        return result;
    }
    if (frame == 0){
        return MOTOR_SDK_ERROR_ARGUMENT;
    }
    if ((command != MOTOR_SDK_SPECIAL_CLEAR_ERROR) &&
        (command != MOTOR_SDK_SPECIAL_ENABLE) &&
        (command != MOTOR_SDK_SPECIAL_DISABLE) &&
        (command != MOTOR_SDK_SPECIAL_SAVE_ZERO)){
        return MOTOR_SDK_ERROR_ARGUMENT;
    }

    MotorSDK_ClearFrame(frame, motor->config.motor_id);
    for (uint8_t i = 0U; i < 7U; i++){
        frame->data[i] = 0xFFU;
    }
    frame->data[7] = (uint8_t)command;
    return MOTOR_SDK_OK;
}

MotorSDK_Result MotorSDK_BuildMITFrame(const MotorSDK_Motor *motor,
                                       const MotorSDK_MITCommand *command,
                                       MotorSDK_CANFrame *frame){
    MotorSDK_Result result = MotorSDK_CheckMotor(motor);
    if (result != MOTOR_SDK_OK){
        return result;
    }
    if ((command == 0) || (frame == 0)){
        return MOTOR_SDK_ERROR_ARGUMENT;
    }
    if (!MotorSDK_IsFinite(command->position_rad) ||
        !MotorSDK_IsFinite(command->velocity_rad_s) ||
        !MotorSDK_IsFinite(command->kp) ||
        !MotorSDK_IsFinite(command->kd) ||
        !MotorSDK_IsFinite(command->torque_feedforward_nm)){
        return MOTOR_SDK_ERROR_NOT_FINITE;
    }

    uint8_t clamped = 0U;
    float position = MotorSDK_Clamp(command->position_rad,
                                    motor->config.position_min_rad,
                                    motor->config.position_max_rad,
                                    &clamped);
    float velocity = MotorSDK_Clamp(command->velocity_rad_s,
                                    motor->config.velocity_min_rad_s,
                                    motor->config.velocity_max_rad_s,
                                    &clamped);
    float kp = MotorSDK_Clamp(command->kp,
                              motor->config.kp_min,
                              motor->config.kp_max,
                              &clamped);
    float kd = MotorSDK_Clamp(command->kd,
                              motor->config.kd_min,
                              motor->config.kd_max,
                              &clamped);
    float torque = MotorSDK_Clamp(command->torque_feedforward_nm,
                                  motor->config.torque_min_nm,
                                  motor->config.torque_max_nm,
                                  &clamped);

    uint32_t p = MotorSDK_FloatToUint(position,
                                      motor->config.position_min_rad,
                                      motor->config.position_max_rad,
                                      16U);
    uint32_t v = MotorSDK_FloatToUint(velocity,
                                      motor->config.velocity_min_rad_s,
                                      motor->config.velocity_max_rad_s,
                                      12U);
    uint32_t kp_uint = MotorSDK_FloatToUint(kp,
                                            motor->config.kp_min,
                                            motor->config.kp_max,
                                            12U);
    uint32_t kd_uint = MotorSDK_FloatToUint(kd,
                                            motor->config.kd_min,
                                            motor->config.kd_max,
                                            12U);
    uint32_t t = MotorSDK_FloatToUint(torque,
                                      motor->config.torque_min_nm,
                                      motor->config.torque_max_nm,
                                      12U);

    MotorSDK_ClearFrame(frame, motor->config.motor_id);
    frame->data[0] = (uint8_t)(p >> 8);
    frame->data[1] = (uint8_t)p;
    frame->data[2] = (uint8_t)(v >> 4);
    frame->data[3] = (uint8_t)(((v & 0x0FU) << 4) | (kp_uint >> 8));
    frame->data[4] = (uint8_t)kp_uint;
    frame->data[5] = (uint8_t)(kd_uint >> 4);
    frame->data[6] = (uint8_t)(((kd_uint & 0x0FU) << 4) | (t >> 8));
    frame->data[7] = (uint8_t)t;
    return clamped ? MOTOR_SDK_CLAMPED : MOTOR_SDK_OK;
}

MotorSDK_Result MotorSDK_BuildPositionSpeedFrame(
    const MotorSDK_Motor *motor,
    const MotorSDK_PositionSpeedCommand *command,
    MotorSDK_CANFrame *frame){
    MotorSDK_Result result = MotorSDK_CheckMotor(motor);
    if (result != MOTOR_SDK_OK){
        return result;
    }
    if ((command == 0) || (frame == 0)){
        return MOTOR_SDK_ERROR_ARGUMENT;
    }
    if (!MotorSDK_IsFinite(command->position_rad) ||
        !MotorSDK_IsFinite(command->velocity_limit_rad_s)){
        return MOTOR_SDK_ERROR_NOT_FINITE;
    }

    uint8_t clamped = 0U;
    float position = MotorSDK_Clamp(command->position_rad,
                                    motor->config.position_min_rad,
                                    motor->config.position_max_rad,
                                    &clamped);
    float velocity = MotorSDK_Clamp(command->velocity_limit_rad_s,
                                    0.0f,
                                    MotorSDK_VelocityLimitMaximum(&motor->config),
                                    &clamped);

    MotorSDK_ClearFrame(frame, 0x100U + motor->config.motor_id);
    MotorSDK_PackFloatLE(&frame->data[0], position);
    MotorSDK_PackFloatLE(&frame->data[4], velocity);
    return clamped ? MOTOR_SDK_CLAMPED : MOTOR_SDK_OK;
}

MotorSDK_Result MotorSDK_BuildSpeedFrame(const MotorSDK_Motor *motor,
                                         const MotorSDK_SpeedCommand *command,
                                         MotorSDK_CANFrame *frame){
    MotorSDK_Result result = MotorSDK_CheckMotor(motor);
    if (result != MOTOR_SDK_OK){
        return result;
    }
    if ((command == 0) || (frame == 0)){
        return MOTOR_SDK_ERROR_ARGUMENT;
    }
    if (!MotorSDK_IsFinite(command->velocity_rad_s)){
        return MOTOR_SDK_ERROR_NOT_FINITE;
    }

    uint8_t clamped = 0U;
    float velocity = MotorSDK_Clamp(command->velocity_rad_s,
                                    motor->config.velocity_min_rad_s,
                                    motor->config.velocity_max_rad_s,
                                    &clamped);
    MotorSDK_ClearFrame(frame, 0x200U + motor->config.motor_id);
    MotorSDK_PackFloatLE(&frame->data[0], velocity);
    return clamped ? MOTOR_SDK_CLAMPED : MOTOR_SDK_OK;
}

MotorSDK_Result MotorSDK_BuildPositionForceFrame(
    const MotorSDK_Motor *motor,
    const MotorSDK_PositionForceCommand *command,
    MotorSDK_CANFrame *frame){
    MotorSDK_Result result = MotorSDK_CheckMotor(motor);
    if (result != MOTOR_SDK_OK){
        return result;
    }
    if ((command == 0) || (frame == 0)){
        return MOTOR_SDK_ERROR_ARGUMENT;
    }
    if (!MotorSDK_IsFinite(command->position_rad) ||
        !MotorSDK_IsFinite(command->velocity_limit_rad_s) ||
        !MotorSDK_IsFinite(command->current_limit_ratio)){
        return MOTOR_SDK_ERROR_NOT_FINITE;
    }

    uint8_t clamped = 0U;
    float position = MotorSDK_Clamp(command->position_rad,
                                    motor->config.position_min_rad,
                                    motor->config.position_max_rad,
                                    &clamped);
    float velocity_max = MotorSDK_VelocityLimitMaximum(&motor->config);
    if (velocity_max > 655.35f){
        velocity_max = 655.35f;
    }
    float velocity = MotorSDK_Clamp(command->velocity_limit_rad_s,
                                    0.0f,
                                    velocity_max,
                                    &clamped);
    float current_ratio = MotorSDK_Clamp(command->current_limit_ratio,
                                         0.0f,
                                         1.0f,
                                         &clamped);
    uint16_t velocity_uint = (uint16_t)(velocity * 100.0f);
    uint16_t current_uint = (uint16_t)(current_ratio * 10000.0f);

    MotorSDK_ClearFrame(frame, 0x300U + motor->config.motor_id);
    MotorSDK_PackFloatLE(&frame->data[0], position);
    frame->data[4] = (uint8_t)velocity_uint;
    frame->data[5] = (uint8_t)(velocity_uint >> 8);
    frame->data[6] = (uint8_t)current_uint;
    frame->data[7] = (uint8_t)(current_uint >> 8);
    return clamped ? MOTOR_SDK_CLAMPED : MOTOR_SDK_OK;
}

MotorSDK_Result MotorSDK_DecodeFeedback(MotorSDK_Motor *motor,
                                        const MotorSDK_CANFrame *frame,
                                        MotorSDK_Feedback *feedback){
    MotorSDK_Result result = MotorSDK_CheckMotor(motor);
    if (result != MOTOR_SDK_OK){
        return result;
    }
    if (frame == 0){
        return MOTOR_SDK_ERROR_ARGUMENT;
    }
    if (frame->identifier != motor->config.master_id){
        return MOTOR_SDK_NOT_FOR_THIS_MOTOR;
    }
    if (frame->length != MOTOR_SDK_CAN_PAYLOAD_LENGTH){
        return MOTOR_SDK_ERROR_FRAME;
    }

    uint8_t motor_id = frame->data[0] & 0x0FU;
    if (motor_id != motor->config.motor_id){
        return MOTOR_SDK_NOT_FOR_THIS_MOTOR;
    }

    uint32_t position = ((uint32_t)frame->data[1] << 8) |
                        (uint32_t)frame->data[2];
    uint32_t velocity = ((uint32_t)frame->data[3] << 4) |
                        ((uint32_t)frame->data[4] >> 4);
    uint32_t torque = (((uint32_t)frame->data[4] & 0x0FU) << 8) |
                      (uint32_t)frame->data[5];

    MotorSDK_Feedback decoded;
    decoded.motor_id = motor_id;
    decoded.error = (MotorSDK_ProtocolError)(frame->data[0] >> 4);
    decoded.position_rad = MotorSDK_UintToFloat(
        position,
        motor->config.position_min_rad,
        motor->config.position_max_rad,
        16U);
    decoded.velocity_rad_s = MotorSDK_UintToFloat(
        velocity,
        motor->config.velocity_min_rad_s,
        motor->config.velocity_max_rad_s,
        12U);
    decoded.torque_nm = MotorSDK_UintToFloat(
        torque,
        motor->config.torque_min_nm,
        motor->config.torque_max_nm,
        12U);
    decoded.mos_temperature_c = frame->data[6];
    decoded.rotor_temperature_c = frame->data[7];

    motor->feedback = decoded;
    motor->feedback_valid = 1U;
    if (feedback != 0){
        *feedback = decoded;
    }
    return MOTOR_SDK_OK;
}

const MotorSDK_Feedback *MotorSDK_GetFeedback(const MotorSDK_Motor *motor){
    if ((motor == 0) || !motor->initialized || !motor->feedback_valid){
        return 0;
    }
    return &motor->feedback;
}
