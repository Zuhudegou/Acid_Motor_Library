#include "MIT_Protocol.h"

#include "FOC.h"
#include "UserData_Function.h"
#include <string.h>

MIT_PROTOCOL_STRUCT MIT;

static float MIT_Limit(float value, float value_max, float value_min){
    if (value > value_max){
        return value_max;
    }
    if (value < value_min){
        return value_min;
    }
    return value;
}

static float MIT_Abs(float value){
    return value < 0.0f ? -value : value;
}

static uint8_t MIT_IsSpecialCommand(const uint8_t data[8], uint8_t command){
    for (uint8_t i = 0; i < 7; i++){
        if (data[i] != 0xFFU){
            return 0U;
        }
    }
    return data[7] == command;
}

uint8_t MIT_Protocol_GetErrorCode(void){
    if (MIT.Timed_Out){
        return MIT_ERROR_COMM_LOST;
    }

    switch (FOC.status){
    case MOTOR_STATUS_OVERVOLTAGE:
        return MIT_ERROR_OVERVOLTAGE;
    case MOTOR_STATUS_UNDERVOLTAGE:
        return MIT_ERROR_UNDERVOLTAGE;
    case MOTOR_STATUS_OVERCURRENT:
        return MIT_ERROR_OVERCURRENT;
    case MOTOR_STATUS_OVERTEMPERATURE:
    case MOTOR_STATUS_UNDERTEMPERATURE:
        return MIT_ERROR_MOS_OVERHEAT;
    case MOTOR_STATUS_ENCODER_ERROR:
    case MOTOR_STATUS_SENSOR_ERROR:
    case MOTOR_STATUS_PWM_CALC_FAULT:
    case MOTOR_STATUS_EMERGENCY_STOP:
        return MIT_ERROR_OVERLOAD;
    default:
        return MIT_ERROR_NONE;
    }
}

static float MIT_GetTorqueConstant(void){
    float torque_constant = MIT_KT_OUT;
    if ((torque_constant < 1e-9f) && (torque_constant > -1e-9f)){
        torque_constant = 1.5f * (float)FOC.motor.Poles *
                          FOC.motor.identify.Flux;
    }
    return torque_constant;
}

static uint8_t MIT_TemperatureToUint8(float temperature){
    temperature = MIT_Limit(temperature, 255.0f, 0.0f);
    return (uint8_t)temperature;
}

static void MIT_ClearTargets(void){
    MIT.Command_Valid = 0U;
    MIT.Target_Torque = 0.0f;
    MIT.Target_Iq = 0.0f;
    FOC.foc.Target_Id = 0.0f;
    FOC.foc.Target_Iq = 0.0f;
    FOC.foc.Target_Speed = 0.0f;
    FOC.foc.Target_Pos = MIT.Real_Position;
    FOC.foc.Speed_in = 0.0f;
}

static void MIT_StopWithoutClearingFault(void){
    if (FOC.status < MOTOR_STATUS_OVERVOLTAGE){
        FOC.Func_Stop();
    }
}

static void MIT_ResetCommand(void){
    MIT.command.Position = MIT.Real_Position;
    MIT.command.Velocity = 0.0f;
    MIT.command.Kp = 0.0f;
    MIT.command.Kd = 0.0f;
    MIT.command.Torque = 0.0f;
    MIT.command.Current_Limit = 0.0f;
    MIT.Outer_Loop_Count = 0U;
}

static uint8_t MIT_DecodeFloatLE(const uint8_t data[4], float *value){
    uint32_t bits = (uint32_t)data[0] |
                    ((uint32_t)data[1] << 8) |
                    ((uint32_t)data[2] << 16) |
                    ((uint32_t)data[3] << 24);
    if ((bits & 0x7F800000UL) == 0x7F800000UL){
        return 0U;
    }
    memcpy(value, &bits, sizeof(bits));
    return 1U;
}

static float MIT_VelocityController(float reference, float feedback){
    #if CONFIG_CtrlVel == Control_LADRC
    FOC.transfer.Velocity.run.Ref = reference;
    FOC.transfer.Velocity.run.Fbk = feedback;
    Ladrc_Loop(&FOC.transfer.Velocity);
    #elif CONFIG_CtrlVel == Control_SMC
    FOC.transfer.Velocity.run.Ref = reference;
    FOC.transfer.Velocity.run.Fbk = feedback;
    SMC_Loop(&FOC.transfer.Velocity);
    #elif CONFIG_CtrlVel == Control_STA
    FOC.transfer.Velocity.run.Ref = reference;
    FOC.transfer.Velocity.run.Fbk = feedback;
    STA_Loop(&FOC.transfer.Velocity);
    #else
    FOC.transfer.Velocity.run.Ref = reference;
    FOC.transfer.Velocity.run.Fbk = feedback;
    PID_Loop(&FOC.transfer.Velocity);
    #endif
    return FOC.transfer.Velocity.run.Output;
}

static float MIT_PositionController(float reference, float feedback){
    #if CONFIG_CtrlPos == Control_LADRC
    FOC.transfer.Position.run.Ref = reference;
    FOC.transfer.Position.run.Fbk = feedback;
    Ladrc_Loop(&FOC.transfer.Position);
    #elif CONFIG_CtrlPos == Control_SMC
    FOC.transfer.Position.run.Ref = reference;
    FOC.transfer.Position.run.Fbk = feedback;
    SMC_Loop(&FOC.transfer.Position);
    #elif CONFIG_CtrlPos == Control_STA
    FOC.transfer.Position.run.Ref = reference;
    FOC.transfer.Position.run.Fbk = feedback;
    STA_Loop(&FOC.transfer.Position);
    #else
    FOC.transfer.Position.run.Ref = reference;
    FOC.transfer.Position.run.Fbk = feedback;
    PID_Loop(&FOC.transfer.Position);
    #endif
    return FOC.transfer.Position.run.Output;
}

uint8_t MIT_Protocol_SetControlMode(uint8_t mode){
#if CONFIG_MIT
    if ((mode < MIT_MODE_MIT) || (mode > MIT_MODE_POSITION_FORCE)){
        return 0U;
    }
    if (MIT.Control_Mode == mode){
        return 1U;
    }

    MIT.Enabled = 0U;
    MIT.Timed_Out = 0U;
    MIT.Timeout_Count = 0U;
    MIT_ClearTargets();
    MIT_ResetCommand();
    MIT.Control_Mode = mode;
    return 1U;
#else
    (void)mode;
    return 0U;
#endif
}

uint32_t MIT_Protocol_GetCommandIdentifier(void){
    switch (MIT.Control_Mode){
    case MIT_MODE_POSITION_SPEED:
        return MIT_POSITION_SPEED_ID;
    case MIT_MODE_SPEED:
        return MIT_SPEED_ID;
    case MIT_MODE_POSITION_FORCE:
        return MIT_POSITION_FORCE_ID;
    default:
        return MIT_CAN_ID;
    }
}

uint8_t MIT_Protocol_IsSupportedIdentifier(uint32_t identifier){
    return (identifier == MIT_CAN_ID) ||
           (identifier == MIT_POSITION_SPEED_ID) ||
           (identifier == MIT_SPEED_ID) ||
           (identifier == MIT_POSITION_FORCE_ID);
}

uint32_t MIT_FloatToUint(float value,
                         float value_min,
                         float value_max,
                         uint8_t bits){
    uint32_t integer_max = (1UL << bits) - 1UL;
    value = MIT_Limit(value, value_max, value_min);
    return (uint32_t)(((value - value_min) * (float)integer_max) /
                      (value_max - value_min));
}

float MIT_UintToFloat(uint32_t value,
                      float value_min,
                      float value_max,
                      uint8_t bits){
    uint32_t integer_max = (1UL << bits) - 1UL;
    return ((float)value * (value_max - value_min) /
            (float)integer_max) + value_min;
}

void MIT_Protocol_Init(void){
    MIT.command.Position = 0.0f;
    MIT.command.Velocity = 0.0f;
    MIT.command.Kp = 0.0f;
    MIT.command.Kd = 0.0f;
    MIT.command.Torque = 0.0f;
    MIT.command.Current_Limit = 0.0f;

    MIT.Real_Position = 0.0f;
    MIT.Real_Velocity = 0.0f;
    MIT.Real_Torque = 0.0f;
    MIT.Target_Torque = 0.0f;
    MIT.Target_Iq = 0.0f;
    MIT.Torque_Constant = 0.0f;
    MIT.Position_Zero = 0.0f;
    MIT.Position_Unwrapped = 0.0f;
    MIT.Position_Last = 0.0f;
    MIT.Timeout_Count = 0U;
    MIT.Control_Mode = MIT_DEFAULT_CONTROL_MODE;
    MIT.Enabled = 0U;
    MIT.Command_Valid = 0U;
    MIT.Timed_Out = 0U;
    MIT.Position_Initialized = 0U;
    MIT.Outer_Loop_Count = 0U;
}

uint8_t MIT_Protocol_Receive(uint32_t identifier,
                            const uint8_t data[8],
                            uint8_t length){
#if CONFIG_MIT
    if ((data == 0) || (length != 8U) ||
        !MIT_Protocol_IsSupportedIdentifier(identifier)){
        return 0U;
    }

    if ((identifier == MIT_CAN_ID) &&
        MIT_IsSpecialCommand(data, MIT_SPECIAL_ENABLE)){
        MIT.Enabled = 1U;
        MIT_ClearTargets();
        MIT_ResetCommand();
        MIT.Timed_Out = 0U;
        MIT.Timeout_Count = 0U;
        if (FOC.status == MOTOR_STATUS_STANDBY){
            FOC.Func_Start();
        }
        return 1U;
    }

    if ((identifier == MIT_CAN_ID) &&
        MIT_IsSpecialCommand(data, MIT_SPECIAL_DISABLE)){
        MIT.Enabled = 0U;
        MIT_ClearTargets();
        MIT_StopWithoutClearingFault();
        return 1U;
    }

    if ((identifier == MIT_CAN_ID) &&
        MIT_IsSpecialCommand(data, MIT_SPECIAL_CLEAR_ERROR)){
        MIT.Enabled = 0U;
        MIT_ClearTargets();
        MIT.Timed_Out = 0U;
        MIT.Timeout_Count = 0U;
        FOC.Func_Stop();
        return 1U;
    }

    if ((identifier == MIT_CAN_ID) &&
        MIT_IsSpecialCommand(data, MIT_SPECIAL_SAVE_ZERO)){
        MIT.Position_Zero = MIT.Position_Unwrapped;
        MIT.Real_Position = 0.0f;
        MIT.command.Position = 0.0f;
        User_MIT_SaveZero(MIT.Position_Zero);
        return 1U;
    }

    if ((!MIT.Enabled) ||
        (identifier != MIT_Protocol_GetCommandIdentifier())){
        return 0U;
    }

    MIT.Command_Valid = 0U;

    switch (MIT.Control_Mode){
    case MIT_MODE_POSITION_SPEED:{
        float position;
        float velocity;
        if (!MIT_DecodeFloatLE(&data[0], &position) ||
            !MIT_DecodeFloatLE(&data[4], &velocity)){
            MIT_ClearTargets();
            return 0U;
        }
        MIT.command.Position = position;
        MIT.command.Velocity = MIT_Abs(velocity);
        MIT.command.Current_Limit = 1.0f;
        break;
    }

    case MIT_MODE_SPEED:{
        float velocity;
        if (!MIT_DecodeFloatLE(&data[0], &velocity)){
            MIT_ClearTargets();
            return 0U;
        }
        MIT.command.Velocity = velocity;
        MIT.command.Current_Limit = 1.0f;
        break;
    }

    case MIT_MODE_POSITION_FORCE:{
        float position;
        if (!MIT_DecodeFloatLE(&data[0], &position)){
            MIT_ClearTargets();
            return 0U;
        }
        uint32_t velocity = (uint32_t)data[4] |
                            ((uint32_t)data[5] << 8);
        uint32_t current = (uint32_t)data[6] |
                           ((uint32_t)data[7] << 8);
        MIT.command.Position = position;
        MIT.command.Velocity = (float)velocity * 0.01f;
        MIT.command.Current_Limit = MIT_Limit((float)current * 0.0001f,
                                               1.0f,
                                               0.0f);
        break;
    }

    default:{
        uint32_t position = ((uint32_t)data[0] << 8) |
                            (uint32_t)data[1];
        uint32_t velocity = ((uint32_t)data[2] << 4) |
                            ((uint32_t)data[3] >> 4);
        uint32_t kp = (((uint32_t)data[3] & 0x0FU) << 8) |
                      (uint32_t)data[4];
        uint32_t kd = ((uint32_t)data[5] << 4) |
                      ((uint32_t)data[6] >> 4);
        uint32_t torque = (((uint32_t)data[6] & 0x0FU) << 8) |
                          (uint32_t)data[7];

        MIT.command.Position = MIT_UintToFloat(position,
                                                MIT_POSITION_MIN,
                                                MIT_POSITION_MAX,
                                                16U);
        MIT.command.Velocity = MIT_UintToFloat(velocity,
                                                MIT_VELOCITY_MIN,
                                                MIT_VELOCITY_MAX,
                                                12U);
        MIT.command.Kp = MIT_UintToFloat(kp,
                                         MIT_KP_MIN,
                                         MIT_KP_MAX,
                                         12U);
        MIT.command.Kd = MIT_UintToFloat(kd,
                                         MIT_KD_MIN,
                                         MIT_KD_MAX,
                                         12U);
        MIT.command.Torque = MIT_UintToFloat(torque,
                                             MIT_TORQUE_MIN,
                                             MIT_TORQUE_MAX,
                                             12U);
        MIT.command.Current_Limit = 1.0f;
        break;
    }
    }
    MIT.Timeout_Count = 0U;
    MIT.Timed_Out = 0U;
    MIT.Command_Valid = 1U;
    return 1U;
#else
    (void)identifier;
    (void)data;
    (void)length;
    return 0U;
#endif
}

void MIT_Protocol_FeedbackUpdate(void){
#if CONFIG_MIT
    float position = FOC.encoder.Real_Pos;
    if (!MIT.Position_Initialized){
        MIT.Position_Last = position;
        MIT.Position_Unwrapped = position;
        MIT.Position_Initialized = 1U;
    }
    else{
        float delta = position - MIT.Position_Last;
        if (delta > Value_PI){
            delta -= Value_2PI;
        }
        else if (delta < -Value_PI){
            delta += Value_2PI;
        }
        MIT.Position_Unwrapped += delta;
        MIT.Position_Last = position;
    }

    MIT.Real_Position = MIT.Position_Unwrapped - MIT.Position_Zero;
    MIT.Real_Velocity = FOC.encoder.Real_Speed;
    MIT.Torque_Constant = MIT_GetTorqueConstant();
    MIT.Real_Torque = MIT.Torque_Constant * FOC.current.Real_Iq;
#endif
}

static void MIT_ApplyTargetIq(float target_iq, float current_limit){
    current_limit = MIT_Limit(MIT_Abs(current_limit),
                              FOC.safe.Qcur_MAX,
                              0.0f);
    target_iq = MIT_Limit(target_iq, current_limit, -current_limit);

    MIT.Target_Iq = target_iq;
    MIT.Torque_Constant = MIT_GetTorqueConstant();
    MIT.Target_Torque = target_iq * MIT.Torque_Constant;
    FOC.foc.Target_Id = 0.0f;
    FOC.foc.Target_Iq = target_iq;
}

static void MIT_ControlImpedance(void){
    float target_torque = MIT.command.Kp *
                          (MIT.command.Position - MIT.Real_Position) +
                          MIT.command.Kd *
                          (MIT.command.Velocity - MIT.Real_Velocity) +
                          MIT.command.Torque;
    target_torque = MIT_Limit(target_torque,
                              MIT_TORQUE_MAX,
                              MIT_TORQUE_MIN);

    float torque_constant = MIT_GetTorqueConstant();
    float target_iq = 0.0f;
    if ((torque_constant > 1e-9f) || (torque_constant < -1e-9f)){
        target_iq = target_torque / torque_constant;
    }
    MIT_ApplyTargetIq(target_iq, FOC.safe.Qcur_MAX);
}

static void MIT_ControlSpeed(void){
    uint16_t response = FOC.value.Response;
    if (response == 0U){
        response = 1U;
    }

    FOC.foc.Target_Speed = MIT_Limit(MIT.command.Velocity,
                                      MIT_V_MAX,
                                      -MIT_V_MAX);
    FOC.foc.Speed_in = FOC.foc.Target_Speed;

    MIT.Outer_Loop_Count++;
    if (MIT.Outer_Loop_Count >= response){
        float target_iq = MIT_VelocityController(FOC.foc.Speed_in,
                                                  MIT.Real_Velocity);
        MIT_ApplyTargetIq(target_iq, FOC.safe.Qcur_MAX);
        MIT.Outer_Loop_Count = 0U;
    }
}

static void MIT_ControlPosition(uint8_t force_limit_enabled){
    uint16_t response = FOC.value.Response;
    if (response == 0U){
        response = 1U;
    }
    uint16_t position_divider = response * response;
    float velocity_limit = MIT_Limit(MIT_Abs(MIT.command.Velocity),
                                      MIT_V_MAX,
                                      0.0f);

    FOC.foc.Target_Pos = MIT.command.Position;
    MIT.Outer_Loop_Count++;
    if (MIT.Outer_Loop_Count >= position_divider){
        float target_speed = MIT_PositionController(MIT.command.Position,
                                                     MIT.Real_Position);
        FOC.foc.Target_Speed = MIT_Limit(target_speed,
                                         velocity_limit,
                                         -velocity_limit);
        FOC.foc.Speed_in = FOC.foc.Target_Speed;
        MIT.Outer_Loop_Count = 0U;
    }

    float current_limit = FOC.safe.Qcur_MAX;
    if (force_limit_enabled){
        current_limit *= MIT.command.Current_Limit;
    }

    if ((MIT.Outer_Loop_Count % response) == 0U){
        float target_iq = MIT_VelocityController(FOC.foc.Speed_in,
                                                  MIT.Real_Velocity);
        MIT_ApplyTargetIq(target_iq, current_limit);
    }
    else{
        MIT_ApplyTargetIq(MIT.Target_Iq, current_limit);
    }
}

void MIT_Protocol_ControlLoop(void){
#if CONFIG_MIT
    if (FOC.status >= MOTOR_STATUS_OVERVOLTAGE){
        MIT_ClearTargets();
        return;
    }

    if ((!MIT.Enabled) || (!MIT.Command_Valid) || MIT.Timed_Out){
        return;
    }

    switch (MIT.Control_Mode){
    case MIT_MODE_POSITION_SPEED:
        MIT_ControlPosition(0U);
        break;
    case MIT_MODE_SPEED:
        MIT_ControlSpeed();
        break;
    case MIT_MODE_POSITION_FORCE:
        MIT_ControlPosition(1U);
        break;
    default:
        MIT_ControlImpedance();
        break;
    }
#endif
}

void MIT_Protocol_1msLoop(void){
#if CONFIG_MIT
    if (MIT.Enabled && MIT.Command_Valid){
        if (MIT.Timeout_Count < MIT_COMMAND_TIMEOUT_MS){
            MIT.Timeout_Count++;
        }
        if (MIT.Timeout_Count >= MIT_COMMAND_TIMEOUT_MS){
            MIT.Timed_Out = 1U;
            MIT_ClearTargets();
            #if MIT_TIMEOUT_STOP
            MIT.Enabled = 0U;
            FOC.Func_Stop();
            #endif
        }
    }
#endif
}

void MIT_Protocol_PackFeedback(uint8_t data[8]){
#if CONFIG_MIT
    uint32_t position = MIT_FloatToUint(MIT.Real_Position,
                                        MIT_POSITION_MIN,
                                        MIT_POSITION_MAX,
                                        16U);
    uint32_t velocity = MIT_FloatToUint(MIT.Real_Velocity,
                                        MIT_VELOCITY_MIN,
                                        MIT_VELOCITY_MAX,
                                        12U);
    uint32_t torque = MIT_FloatToUint(MIT.Real_Torque,
                                      MIT_TORQUE_MIN,
                                      MIT_TORQUE_MAX,
                                      12U);
    float mos_temperature = FOC.foc.Real_Temp;
    float rotor_temperature = User_RotorTemperature_DataGet();
    if (rotor_temperature == Value_N_INF){
        rotor_temperature = mos_temperature;
    }

    data[0] = (uint8_t)((MIT_Protocol_GetErrorCode() << 4) |
                        (MIT_CAN_ID & 0x0FU));
    data[1] = (uint8_t)(position >> 8);
    data[2] = (uint8_t)position;
    data[3] = (uint8_t)(velocity >> 4);
    data[4] = (uint8_t)(((velocity & 0x0FU) << 4) |
                        (torque >> 8));
    data[5] = (uint8_t)torque;
    data[6] = MIT_TemperatureToUint8(mos_temperature);
    data[7] = MIT_TemperatureToUint8(rotor_temperature);
#else
    for (uint8_t i = 0; i < 8U; i++){
        data[i] = 0U;
    }
#endif
}
