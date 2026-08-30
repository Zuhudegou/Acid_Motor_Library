#include "FOC.h"

/* USER CODE BEGIN Includes */
// 电机控制User用户设置声明
#include "UserData_Function.h"
#include "UserData_Motor.h"
#include "UserData_Parameter.h"
#include "UserData_UserControl.h"
#include "MIT_FDCAN.h"
/* USER CODE END Includes */

/**
 * @description: 1.电机控制函数实现(FOC.*使用)
 * @reminder: FOC控制函数接口的使用方法，如下：
 * @reminder: 启动电机  FOC.Func_Start()
 * @reminder: 停止电机  FOC.Func_Stop()
 * @reminder: 设计电压  FOC.Func_Set_Uq(1.68f)
 * @reminder: 设计电流  FOC.Func_Set_Iq(2.68f)
 * @reminder: 设计转速  FOC.Func_Set_Velocity(16.8f)
 * @reminder: 设计位置  FOC.Func_Set_Position(26.8f)
 * @reminder: 数据打印  FOC.Func_Set_TXdata(0,FOC.status)
 * @param {void} *ctrl
 * @return {*}
 */
static void Function_Start(void);
static void Function_Stop(void);
static void Function_SetUd(float ud);
static void Function_SetUq(float uq);
static void Function_SetId(float id);
static void Function_SetIq(float iq);
static void Function_SetVelocity(float speed);
static void Function_SetPosition(float pos);
static void Function_SetTXdata(uint8_t ch,float data);

// 电机控制核心结构体设计
FOC_System_STRUCT FOC = {
    .Func_Start = Function_Start,
    .Func_Stop = Function_Stop,
    .Func_Set_Ud = Function_SetUd,
    .Func_Set_Uq = Function_SetUq,
    .Func_Set_Id = Function_SetId,
    .Func_Set_Iq = Function_SetIq,
    .Func_Set_Velocity = Function_SetVelocity,
    .Func_Set_Position = Function_SetPosition,
    .Func_Set_TXdata = Function_SetTXdata
};

// ============================= FOC版本代码(仅声明) ============================
/**
 * @description: 2.Transfer传递函数的离散化运算，采用双线性变换
 * @param {PID_STRUCT} *pid (控制)PID闭环控制系统运算
 * @param {LADRC_STRUCT} *ladrc (控制)LADRC线自抗扰运算
 * @param {SMC_STRUCT} *smc (控制)SMC传统滑模控制运算
 * @param {STA_STRUCT} *sta (控制)STA超螺旋简化滑模运算
 * @param {BPF_STRUCT} *bpf (滤波)BPF二阶巴特沃斯低通滤波
 * @param {PLL_STRUCT} *pll (估算)PLL速度跟踪锁相环
 * @param {DOB_STRUCT} *dob (观测器)DOB超螺旋滑模扰动观测器
 * @return {*}
 */
static void Transfer_PID_Loop(void *pid,float Ref,float Fbk);
static void Transfer_Ladrc_Loop(void *ladrc,float Ref,float Fbk);
static void Transfer_SMC_Loop(void *smc,float Ref,float Fbk);
static void Transfer_STA_Loop(void *sta,float Ref,float Fbk);
static void (*const CtrlFunc_Tick[])(void *, float, float)={
    /*CtrlFunc_Tick[CONFIG_CtrlVel](*)或者CtrlFunc_Tick[CONFIG_CtrlPos](*)*/
    Transfer_PID_Loop,              // PID
    Transfer_Ladrc_Loop,            // Ladrc
    Transfer_SMC_Loop,              // SMC
    Transfer_STA_Loop               // STA
};
static void Transfer_LPF_Loop(LPF_STRUCT *lpf,float input);
static void Transfer_TPNF_Loop(TPNF_STRUCT *tpnf,float input);
static void Transfer_DOB_Loop(DOB_STRUCT *dob,float Iq,float Wm);
static void Transfer_LTD_Loop(LTD_STRUCT *ltd, 
                            float *output, 
                            float input);
static void Transfer_PLL_Loop(PLL_STRUCT *pll,
                            uint8_t mode,
                            uint8_t Poles,
                            float input_Rad);
static void Transfer_Hall_Loop(FOC_System_STRUCT *foc, 
                            PLL_STRUCT *pll);
static void Transfer_HFI_Loop(FOC_System_STRUCT *foc, 
                            PLL_STRUCT *pll);
static void Transfer_SMO_Loop(FOC_System_STRUCT *foc, 
                            PLL_STRUCT *pll);
static void Transfer_NLFO_Loop(FOC_System_STRUCT *foc, 
                            PLL_STRUCT *pll);
/**
 * @description: 3.Transfer传递函数的初始化
 * @param {FOC_System_STRUCT} *foc
 * @return {*}
 */
static void Transfer_Init(FOC_System_STRUCT *foc);
static void Transfer_Null(FOC_System_STRUCT *foc);
/**
 * @description: 4.Offset内部静态函数声明
 * @param {FOC_System_STRUCT} *foc
 * @return {*}
 */
static void Offset_Current_Init(FOC_System_STRUCT *foc);
static void Offset_Rad_Encoder(FOC_System_STRUCT *foc);
static void Offset_Rad_Hall(FOC_System_STRUCT *foc);
static void Offset_HFI_Init(FOC_System_STRUCT *foc);
static void (*const Offset_Rad_Init[])(FOC_System_STRUCT *)={
    /* 如此使用:Offset_Rad_Tick[CONFIG_MODE](*) */
    Transfer_Null,                  // MODE_VF_OPENLOOP    = 0
    Transfer_Null,                  // MODE_IF_OPENLOOP    = 1
    Offset_Rad_Encoder,             // MODE_Voltag_OPEN    = 2
    Offset_Rad_Encoder,             // MODE_Current_SINGLE = 3
    Offset_Rad_Encoder,             // MODE_VelCur_DOUBLE  = 4
    Offset_Rad_Encoder,             // MODE_PosVelCur_THREE= 5
    Offset_Rad_Hall,                // MODE_Sensor_Hall    = 6
    Transfer_Null,                  // MODE_Sensorless_HFI = 7
    Transfer_Null,                  // MODE_Sensorless_SMO = 8
    Transfer_Null,                  // MODE_Sensorless_NLFO= 9
    Transfer_Null,                  // MODE_Sensorless_HS  = 10
    Transfer_Null,                  // MODE_Sensorless_HN  = 11
    Offset_Rad_Hall,                // MODE_Sensorless_AS  = 12
    Offset_Rad_Hall,                // MODE_Sensorless_AN  = 13
    Offset_Rad_Encoder,             // MODE_Debug_HFI      = 14
    Offset_Rad_Encoder,             // MODE_Debug_SMO      = 15
    Offset_Rad_Encoder,             // MODE_Debug_NLFO     = 16
    Offset_Rad_Encoder,             // MODE_Debug_HS       = 17
    Offset_Rad_Encoder              // MODE_Debug_HN       = 18
};
/**
 * @description: 5.Current内部静态函数声明
 * @param {FOC_System_STRUCT} *foc
 * @return {*}
 */
static void Init_Tick(FOC_System_STRUCT *foc);
/**
 * @description: 6.Current内部静态函数声明
 * @param {FOC_System_STRUCT} *foc
 * @return {*}
 */
static void Current_ReadIabc(FOC_System_STRUCT *foc);
static void Current_Tick(FOC_System_STRUCT *foc);
/**
 * @description: 7.Encoder内部静态函数声明
 * @param {FOC_System_STRUCT} *foc
 * @return {*}
 */
static void Encoder_Virtual_Angle(FOC_System_STRUCT *foc);
static void Encoder_Sensor_Encoder(FOC_System_STRUCT *foc);
static void Encoder_Sensor_Hall(FOC_System_STRUCT *foc);
static void Encoder_Sensorless_HFI(FOC_System_STRUCT *foc);
static void Encoder_Sensorless_Tall(FOC_System_STRUCT *foc);
static void Encoder_Sensorless_Normal(FOC_System_STRUCT *foc);
static void (*const Encoder_Tick[])(FOC_System_STRUCT*)={
    /*这里注意“枚举变量”的边界, Encoder_Tick[CONFIG_MODE](foc)使用*/
    Encoder_Virtual_Angle,          // MODE_VF_OPENLOOP    = 0
    Encoder_Virtual_Angle,          // MODE_IF_OPENLOOP    = 1
    Encoder_Sensor_Encoder,         // MODE_Voltag_OPEN    = 2
    Encoder_Sensor_Encoder,         // MODE_Current_SINGLE = 3
    Encoder_Sensor_Encoder,         // MODE_VelCur_DOUBLE  = 4
    Encoder_Sensor_Encoder,         // MODE_PosVelCur_THREE= 5
    Encoder_Sensor_Hall,            // MODE_Sensor_Hall    = 6
    Encoder_Sensorless_HFI,         // MODE_Sensorless_HFI = 7
    Encoder_Sensorless_Tall,        // MODE_Sensorless_SMO = 8
    Encoder_Sensorless_Tall,        // MODE_Sensorless_NLFO= 9
    Encoder_Sensorless_Normal,      // MODE_Sensorless_HS  = 10
    Encoder_Sensorless_Normal,      // MODE_Sensorless_HN  = 11
    Encoder_Sensorless_Normal,      // MODE_Sensorless_AS  = 12
    Encoder_Sensorless_Normal,      // MODE_Sensorless_AN  = 13
    Encoder_Sensor_Encoder,         // MODE_Debug_HFI      = 14
    Encoder_Sensor_Encoder,         // MODE_Debug_SMO      = 15
    Encoder_Sensor_Encoder,         // MODE_Debug_NLFO     = 16
    Encoder_Sensor_Encoder,         // MODE_Debug_HS       = 17
    Encoder_Sensor_Encoder          // MODE_Debug_HN       = 18
};
/**
 * @description: 8.Debug模式下的Encoder特殊处理函数声明
 * @param {FOC_System_STRUCT} *foc
 * @return {*}
 */
static void Debug_Sensorless_HFI(FOC_System_STRUCT *foc);
static void Debug_Sensorless_Tall(FOC_System_STRUCT *foc);
static void Debug_Sensorless_Normal(FOC_System_STRUCT *foc);
static void (*const Debug_Tick[])(FOC_System_STRUCT*)={
    /*这里注意“枚举变量”的边界, Encoder_Tick[CONFIG_MODE](foc)使用*/
    Transfer_Null,                  // MODE_VF_OPENLOOP    = 0
    Transfer_Null,                  // MODE_IF_OPENLOOP    = 1
    Transfer_Null,                  // MODE_Voltag_OPEN    = 2
    Transfer_Null,                  // MODE_Current_SINGLE = 3
    Transfer_Null,                  // MODE_VelCur_DOUBLE  = 4
    Transfer_Null,                  // MODE_PosVelCur_THREE= 5
    Transfer_Null,                  // MODE_Sensor_Hall    = 6
    Transfer_Null,                  // MODE_Sensorless_HFI = 7
    Transfer_Null,                  // MODE_Sensorless_SMO = 8
    Transfer_Null,                  // MODE_Sensorless_NLFO= 9
    Transfer_Null,                  // MODE_Sensorless_HS  = 10
    Transfer_Null,                  // MODE_Sensorless_HN  = 11
    Transfer_Null,                  // MODE_Sensorless_AS  = 12
    Transfer_Null,                  // MODE_Sensorless_AN  = 13
    Debug_Sensorless_HFI,           // MODE_Debug_HFI      = 14
    Debug_Sensorless_Tall,          // MODE_Debug_SMO      = 15
    Debug_Sensorless_Tall,          // MODE_Debug_NLFO     = 16
    Debug_Sensorless_Normal,        // MODE_Debug_HS       = 17
    Debug_Sensorless_Normal         // MODE_Debug_HN       = 18
};
/**
 * @description: 9.Control运算及其模式切换
 * @param {FOC_System_STRUCT} *foc
 * @return {*}
 */
static void Control_Current_SINGLE(FOC_System_STRUCT *foc);
static void Control_VelCur_DOUBLE(FOC_System_STRUCT *foc);
static void Control_PosVelCur_THREE(FOC_System_STRUCT *foc);
static void (*const Control_Tick[])(FOC_System_STRUCT*)={
    /*这里注意“枚举变量”的边界, Control_Tick[CONFIG_MODE](foc)使用*/
    Transfer_Null,                  // MODE_VF_OPENLOOP    = 0
    Control_Current_SINGLE,         // MODE_IF_OPENLOOP    = 1
    Transfer_Null,                  // MODE_Voltag_OPEN    = 2
    Control_Current_SINGLE,         // MODE_Current_SINGLE = 3
    Control_VelCur_DOUBLE,          // MODE_VelCur_DOUBLE  = 4
    Control_PosVelCur_THREE,        // MODE_PosVelCur_THREE= 5
    Control_VelCur_DOUBLE,          // MODE_Sensor_Hall    = 6
    Control_VelCur_DOUBLE,          // MODE_Sensorless_HFI = 7
    Control_VelCur_DOUBLE,          // MODE_Sensorless_SMO = 8
    Control_VelCur_DOUBLE,          // MODE_Sensorless_NLFO= 9
    Control_VelCur_DOUBLE,          // MODE_Sensorless_HS  = 10
    Control_VelCur_DOUBLE,          // MODE_Sensorless_HN  = 11
    Control_VelCur_DOUBLE,          // MODE_Sensorless_AS  = 12
    Control_VelCur_DOUBLE,          // MODE_Sensorless_AN  = 13
    Control_VelCur_DOUBLE,          // MODE_Debug_HFI      = 14
    Control_VelCur_DOUBLE,          // MODE_Debug_SMO      = 15
    Control_VelCur_DOUBLE,          // MODE_Debug_NLFO     = 16
    Control_VelCur_DOUBLE,          // MODE_Debug_HS       = 17
    Control_VelCur_DOUBLE           // MODE_Debug_HN       = 18
};
/**
 * @description: 10.PWM占空比计算及其模式切换
 * @param {FOC_System_STRUCT} *foc
 * @param {float} U_alpha
 * @param {float} U_beta
 * @return {*}
 */
static void PWM_Tick(FOC_System_STRUCT *foc,
                    float U_alpha,
                    float U_beta);
/**
 * @description: 11.Status判断并切换状态机
 * @param {FOC_System_STRUCT} *foc
 * @param {uint32_t} *count
 * @return {*}
 */
static void Status_Protection_Loop(FOC_System_STRUCT *foc);
static void Status_Zero(FOC_System_STRUCT *foc);
static void Status_Switch_STANDBY(FOC_System_STRUCT *foc,uint32_t *count);
static void Status_Switch_IDLE(FOC_System_STRUCT *foc,uint32_t *count);
static void Status_VBUS_OVERVOLTAGE(FOC_System_STRUCT *foc,uint32_t *count);
static void Status_VBUS_UNDERVOLTAGE(FOC_System_STRUCT *foc,uint32_t *count);
static void Status_Temp_OVERTEMPERATURE(FOC_System_STRUCT *foc,uint32_t *count);
static void Status_Temp_UNDERTEMPERATURE(FOC_System_STRUCT *foc,uint32_t *count);
static void Status_Current_OVERCURRENT(FOC_System_STRUCT *foc,uint32_t *count);
static void Status_Switch_Loop(FOC_System_STRUCT *foc);
static void Status_RUN_Loop(FOC_System_STRUCT *foc);
/**
 * @description: 12.FOC主循环函数，包含状态机和控制器运算
 * @param {FOC_System_STRUCT} *foc
 * @return {*}
 */
static void Printf_Normal_Loop(FOC_System_STRUCT *foc);
static void Printf_Debug_Loop(FOC_System_STRUCT *foc);
static void Printf_Cogging_Loop(void);
/**
 * @description: 13.FOC...Loop电机运算汇总
 * @param {FOC_System_STRUCT} *foc
 * @return {*}
 */
static void Calculate_High_Loop(FOC_System_STRUCT *foc);
static void Calculate_Low_Loop(FOC_System_STRUCT *foc);
static void Calculate_main_Loop(FOC_System_STRUCT *foc);


// =============================== 电机控制函数(接口) =============================
// Function控制接口->启动电机
static void Function_Start(void){
    FOC.status = MOTOR_STATUS_START;
}

// Function控制接口->停止电机
static void Function_Stop(void){
    FOC.status = MOTOR_STATUS_STANDBY;
}

// Function控制接口->设计目标电压
static void Function_SetUd(float ud){
    FOC.foc.Ud_in = ud;
}

// Function控制接口->设计目标电流
static void Function_SetUq(float uq){
    FOC.foc.Uq_in = uq;
}

// Function控制接口->设计目标电压
static void Function_SetId(float id){
    FOC.foc.Target_Id = id;
}

// Function控制接口->设计目标电流
static void Function_SetIq(float iq){
    FOC.foc.Target_Iq = iq;
}

// Function控制接口->设计目标转速
static void Function_SetVelocity(float speed){
    FOC.foc.Target_Speed = speed;
}

// Function控制接口->设计目标位置
static void Function_SetPosition(float pos){
    FOC.foc.Target_Pos = pos;
}

// Function控制接口->数据打印
static void Function_SetTXdata(uint8_t ch,float data){
    FOC.txdata.fdata[ch] = data;
}

// ============================= FOC版本代码(仅实现) ============================
// Transfer运算_PID运算
static void Transfer_PID_Loop(void *pid,float Ref,float Fbk){
    PID_STRUCT *p = (PID_STRUCT*)pid;
    p->run.Ref = Ref;
    p->run.Fbk = Fbk;
    PID_Loop(p);
    // 输出pid->run.Output;
}

// Transfer运算_LADRC控制
static void Transfer_Ladrc_Loop(void *ladrc,float Ref,float Fbk){
    LADRC_STRUCT *p = (LADRC_STRUCT*)ladrc;
    p->run.Ref = Ref;
    p->run.Fbk = Fbk;
    Ladrc_Loop(p);
    // 输出ladrc->run.Output;
}

// Transfer运算_SMC控制
static void Transfer_SMC_Loop(void *smc,float Ref,float Fbk){
    SMC_STRUCT *p = (SMC_STRUCT*)smc;
    p->run.Ref = Ref;
    p->run.Fbk = Fbk;
    SMC_Loop(p);
    // 输出SMC->run.Output;
}

// Transfer运算_STA控制
static void Transfer_STA_Loop(void *sta,float Ref,float Fbk){
    STA_STRUCT *p = (STA_STRUCT*)sta;
    p->run.Ref = Ref;
    p->run.Fbk = Fbk;
    STA_Loop(p);
    // 输出sta->run.Output;
}

// Transfer运算_二阶巴特沃斯低通滤波器
static void Transfer_LPF_Loop(LPF_STRUCT *lpf,float input){
    lpf->filter.Input = input;
    LPF_Loop(lpf);
    // 输出lpf->filter.Output;
}

// Transfer运算_陷波滤波器
static void Transfer_TPNF_Loop(TPNF_STRUCT *tpnf,float input){
    tpnf->filter.Input = input;
    TPNF_Loop(tpnf);
    // 输出bsf->filter.Output;
}

// Transfer运算_超螺旋滑模扰动观测器
static void Transfer_DOB_Loop(DOB_STRUCT *dob,float Iq,float Wm){
    #if CONFIG_DOB
    dob->smdo.Input_Iq = Iq;
    dob->smdo.Input_Wm = Wm;
    DOB_Loop(dob);
    #endif // CONFIG_DOB
    // 输出dob->smdo.Output_Fd;
    // 输出dob->smdo.Output_Wm;
}

// Transfer运算_LTD最速控制
static void Transfer_LTD_Loop(LTD_STRUCT *ltd, 
                            float *output, 
                            float input){
    ltd->go.Input = input;
    LTD_Loop(ltd);
    *output = ltd->go.Output;
}

// Transfer运算_速度锁相环
static void Transfer_PLL_Loop(PLL_STRUCT *pll,
                            uint8_t mode,
                            uint8_t Poles,
                            float input_Rad){
    if (mode == MODE_PosVelCur_THREE){
        // 位置环模式：PLL连续积分（可以超过2π）
        // （Value_Correct可以解决位置环下...）
        // （input_Rad为0-2pi,OutRe*Poles取模后的越界行为）
        pll->go.Error = input_Rad - Value_normalize(pll->go.OutRe*Poles);
    }
    else{
        // 非位置环模式：PLL输出归一化到[0, 2π)
        pll->go.Error = input_Rad - pll->go.OutRe*Poles;
    }

    // 计算角度误差,始终归一化到[-π, π)范围
    // (此处为绝对编码器提供角度运算)
    Value_Correct(&pll->go.Error, pll->go.Error);

    PLL_Loop(pll);
    // 输出pll->go.OutWe;
    // 输出pll->go.OutRe;
}

// Transfer运算_霍尔传感器
static void Transfer_Hall_Loop(FOC_System_STRUCT *foc, 
                            PLL_STRUCT *pll){
    #if IS_HALL_MODE
    foc->transfer.Hall.go.Input_Ga = User_Encoder_ReadHall(0);
    foc->transfer.Hall.go.Input_Gb = User_Encoder_ReadHall(1);
    foc->transfer.Hall.go.Input_Gc = User_Encoder_ReadHall(2);
	Hall_Loop(&foc->transfer.Hall);
    float Hall_We = foc->transfer.Hall.go.Output_Rad;
    Transfer_PLL_Loop(pll, 
                    CONFIG_MODE, 
                    foc->motor.Poles, 
                    (Hall_We - Value_normalize(foc->encoder.Real_offset*foc->motor.Poles))*
                    foc->motor.Encoder_Dir);
    #endif // IS_HALL_MODE
}

// Transfer运算_高频注入
static void Transfer_HFI_Loop(FOC_System_STRUCT *foc, 
                            PLL_STRUCT *pll){
    #if IS_HFI_MODE
    static float HFI_Re = 0.0f;
    float Sine, Cosine;
    fast_sin_cos(HFI_Re, 
                &Sine, 
                &Cosine);
    foc->transfer.HFI.go.Input_Q = foc->current.Real_Ibeta*Cosine - 
                                    foc->current.Real_Ialpha*Sine;
    HFI_ReadRad_Loop(&foc->transfer.HFI);
    
    // 2.正交锁相环运算电子角度
    if ((foc->value.Sensorless_Flag == 0) || 
        (foc->value.Sensorless_Flag == 1)){
        pll->go.Error = foc->transfer.HFI.go.Output_Q;
    }
    else{
        #if IS_DEBUG_MODE
        pll->go.Error = foc->encoder.Sensorless_Re - 
                    HFI_Re + foc->encoder.Sensorless_offset;
        Value_Correct(&pll->go.Error, pll->go.Error);
        // （若高频注入锁到另一极，offset = -pi）
        // （这是因为输入锁相环是sin(2*error)的角度误差）
        #else // IS_Debug_MODE
        pll->go.Error = foc->encoder.Real_Re - 
                    HFI_Re + foc->encoder.Real_offset;
        #endif // IS_Debug_MODE
    }
    PLL_Loop(pll);
    HFI_Re = Value_normalize(pll->go.OutRe*
            foc->motor.Poles);
    #endif // IS_HFI_MODE
}

// Transfer运算_滑模观测器
static void Transfer_SMO_Loop(FOC_System_STRUCT *foc, 
                            PLL_STRUCT *pll){
    #if IS_SMO_MODE
    static float SMO_We = 0.0f;
    float Sine, Cosine;
    foc->transfer.SMO.data.Input_We = SMO_We;
    // foc->transfer.SMO.data.Input_We = foc->encoder.Real_We;
    foc->transfer.SMO.alpha.Input_Ix = foc->current.Real_Ialpha;
    foc->transfer.SMO.alpha.Input_Ux = foc->foc.Ualpha;
    foc->transfer.SMO.beta.Input_Ix = foc->current.Real_Ibeta;
    foc->transfer.SMO.beta.Input_Ux = foc->foc.Ubeta;
    SMO_Loop(&foc->transfer.SMO);
    fast_sin_cos(pll->go.OutRe*
                foc->motor.Poles,
                &Sine,
                &Cosine);
    pll->go.Error = - (foc->transfer.SMO.alpha.Output_Ex*Cosine + 
        foc->transfer.SMO.beta.Output_Ex*Sine);
    PLL_Loop(pll);
    SMO_We = pll->go.OutWe*
            foc->motor.Poles;
    #endif // IS_SMO_MODE
}

// Transfer运算_非线性磁链观测器器
static void Transfer_NLFO_Loop(FOC_System_STRUCT *foc, 
                            PLL_STRUCT *pll){
    #if IS_NLFO_MODE
    float Sine, Cosine;
    foc->transfer.NLFO.go.Input_Ialpha = foc->current.Real_Ialpha;
    foc->transfer.NLFO.go.Input_Ibeta = foc->current.Real_Ibeta;
    foc->transfer.NLFO.go.Input_Ualpha = foc->foc.Ualpha;
    foc->transfer.NLFO.go.Input_Ubeta = foc->foc.Ubeta;
    NLFO_Loop(&foc->transfer.NLFO);
    fast_sin_cos(pll->go.OutRe*
                foc->motor.Poles,
                &Sine,
                &Cosine);
    float error = foc->transfer.NLFO.go.Output_Sine*Cosine - 
        foc->transfer.NLFO.go.Output_Cosine*Sine;
    pll->go.Error = error;
    PLL_Loop(pll);
    #endif // IS_NLFO_MODE
}

// Transfer传递函数初始化
static void Transfer_Init(FOC_System_STRUCT *foc){
    // 0.用户自定义控制器参数
    User_StartMotor_Init();
    User_Motor_Init(foc);
    User_Parameter_Init(foc);
    foc->foc.sine = 0.0f;
    foc->foc.cosine = 1.0f;

    // 1.电流环参数
    foc->transfer.Current_D.T = PMSM_RUN_T;
    foc->transfer.Current_Q.T = PMSM_RUN_T;
    PID_Init(&foc->transfer.Current_D);
    PID_Init(&foc->transfer.Current_Q);
    
    // 2.转速环参数
    foc->transfer.Velocity.T = PMSM_RUN_T*foc->value.Response;
    #if CONFIG_CtrlVel==Control_LADRC
    Ladrc_Init(&foc->transfer.Velocity);
    #elif CONFIG_CtrlVel==Control_SMC
    foc->transfer.Velocity.Gain = foc->motor.identify.J/
        (1.5f*foc->motor.Poles*foc->motor.identify.Flux);
    SMC_Init(&foc->transfer.Velocity);
    #elif CONFIG_CtrlVel==Control_STA
    STA_Init(&foc->transfer.Velocity);
    #else // CONFIG_CtrlVel
    PID_Init(&foc->transfer.Velocity);
    #endif // CONFIG_Control

    // 3.位置环参数
    foc->transfer.Position.T = PMSM_RUN_T*foc->value.Response*
                                foc->value.Response;
    #if CONFIG_CtrlPos==Control_LADRC
    Ladrc_Init(&foc->transfer.Position);
    #elif CONFIG_CtrlPos==Control_SMC
    foc->transfer.Position.Gain = 1.0f;
    SMC_Init(&foc->transfer.Position);
    #elif CONFIG_CtrlPos==Control_STA
    STA_Init(&foc->transfer.Position);
    #else // CONFIG_CtrlPos
    PID_Init(&foc->transfer.Position);
    #endif // CONFIG_CtrlPos

    // 4.响应倍数参数
    // （此处暂无数据）

    // 5.滤波器参数
    foc->transfer.LPF_D.T = PMSM_RUN_T;
    foc->transfer.LPF_Q.T = PMSM_RUN_T;
    foc->transfer.LPF_encoder.T = PMSM_RUN_T;
    LPF_Init(&foc->transfer.LPF_D);
    LPF_Init(&foc->transfer.LPF_Q);
    LPF_Init(&foc->transfer.LPF_encoder);
    
    // 6.锁相环参数
    foc->transfer.PLL_encoder.T = PMSM_RUN_T;
    PLL_Init(&foc->transfer.PLL_encoder);
    if (CONFIG_MODE == MODE_PosVelCur_THREE){
        foc->transfer.PLL_encoder.is_position_mode = 1;
    }
    #if IS_COM_MODE
    foc->transfer.PLL_another.T = PMSM_RUN_T;
    PLL_Init(&foc->transfer.PLL_another);
    #endif // IS_COM_MODE

    // 7.Debug模式参数
    #if IS_DEBUG_MODE
    foc->transfer.PLL_Debug.T = PMSM_RUN_T;
    PLL_Init(&foc->transfer.PLL_Debug);
    foc->transfer.LPF_Debug.T = PMSM_RUN_T;
    LPF_Init(&foc->transfer.LPF_Debug);
    #endif // IS_DEBUG_MODE

    // 8.扰动观测器参数
    #if CONFIG_DOB
    foc->transfer.DOB.T = PMSM_RUN_T;
    foc->transfer.DOB.Pn = foc->motor.Poles;
    foc->transfer.DOB.Flux = foc->motor.identify.Flux;
    foc->transfer.DOB.B = foc->motor.identify.B;
    foc->transfer.DOB.J = foc->motor.identify.J;
    DOB_Init(&foc->transfer.DOB);
    #endif // CONFIG_DOB    

    // 9.谐波抑制
    #if CONFIG_Inhibit
    foc->transfer.TPNF_D.T = PMSM_RUN_T;
    foc->transfer.TPNF_Q.T = PMSM_RUN_T;
    TPNF_Init(&foc->transfer.TPNF_D);
    TPNF_Init(&foc->transfer.TPNF_Q);
    #endif // CONFIG_Inhibit

    // 10.弱磁控制参数
    #if CONFIG_FW
    foc->transfer.FW.T = PMSM_RUN_T;
    PID_Init(&foc->transfer.FW);
    #endif // CONFIG_FW

    // 11.速度前馈的参数
    // 12.死区补偿参数
    // 13.角度补偿参数
    // （此处暂无数据）

    // 14.抗齿槽离线标定
    #if CONFIG_Cogging
    foc->transfer.Cogging.T = PMSM_RUN_T;
    #endif // CONFIG_Cogging

    // 15.最速控制结构体
    #if IS_LTD_MODE
    foc->transfer.LTD.T = PMSM_RUN_T;
    LTD_Init(&foc->transfer.LTD);
    #endif // IS_LTD_MODE

    // 15.霍尔有感结构体
    #if IS_HALL_MODE
    foc->transfer.Hall.T = PMSM_RUN_T;
    Hall_Init(&foc->transfer.Hall);
    #endif // IS_HALL_MODE

    // 17.高频正弦波注入相关
    #if IS_HFI_MODE
    foc->transfer.HFI.T = PMSM_RUN_T;
    HFI_Init(&foc->transfer.HFI);

    foc->transfer.NSD.T = PMSM_RUN_T;
    foc->transfer.NSD.Wo = foc->transfer.HFI.Wo;
    NSD_Init(&foc->transfer.NSD);
    #endif // IS_HFI_MODE

    // 18.滑模观测器
    #if IS_SMO_MODE
    foc->transfer.SMO.T = PMSM_RUN_T;
    foc->transfer.SMO.Rs = foc->motor.identify.Rs;
    foc->transfer.SMO.Ld = foc->motor.identify.Ld;
    foc->transfer.SMO.Lq = foc->motor.identify.Lq;
    SMO_Init(&foc->transfer.SMO);
    #endif // IS_SMO_MODE

    // 19.非线性磁链观测器
    #if IS_NLFO_MODE
    foc->transfer.NLFO.T = PMSM_RUN_T;
    foc->transfer.NLFO.Flux = foc->motor.identify.Flux;
    foc->transfer.NLFO.Rs = foc->motor.identify.Rs;
    foc->transfer.NLFO.Ld = foc->motor.identify.Ld;
    foc->transfer.NLFO.Lq = foc->motor.identify.Lq;
    NLFO_Init(&foc->transfer.NLFO);
    #endif // IS_NLFO_MODE

    // 20.无感参数数据
    // （此处暂无数据）
    // （下面代码内容用于消除未使用函数的编译警告）
    (void)Transfer_TPNF_Loop;
    (void)Transfer_DOB_Loop;
    (void)Transfer_LTD_Loop;
    (void)Transfer_Hall_Loop;
    (void)Transfer_HFI_Loop;
    (void)Transfer_SMO_Loop;
    (void)Transfer_NLFO_Loop;

    (void)Printf_Normal_Loop;
    (void)Printf_Debug_Loop;
    (void)Printf_Cogging_Loop;
}

// Transfer空传递函数
static void Transfer_Null(FOC_System_STRUCT *foc){
    // 空函数，不执行具体数据
}

// Offset读取电流偏置
static void Offset_Current_Init(FOC_System_STRUCT *foc){
   // 读取电流偏置
   for (uint8_t i = 0; i < 24; i++){
       foc->current.Current_offset0 += User_ReadADC_Raw(0);
       foc->current.Current_offset1 += User_ReadADC_Raw(1);
       User_Delay(2);
   }
   foc->current.Current_offset0 = foc->current.Current_offset0/24;
   foc->current.Current_offset1 = foc->current.Current_offset1/24;

   // 计算电流最后增益
   foc->current.Final_Gain = foc->motor.MCU_Voltage/
       (foc->motor.ADC_Precision*foc->motor.Amplifier*foc->motor.Sampling_Rs);
}

// Offset读取编码器偏置
static void Offset_Rad_Encoder(FOC_System_STRUCT *foc){
    // 1.电机强拖到D轴零位
    foc->foc.Ud_in = 3.0f;
    User_Delay(1200);

    // 2.读取高精度编码器的偏置
    for (uint8_t i = 0; i < 10; i++){
        foc->encoder.Real_offset += User_Encoder_ReadRad();
        User_Delay(2);
    }
    foc->encoder.Real_offset = foc->encoder.Real_offset/10.0f;

    // 3.释放电机电压
    foc->foc.Ud_in = 0.0f;
    User_Delay(800);
}

// Offset读取霍尔传感器偏置
static void Offset_Rad_Hall(FOC_System_STRUCT *foc){
    #if IS_HALL_MODE
    // 1.电机强拖到D轴零位
    foc->foc.Ud_in = 3.0f;
    User_Delay(1200);

    // 2.读取霍尔传感器的偏置
    foc->transfer.Hall.go.Input_Ga = User_Encoder_ReadHall(0);
    foc->transfer.Hall.go.Input_Gb = User_Encoder_ReadHall(1);
    foc->transfer.Hall.go.Input_Gc = User_Encoder_ReadHall(2);
    Hall_Loop(&foc->transfer.Hall);

    // 3.偏置参数赋值
    #if IS_DEBUG_MODE
    foc->encoder.Sensorless_offset = foc->transfer.Hall.go.Output_Rad;
    #else // IS_DEBUG_MODE
    foc->encoder.Real_offset = foc->transfer.Hall.go.Output_Rad;
    #endif // IS_DEBUG_MODE

    // 4.释放电机电压
    foc->foc.Ud_in = 0.0f;
    User_Delay(800);   

    // 5.计算霍尔传感器最后增益
    foc->transfer.Hall.go.Gain = (float)(((double)foc->transfer.Hall.T)*
                                ((double)foc->transfer.Hall.Wc));
    foc->transfer.Hall.go.Normalized_Gain = (float)(1.0 - 
                                (((double)foc->transfer.Hall.T)*
                                ((double)foc->transfer.Hall.Wc)));
    #endif // IS_HALL_MODE
}

// Offset电机极性辨识
static void Offset_HFI_Init(FOC_System_STRUCT *foc){
    #if IS_HFI_MODE
    NSD_SetUd_Init(&foc->transfer.NSD);
    #endif // IS_HFI_MODE
}

// Tick高频注入和电感辨识的初始化，在High_Loop中执行
static void Init_Tick(FOC_System_STRUCT *foc){
    #if IS_HFI_MODE
    // 1.若NSD极性辨识成功，则赋值给offset
    if (foc->status >= MOTOR_STATUS_INITIALIZING){        
        foc->foc.Ud_in = foc->transfer.NSD.go.Output_Ud;
    }
    foc->transfer.NSD.go.Input_D = foc->current.Real_Id;
    if (NSD_ReadOffset_Loop(&foc->transfer.NSD)){
        #if IS_DEBUG_MODE
        foc->encoder.Sensorless_offset = -foc->transfer.NSD.go.Output_Flag*Value_PI;
        #else // IS_DEBUG_MODE
        foc->encoder.Real_offset = -foc->transfer.NSD.go.Output_Flag*Value_PI;
        #endif // IS_DEBUG_MODE
    }

    // // 2.若控制器在失能，高频注入权力交接在它手中
    foc->foc.Ud_in += foc->transfer.HFI.go.Output_Uin;
    #endif // IS_HFI_MODE

    // 3.电机DQ轴电感参数辨识
    foc->motor.identify.go.Input_Id = foc->current.Real_Id;
    foc->motor.identify.go.Input_Iq = foc->current.Real_Iq;
    foc->motor.identify.go.Input_We = foc->encoder.Real_We;
    Identify_ReadLs_Loop(&foc->motor.identify);
    if ((foc->motor.identify.go.Step == 1) || 
        (foc->motor.identify.go.Step == 2)){
        foc->foc.Ud_in = foc->motor.identify.go.Output_Ud;
        foc->foc.Uq_in = foc->motor.identify.go.Output_Uq;
    }
}

// Current读取当前的电流值并更新3相电流(未滤波)
static void Current_ReadIabc(FOC_System_STRUCT *foc){
    float I0 = (User_ReadADC_Raw(0) - foc->current.Current_offset0)*
                            foc->current.Final_Gain*foc->motor.Current_Dir0;
    float I1 = (User_ReadADC_Raw(1) - foc->current.Current_offset1)*
                            foc->current.Final_Gain*foc->motor.Current_Dir1;
    float I2 = -(I0 + I1);
    if (foc->motor.Current_Num == 0){  // AB采样(判断电流相序和电机方向)
        foc->current.Real_Ia = foc->motor.Motor_Dir == 1 ? I0 : I1;
        foc->current.Real_Ib = foc->motor.Motor_Dir == 1 ? I1 : I0;
        foc->current.Real_Ic = I2;
    } else if (foc->motor.Current_Num == 1){  // AC采样(判断电流相序和电机方向)
        foc->current.Real_Ia = foc->motor.Motor_Dir == 1 ? I0 : I2;
        foc->current.Real_Ib = foc->motor.Motor_Dir == 1 ? I2 : I0;
        foc->current.Real_Ic = I1;
    } else {  // BC采样(判断电流相序和电机方向)
        foc->current.Real_Ia = foc->motor.Motor_Dir == 1 ? I2 : I0;
        foc->current.Real_Ib = foc->motor.Motor_Dir == 1 ? I0 : I2;
        foc->current.Real_Ic = I1;
    }
}

// Current处理电流的坐标变换、滤波等，更新最终的DQ轴电流值
static void Current_Tick(FOC_System_STRUCT *foc){
    // 1.读取三相原始电流量
    Current_ReadIabc(foc);
    
    // 2.坐标变换到DQ轴电流
    float Id_Raw,Iq_Raw;
    clarke(&foc->current.Real_Ialpha,
        &foc->current.Real_Ibeta,
        foc->current.Real_Ia,
        foc->current.Real_Ib);
    park(&Id_Raw,
        &Iq_Raw,
        foc->current.Real_Ialpha,
        foc->current.Real_Ibeta,
        foc->foc.sine,
        foc->foc.cosine);

    // 3.DQ轴高频信号陷波过滤
    #if IS_HFI_MODE
    foc->transfer.HFI.go.Input_Id = Id_Raw;
    foc->transfer.HFI.go.Input_Iq = Iq_Raw;
    HFI_Current_Loop(&foc->transfer.HFI);
    Id_Raw = foc->transfer.HFI.go.Output_Id;
    Iq_Raw = foc->transfer.HFI.go.Output_Iq;
    #endif // IS_HFI_MODE

    // 4.DQ轴电流谐波抑制(抑制电机5、7次主要谐波)
    #if CONFIG_Inhibit
    float We_6th = Value_fabsf(6.0f*foc->encoder.Real_We);
    foc->transfer.TPNF_D.Wo = We_6th;
    foc->transfer.TPNF_Q.Wo = We_6th;
    Transfer_TPNF_Loop(&foc->transfer.TPNF_D, 
                    Id_Raw);
    Transfer_TPNF_Loop(&foc->transfer.TPNF_Q, 
                    Iq_Raw);
    Id_Raw = foc->transfer.TPNF_D.filter.Output;
    Iq_Raw = foc->transfer.TPNF_Q.filter.Output;
    #endif // CONFIG_Inhibit

    // 5.DQ轴电流低通滤波
    Transfer_LPF_Loop(&foc->transfer.LPF_D,
                    Id_Raw);
    Transfer_LPF_Loop(&foc->transfer.LPF_Q,
                    Iq_Raw);
    foc->current.Real_Id = foc->transfer.LPF_D.filter.Output;
    foc->current.Real_Iq = foc->transfer.LPF_Q.filter.Output;
}

// Encoder处理虚拟角度，计算虚拟生成数值
static void Encoder_Virtual_Angle(FOC_System_STRUCT *foc){
    // 1.计算虚拟角度并设置VF/IF的参数
    #if CONFIG_MODE==MODE_VF_OPENLOOP
    foc->foc.Uq_in = foc->foc.Target_VF_Uq;

    // （虽然不是“无感模式”，但VF开环强拖不能阶跃，所以有LTD）
    // （速度环输入值会被LTD约束到渐进至Target_Speed）
    Transfer_LTD_Loop(&foc->transfer.LTD, 
                    &foc->foc.Speed_in, 
                    foc->foc.Target_Speed);
    foc->encoder.Real_Speed = foc->foc.Speed_in;
    #elif CONFIG_MODE==MODE_IF_OPENLOOP
    foc->foc.Target_Iq = foc->foc.Target_IF_Iq;

    // （虽然不是“无感模式”，但IF开环强拖不能阶跃，所以有LTD）
    // （速度环输入值会被LTD约束到渐进至Target_Speed）
    Transfer_LTD_Loop(&foc->transfer.LTD, 
                    &foc->foc.Speed_in, 
                    foc->foc.Target_Speed);
    foc->encoder.Real_Speed = foc->foc.Speed_in;
    #endif // CONFIG_MODE

    // 2.电机角度相关数值更新
    static float Last_Real_Speed = 0.0f;
    Value_Rad_Loop(&foc->encoder.Real_Pos, 
                foc->encoder.Real_Speed, 
                Last_Real_Speed, 
                PMSM_RUN_T);
    foc->encoder.Real_We = foc->encoder.Real_Speed*foc->motor.Poles;
    foc->encoder.Real_Re = Value_normalize(
                foc->encoder.Real_Pos*foc->motor.Poles);
    Last_Real_Speed = foc->encoder.Real_Speed;
}

// Encoder处理机械角度，计算编码数值Encoder
static void Encoder_Sensor_Encoder(FOC_System_STRUCT *foc){
    // 1.锁相环运算机械角度
    float Rad = User_Encoder_ReadRad();
    Transfer_PLL_Loop(&foc->transfer.PLL_encoder,
                    CONFIG_MODE, 1,
                    (Rad - foc->encoder.Real_offset)*
                    foc->motor.Encoder_Dir);

    // 2.低通滤波
    Transfer_LPF_Loop(&foc->transfer.LPF_encoder,
                    foc->transfer.PLL_encoder.go.OutWe);
    
    // 3.电机角度相关数值更新
    foc->encoder.Real_Speed = foc->transfer.LPF_encoder.filter.Output;
    foc->encoder.Real_Pos = foc->transfer.PLL_encoder.go.OutRe;
    foc->encoder.Real_We = foc->encoder.Real_Speed*foc->motor.Poles;
    foc->encoder.Real_Re = Value_normalize(
                            foc->encoder.Real_Pos*
                            foc->motor.Poles);

    // 4.速度输入值更新
    // （若为普通速度控制不是“无感模式”，所以没有LTD）
    // （若不是，速度环输入值等于由LTD渐进，不可阶跃）
    #if IS_DEBUG_MODE
    Transfer_LTD_Loop(&foc->transfer.LTD, 
                    &foc->foc.Speed_in, 
                    foc->foc.Target_Speed);
    #else // IS_Debug_MODE
    foc->foc.Speed_in = foc->foc.Target_Speed;
    #endif // IS_Debug_MODE
}

// Encoder处理电角度，计算编码器数值HALL
static void Encoder_Sensor_Hall(FOC_System_STRUCT *foc){
    #if IS_HALL_MODE
    // 1.霍尔锁相环运算电子角度
    Transfer_Hall_Loop(foc, &foc->transfer.PLL_encoder);

    // 2.低通滤波
    Transfer_LPF_Loop(&foc->transfer.LPF_encoder, 
                    foc->transfer.PLL_encoder.go.OutWe);

    // 3.电机角度相关数值更新
    foc->encoder.Real_Speed = foc->transfer.LPF_encoder.filter.Output;
    foc->encoder.Real_Pos = foc->transfer.PLL_encoder.go.OutRe;
    foc->encoder.Real_We = foc->encoder.Real_Speed*foc->motor.Poles;
    foc->encoder.Real_Re = Value_normalize(
                            foc->encoder.Real_Pos*
                            foc->motor.Poles);

    // 4.速度输入值更新
    // （虽然不是“无感模式”，但霍尔电机速度还是不要阶跃的好，所以有LTD）
    // （速度环输入值会被LTD约束到渐进至Target_Speed）
    Transfer_LTD_Loop(&foc->transfer.LTD, 
                    &foc->foc.Speed_in, 
                    foc->foc.Target_Speed);
    #endif // IS_HALL_MODE
}

// Encoder处理电角度，计算编码器数值HFI
static void Encoder_Sensorless_HFI(FOC_System_STRUCT *foc){
    #if IS_HFI_MODE
    // 1.高频注入信号解调及正交锁相环
    Transfer_HFI_Loop(foc, &foc->transfer.PLL_encoder);

    float Pos = Value_normalize(
        foc->transfer.PLL_encoder.go.OutRe - 
        (foc->encoder.Real_offset/((float)foc->motor.Poles)));

    // 2.低通滤波
    Transfer_LPF_Loop(&foc->transfer.LPF_encoder, 
                    foc->transfer.PLL_encoder.go.OutWe);

    // 3.电机角度相关数值更新
    foc->encoder.Real_Speed = foc->transfer.LPF_encoder.filter.Output;
    foc->encoder.Real_Pos = Pos;
    foc->encoder.Real_We = foc->encoder.Real_Speed*foc->motor.Poles;
    foc->encoder.Real_Re = Value_normalize(
                            foc->encoder.Real_Pos*
                            foc->motor.Poles);

    // 4.速度输入值更新
    // （“HFI的无感模式”，速度可阶跃，不过无感模式要稳定，还是得有LTD）
    // （速度环输入值会被LTD约束到渐进至Target_Speed）
    // （希望无感高频注入的Speed_in快速变化，可调大LTD.r参数）
    Transfer_LTD_Loop(&foc->transfer.LTD, 
                    &foc->foc.Speed_in, 
                    foc->foc.Target_Speed);

    // 6.特殊处理(防止跨越低速域)
    Value_Limit(&foc->foc.Target_Speed, 
            foc->value.Sensorless_AbsMax, 
            -foc->value.Sensorless_AbsMax);
    #endif // IS_HFI_MODE
}

// Encoder处理电角度，计算编码器数值SMO
static void Encoder_Sensorless_Tall(FOC_System_STRUCT *foc){
    #if IS_IF_MODE
    // 1.速度输入值更新
    // （IF开环强拖不能阶跃，所以有LTD）
    // （速度环输入值会被LTD约束到渐进至Target_Speed）
    static float Last_Speed_in = 0.0f;
    static float Last_High_Wm = 0.0f;
    Transfer_LTD_Loop(&foc->transfer.LTD, 
                    &foc->foc.Speed_in, 
                    foc->foc.Target_Speed);

    // 2.特殊处理(防止跨越低速域)
    float Speed_Abs = Value_Gain_Get(&foc->value.Sensorless_Gain, 
                                    foc->encoder.Real_Speed, 
                                    foc->value.Sensorless_AbsMax, 
                                    foc->value.Sensorless_AbsMin);
    float Normalized_Gain = 1.0f - foc->value.Sensorless_Gain;

    // 3.高速域观测器运算电机速度和角度
    #if IS_SMO_MODE
    if (Speed_Abs >= foc->value.Sensorless_Start){
        Transfer_SMO_Loop(foc, &foc->transfer.PLL_encoder);
    }
    #else // IS_SMO_MODE
    if (Speed_Abs >= foc->value.Sensorless_Start){
        Transfer_NLFO_Loop(foc, &foc->transfer.PLL_encoder);
    }
    #endif // IS_SMO_MODE

    // 4.高速域数值更新
    foc->value.High_Wm = foc->transfer.PLL_encoder.go.OutWe;
    foc->value.High_angle = foc->transfer.PLL_encoder.go.OutRe;

    // 5.低速域数值更新
    if (Speed_Abs <= foc->value.Sensorless_AbsMin){
        Value_Rad_Loop(&foc->value.Low_angle, 
                    foc->foc.Speed_in, 
                    Last_Speed_in, 
                    PMSM_RUN_T);
    }
    else{
        Value_Rad_Loop(&foc->value.Low_angle, 
                    foc->value.High_Wm, 
                    Last_High_Wm, 
                    PMSM_RUN_T);
    }
    foc->value.Low_Wm = foc->foc.Speed_in;
    // foc->value.Low_angle = Value_normalize(
    //                 foc->value.Low_angle*foc->motor.Poles);

    // 6.数值融合
    float Speed = foc->value.Low_Wm*Normalized_Gain + 
                            foc->value.High_Wm*foc->value.Sensorless_Gain;
    if (Speed_Abs >= foc->value.Sensorless_AbsMin){
        Speed = foc->value.High_Wm;
    }

    // 7.低通滤波
    Transfer_LPF_Loop(&foc->transfer.LPF_encoder, 
                    Speed);
    foc->encoder.Real_Speed = foc->transfer.LPF_encoder.filter.Output;

    // 8.电机角度相关数值更新
    Value_Correct(&foc->value.Low_angle, (foc->value.Low_angle - foc->value.High_angle));
    foc->encoder.Real_Pos = Value_normalize(foc->value.Low_angle*Normalized_Gain + 
                            foc->value.High_angle*foc->value.Sensorless_Gain);
    if (Speed_Abs >= foc->value.Sensorless_AbsMin){
        foc->encoder.Real_Pos = foc->value.High_angle;
    }
    foc->encoder.Real_We = foc->encoder.Real_Speed*foc->motor.Poles;
    foc->encoder.Real_Re = Value_normalize(
                            foc->encoder.Real_Pos*
                            foc->motor.Poles);
    Last_Speed_in = foc->foc.Speed_in;
    Last_High_Wm = foc->value.High_Wm;
    #endif // IS_IF_MODE
}

// Encoder处理电角度，计算编码器数值SMO/NLFO+HFI
static void Encoder_Sensorless_Normal(FOC_System_STRUCT *foc){
    #if IS_COM_MODE
    // 1.速度输入值更新
    // （IF开环强拖不能阶跃，所以有LTD）
    // （速度环输入值会被LTD约束到渐进至Target_Speed）
    Transfer_LTD_Loop(&foc->transfer.LTD, 
                    &foc->foc.Speed_in, 
                    foc->foc.Target_Speed);

    // 2.特殊处理(防止跨越低速域)
    float Speed_Abs = Value_Gain_Get(&foc->value.Sensorless_Gain, 
                                    foc->encoder.Real_Speed, 
                                    foc->value.Sensorless_AbsMax, 
                                    foc->value.Sensorless_AbsMin);
    float Normalized_Gain = 1.0f - foc->value.Sensorless_Gain;

    // 3.高速域观测器运算电机速度和角度
    #if IS_SMO_MODE
    if (Speed_Abs >= foc->value.Sensorless_Start){
        Transfer_SMO_Loop(foc, &foc->transfer.PLL_encoder);
    }
    #else // IS_SMO_MODE
    if (Speed_Abs >= foc->value.Sensorless_Start){
        Transfer_NLFO_Loop(foc, &foc->transfer.PLL_encoder);
    }
    #endif // IS_SMO_MODE

    // 4.高速域数值更新
    foc->value.High_Wm = foc->transfer.PLL_encoder.go.OutWe;
    foc->value.High_angle = foc->transfer.PLL_encoder.go.OutRe;
    
    // 5.低速域运算电机速度和角度
    // #if IS_HFI_MODE
    // static float Last_Speed_Abs = 0.0f;
    // float Delta = Speed_Abs - Last_Speed_Abs;
    // Last_Speed_Abs = Speed_Abs;

    // Transfer_HFI_Loop(foc, &foc->transfer.PLL_another);
    // foc->value.Low_Wm = foc->transfer.PLL_another.go.OutWe;
    // foc->value.Low_angle = Value_normalize(
    //     foc->transfer.PLL_another.go.OutRe - 
    //     (foc->encoder.Real_offset/((float)foc->motor.Poles)));

    // if ((Speed_Abs >= foc->value.Sensorless_Stop) && 
    //     (Delta >= 0)){
    //     foc->value.Sensorless_Flag = 1;
    // }
    // else if ((Speed_Abs <= foc->value.Sensorless_Open) && 
    //     (Delta < 0)){
    //     foc->value.Sensorless_Flag = 0;
    // }
    // #else // IS_HFI_MODE
    // Transfer_Hall_Loop(foc, &foc->transfer.PLL_another);
    // foc->value.Low_Wm = foc->transfer.PLL_another.go.OutWe;
    // foc->value.Low_angle = foc->transfer.PLL_another.go.OutRe;
    // #endif // IS_HFI_MODE

    float Rad = User_Encoder_ReadRad();
    Transfer_PLL_Loop(&foc->transfer.PLL_another,
                    CONFIG_MODE, 1,
                    (Rad - foc->encoder.Real_offset)*
                    foc->motor.Encoder_Dir);

    foc->value.Low_Wm = foc->transfer.PLL_another.go.OutWe;
    foc->value.Low_angle = foc->transfer.PLL_another.go.OutRe;

    // 6.数值融合
    float Speed = foc->value.Low_Wm*Normalized_Gain + 
                            foc->value.High_Wm*foc->value.Sensorless_Gain;

    // 7.低通滤波
    Transfer_LPF_Loop(&foc->transfer.LPF_encoder, 
                    Speed);

    // 8.电机角度相关数值更新
    foc->encoder.Real_Speed = foc->transfer.LPF_encoder.filter.Output;
    Value_Correct(&foc->value.Low_angle, (foc->value.Low_angle - foc->value.High_angle));
    foc->encoder.Real_Pos = Value_normalize(foc->value.Low_angle*Normalized_Gain + 
                            foc->value.High_angle*foc->value.Sensorless_Gain);
    foc->encoder.Real_We = foc->encoder.Real_Speed*foc->motor.Poles;
    foc->encoder.Real_Re = Value_normalize(
                            foc->encoder.Real_Pos*
                            foc->motor.Poles);
    #endif // IS_COM_MODE
}

// Debug无感调试，计算编码器数值HFI
static void Debug_Sensorless_HFI(FOC_System_STRUCT *foc){
    #if CONFIG_MODE==MODE_Debug_HFI
    // 1.高频注入信号解调及正交锁相环
    Transfer_HFI_Loop(foc, &foc->transfer.PLL_Debug);

    float Pos = Value_normalize(
        foc->transfer.PLL_Debug.go.OutRe - 
        (foc->encoder.Sensorless_offset/((float)foc->motor.Poles)));

    // 2.低通滤波
    Transfer_LPF_Loop(&foc->transfer.LPF_Debug, 
                    foc->transfer.PLL_Debug.go.OutWe);

    // 3.电机角度相关数值更新
    foc->encoder.Sensorless_Speed = foc->transfer.LPF_Debug.filter.Output;
    foc->encoder.Sensorless_Pos = Pos;
    foc->encoder.Sensorless_We = foc->encoder.Sensorless_Speed*foc->motor.Poles;
    foc->encoder.Sensorless_Re = Value_normalize(
                            foc->encoder.Sensorless_Pos*
                            foc->motor.Poles);
    #endif // CONFIG_MODE
}

// Debug无感调试，计算编码器数值SMO/NLFO
static void Debug_Sensorless_Tall(FOC_System_STRUCT *foc){
    #if (CONFIG_MODE==MODE_Debug_SMO) || (CONFIG_MODE==MODE_Debug_NLFO)
    // 1.取绝对值判断何时开启高速域观测器
    float Speed_Abs = Value_fabsf(foc->encoder.Real_Speed);

    // 2.高速域观测器运算电机速度和角度
    #if IS_SMO_MODE
    if (Speed_Abs >= foc->value.Sensorless_Start){
    Transfer_SMO_Loop(foc, &foc->transfer.PLL_Debug);
    }
    #else // IS_SMO_MODE
    if (Speed_Abs >= foc->value.Sensorless_Start){
        Transfer_NLFO_Loop(foc, &foc->transfer.PLL_Debug);
    }
    #endif // IS_SMO_MODE

    // 3.低通滤波
    Transfer_LPF_Loop(&foc->transfer.LPF_Debug, 
                    foc->transfer.PLL_Debug.go.OutWe);
    
    // 4.电机无感角度相关数值更新
    foc->encoder.Sensorless_Speed = foc->transfer.LPF_Debug.filter.Output;
    foc->encoder.Sensorless_Pos = foc->transfer.PLL_Debug.go.OutRe;
    foc->encoder.Sensorless_We = foc->encoder.Sensorless_Speed*foc->motor.Poles;
    foc->encoder.Sensorless_Re = Value_normalize(
                            foc->encoder.Sensorless_Pos*
                            foc->motor.Poles);
    #endif // CONFIG_MODE
}

// Debug无感调试，计算编码器数值SMO/NLFO+HFI
static void Debug_Sensorless_Normal(FOC_System_STRUCT *foc){
    #if (CONFIG_MODE==MODE_Debug_HS) || (CONFIG_MODE==MODE_Debug_HN)
    // 1.取绝对值判断何时开启高速域观测器
    float Speed_Abs = Value_Gain_Get(&foc->value.Sensorless_Gain, 
                                    foc->encoder.Real_Speed, 
                                    foc->value.Sensorless_AbsMax, 
                                    foc->value.Sensorless_AbsMin);
    float Normalized_Gain = 1.0f - foc->value.Sensorless_Gain;

    // 2.高速域观测器运算电机速度和角度
    #if IS_SMO_MODE
    if (Speed_Abs >= foc->value.Sensorless_Start){
        Transfer_SMO_Loop(foc, &foc->transfer.PLL_Debug);
    }
    #else // IS_SMO_MODE
    if (Speed_Abs >= foc->value.Sensorless_Start){
        Transfer_NLFO_Loop(foc, &foc->transfer.PLL_Debug);
    }
    #endif // IS_SMO_MODE

    // 3.高速域数值更新
    foc->value.High_Wm = foc->transfer.PLL_Debug.go.OutWe;
    foc->value.High_angle = foc->transfer.PLL_Debug.go.OutRe;

    // 4.特殊处理(防止跨越低速域)
    static float Last_Speed_Abs = 0.0f;
    float Delta = Speed_Abs - Last_Speed_Abs;
    Last_Speed_Abs = Speed_Abs;

    // 5.低速域运算电机速度和角度
    if ((Speed_Abs >= foc->value.Sensorless_Stop) && 
        (Delta >= 0)){
        foc->value.Sensorless_Flag = 1;
    }
    else if ((Speed_Abs <= foc->value.Sensorless_Open) && 
        (Delta < 0)){
        foc->value.Sensorless_Flag = 0;
    }
    Transfer_HFI_Loop(foc, &foc->transfer.PLL_another);
    foc->value.Low_Wm = foc->transfer.PLL_another.go.OutWe;
    foc->value.Low_angle = Value_normalize(
        foc->transfer.PLL_another.go.OutRe - 
        (foc->encoder.Sensorless_offset/((float)foc->motor.Poles)));

    // 6.数值融合
    float Speed = foc->value.Low_Wm*Normalized_Gain + 
                            foc->value.High_Wm*foc->value.Sensorless_Gain;

    // 7.低通滤波
    Transfer_LPF_Loop(&foc->transfer.LPF_Debug, 
                    Speed);

    // 8.电机无感角度相关数值更新
    foc->encoder.Sensorless_Speed = foc->transfer.LPF_Debug.filter.Output;
    Value_Correct(&foc->value.Low_angle, (foc->value.Low_angle - foc->value.High_angle));
    foc->encoder.Sensorless_Pos = Value_normalize(foc->value.Low_angle*Normalized_Gain + 
                            foc->value.High_angle*foc->value.Sensorless_Gain);
    foc->encoder.Sensorless_We = foc->encoder.Sensorless_Speed*foc->motor.Poles;
    foc->encoder.Sensorless_Re = Value_normalize(
                            foc->encoder.Sensorless_Pos*
                            foc->motor.Poles);
    #endif // CONFIG_MODE
}

// Control电流单环(单闭环)
static void Control_Current_SINGLE(FOC_System_STRUCT *foc){
    // 1.电流环PI控制器计算
    Transfer_PID_Loop(&foc->transfer.Current_D,
        foc->foc.Target_Id,
        foc->current.Real_Id);
    Transfer_PID_Loop(&foc->transfer.Current_Q,
        foc->foc.Target_Iq,
        foc->current.Real_Iq);
    foc->foc.Ud_in = foc->transfer.Current_D.run.Output;
    foc->foc.Uq_in = foc->transfer.Current_Q.run.Output;

    // 2.结果输出到Ud和Uq给定(带前馈计算)
    #if CONFIG_CurFF
    foc->foc.Ud_in += Feedforward_CurrentD(foc->encoder.Real_We,
                                    foc->motor.identify.Lq,
                                    foc->current.Real_Iq);
    foc->foc.Uq_in += Feedforward_CurrentQ(foc->encoder.Real_We,
                                    foc->motor.identify.Ld,
                                    foc->current.Real_Id,
                                    foc->motor.identify.Flux);
    #endif // CONFIG_CurFF
}

// Control速度-电流双环(双闭环)
static void Control_VelCur_DOUBLE(FOC_System_STRUCT *foc){
    static uint8_t Control_Count = 0;  
    Control_Count++;
        
    // 1.转速环控制器计算
    if (Control_Count >= foc->value.Response){
        CtrlFunc_Tick[Value_set(CONFIG_CtrlVel,PMSM_MAX_Ctrl,0)](
            &foc->transfer.Velocity,
            foc->foc.Speed_in,
            foc->encoder.Real_Speed);
        foc->foc.Target_Iq = foc->transfer.Velocity.run.Output;
        Control_Count = 0;
    }

    // 2.MTPA控制
    #if CONFIG_MTPA
    MTPA_Loop(&foc->foc.Target_Id,
            foc->motor.identify.Flux,
            foc->motor.identify.Ld,
            foc->motor.identify.Lq,
            foc->current.Real_Iq);
    #endif // CONFIG_MTPA

    // 3.FW弱磁控制
    #if CONFIG_FW
    foc->foc.Target_Id += FW_Loop(
            &foc->transfer.FW, 
            foc->foc.Ud_in, 
            foc->foc.Uq_in, 
            foc->encoder.Real_Speed, 
            foc->value.FW_BaseSpeed, 
            foc->value.FW_Percentage, 
            foc->foc.Real_VBUS);
    #endif // CONFIG_FW

    // 4.计算速度环输出补偿(速度前馈量叠加)
    #if CONFIG_VelFF
    foc->foc.Target_Iq += Feedforward_Velocity(foc->encoder.Real_Speed,
                                                foc->value.VelFF_Beta);
    #endif // CONFIG_VelFF

    // 5.补偿Q轴电流(扰动前馈量叠加)
    #if CONFIG_DOB
    Transfer_DOB_Loop(&foc->transfer.DOB,
        foc->current.Real_Iq,
        foc->encoder.Real_Speed);
    foc->foc.Target_Iq += Feedforward_DOB(foc->transfer.DOB.smdo.Output_Fd,
                                                foc->motor.Poles,
                                                foc->motor.identify.Flux);
    #endif // CONFIG_DOB

    // 6.纯SMO和NLFO的IF开环强拖启动的电流输入值权重区
    #if IS_IF_MODE
    if (foc->value.Sensorless_Gain == 0.0f){
        foc->foc.Target_Iq = foc->foc.Target_IF_Iq*
            (1.0f - foc->value.Sensorless_Gain) + foc->foc.Target_Iq;
        /* code */
    }
    
    #endif // IS_IF_MODE

    // 7.电流环PI控制器计算
    Transfer_PID_Loop(&foc->transfer.Current_D,
        foc->foc.Target_Id,       // 默认D轴励磁Id为0
        foc->current.Real_Id);
    Transfer_PID_Loop(&foc->transfer.Current_Q,
        foc->foc.Target_Iq,
        foc->current.Real_Iq);
    foc->foc.Ud_in = foc->transfer.Current_D.run.Output;
    foc->foc.Uq_in = foc->transfer.Current_Q.run.Output;
    
    // 8.结果输出到Ud和Uq给定(带前馈计算)
    #if CONFIG_CurFF
    foc->foc.Ud_in += Feedforward_CurrentD(foc->encoder.Real_We,
                                    foc->motor.identify.Lq,
                                    foc->current.Real_Iq);
    foc->foc.Uq_in += Feedforward_CurrentQ(foc->encoder.Real_We,
                                    foc->motor.identify.Ld,
                                    foc->current.Real_Id,
                                    foc->motor.identify.Flux);
    #endif // CONFIG_CurFF

    // 9.高频注入信号叠加处理
    #if IS_HFI_MODE
    if (!foc->value.Sensorless_Flag){
        foc->foc.Ud_in += foc->transfer.HFI.go.Output_Uin;
    }
    #endif // IS_HFI_MODE
}

// Control高性能伺服三环(三闭环)
static void Control_PosVelCur_THREE(FOC_System_STRUCT *foc){
    static uint8_t Control_Count = 0;
    Control_Count++;

    // 1.位置环PD控制器计算
    if (Control_Count >= 
        (foc->value.Response*foc->value.Response)){
        CtrlFunc_Tick[Value_set(CONFIG_CtrlPos,PMSM_MAX_Ctrl,0)](
            &foc->transfer.Position,
            foc->foc.Target_Pos,
            foc->encoder.Real_Pos);
        Control_Count = 0;
        foc->foc.Target_Speed = foc->transfer.Position.run.Output;
        foc->foc.Speed_in = foc->foc.Target_Speed;
        // 位置环不是“无感模式”，所以没有LTD
        // （也就是Speed_in可以直接等于Target_Speed）
        // （无需进行LTD最速控制...）
    }

    // 2.转速环PI控制器计算
    if (Control_Count % foc->value.Response == 0){
        CtrlFunc_Tick[Value_set(CONFIG_CtrlVel,PMSM_MAX_Ctrl,0)](
            &foc->transfer.Velocity,
            foc->foc.Speed_in,
            foc->encoder.Real_Speed);
        foc->foc.Target_Iq = foc->transfer.Velocity.run.Output;
    }

    // 3.MTPA控制
    #if CONFIG_MTPA
    MTPA_Loop(&foc->foc.Target_Id,
            foc->motor.identify.Flux,
            foc->motor.identify.Ld,
            foc->motor.identify.Lq,
            foc->current.Real_Iq);
    #endif // CONFIG_MTPA

    // 4.FW弱磁控制
    #if CONFIG_FW
    foc->foc.Target_Id += FW_Loop(
            &foc->transfer.FW, 
            foc->foc.Ud_in, 
            foc->foc.Uq_in, 
            foc->encoder.Real_Speed, 
            foc->value.FW_BaseSpeed, 
            foc->value.FW_Percentage, 
            foc->foc.Real_VBUS);
    #endif // CONFIG_FW

    // 5.计算速度环输出补偿(速度前馈量叠加)
    #if CONFIG_VelFF
    foc->foc.Target_Iq += Feedforward_Velocity(foc->encoder.Real_Speed,
                                                foc->value.VelFF_Beta);
    #endif // CONFIG_VelFF

    // 6.补偿Q轴电流(扰动前馈量叠加)
    #if CONFIG_DOB
    Transfer_DOB_Loop(&foc->transfer.DOB,
        foc->current.Real_Iq,
        foc->encoder.Real_Speed);
    foc->foc.Target_Iq += Feedforward_DOB(foc->transfer.DOB.smdo.Output_Fd,
                                                foc->motor.Poles,
                                                foc->motor.identify.Flux);
    #endif // CONFIG_DOB

    // 7.电流环PI控制器计算
    Transfer_PID_Loop(&foc->transfer.Current_D,
        foc->foc.Target_Id,       // 默认D轴励磁为0
        foc->current.Real_Id);
    Transfer_PID_Loop(&foc->transfer.Current_Q,
        foc->foc.Target_Iq,
        foc->current.Real_Iq);
    foc->foc.Ud_in = foc->transfer.Current_D.run.Output;
    foc->foc.Uq_in = foc->transfer.Current_Q.run.Output;

    // 8.结果输出到Ud和Uq给定(电流前馈量叠加)
    #if CONFIG_CurFF
    foc->foc.Ud_in += Feedforward_CurrentD(foc->encoder.Real_We,
                                    foc->motor.identify.Lq,
                                    foc->current.Real_Iq);
    foc->foc.Uq_in += Feedforward_CurrentQ(foc->encoder.Real_We,
                                    foc->motor.identify.Ld,
                                    foc->current.Real_Id,
                                    foc->motor.identify.Flux);
    #endif // CONFIG_CurFF
}

// SVPWM电机驱动的马鞍波生成
static void PWM_Tick(FOC_System_STRUCT *foc,
                    float U_alpha,
                    float U_beta){
    // 2.PWM调制量运算
    #if CONFIG_PWM
    SPWM(U_alpha, 
        U_beta,
        &foc->foc.Du,
        &foc->foc.Dv,
        &foc->foc.Dw);          
    #else // CONFIG_PWM
    SVPWM(U_alpha, 
        U_beta,
        &foc->foc.Du,
        &foc->foc.Dv,
        &foc->foc.Dw);
    #endif // CONFIG_PWM

    // 3.提高低速性能(死区补偿叠加量)
    #if CONFIG_DeadZone
    DeadZone_Loop(&foc->foc.Du,
        &foc->foc.Dv,
        &foc->foc.Dw,
        foc->current.Real_Ia,
        foc->current.Real_Ib,
        foc->current.Real_Ic,
        foc->value.DeadZone_CurMin,
        foc->value.DeadZone_Time);
    #endif // CONFIG_DeadZone

    // 4.三相归一化电压值限幅
    Value_Limit(&foc->foc.Du,1.0f,0.0f);
    Value_Limit(&foc->foc.Dv,1.0f,0.0f);
    Value_Limit(&foc->foc.Dw,1.0f,0.0f);

    // 5.电机占空比输出
    uint32_t Duty_u,Duty_v,Duty_w;
    if (foc->motor.PWM_Dir == 1){
        Duty_u = (uint32_t)(foc->foc.Du*foc->motor.Duty);
        Duty_v = (uint32_t)(foc->foc.Dv*foc->motor.Duty);
        Duty_w = (uint32_t)(foc->foc.Dw*foc->motor.Duty);
    }
    else if (foc->motor.PWM_Dir == -1){
        Duty_u = (uint32_t)((1.0f - foc->foc.Du)*foc->motor.Duty);
        Duty_v = (uint32_t)((1.0f - foc->foc.Dv)*foc->motor.Duty);
        Duty_w = (uint32_t)((1.0f - foc->foc.Dw)*foc->motor.Duty);
    }
    if (foc->motor.Motor_Dir == -1){ // 判断电机方向并修改(原理是AB相序交换)
        uint32_t duty_temp = Duty_u;
        Duty_u = Duty_v;
        Duty_v = duty_temp;
    }
    User_PwmDuty_Set(Duty_u, Duty_v, Duty_w);
}


// =============================== 通用数据层(代码实现) =============================
// Status母线电压和驱动器物理温度数据更新
static void Status_Protection_Loop(FOC_System_STRUCT *foc){
    // 1.更新母线电压数值
    if (User_VBUS_DataGet() != Value_N_INF){        
        foc->foc.Real_VBUS = User_VBUS_DataGet();
    }
    else{
        foc->foc.Real_VBUS = foc->motor.VBUS;
    }

    // 2.更新驱动器温度数值
    if (User_Temperature_DataGet() != Value_N_INF){        
        foc->foc.Real_Temp = User_Temperature_DataGet();
    }
}

// Status电机出现错误的时候，参数设置为零
static void Status_Zero(FOC_System_STRUCT *foc){
    static uint8_t status = 0;
    if (status != foc->status){
        if ((foc->status >= MOTOR_STATUS_ENCODER_ERROR) || 
            (foc->status == MOTOR_STATUS_STANDBY)){
            // 1.电压给定归零
            foc->foc.Ud_in = 0.0f;
            foc->foc.Uq_in = 0.0f;

            // 2.偏置数值归零
            foc->encoder.Real_offset = 0.0f;
            foc->current.Current_offset0 = 0.0f;
            foc->current.Current_offset1 = 0.0f;

            // 3.清零Target数值
            foc->foc.Target_Id = 0.0f;
            foc->foc.Target_Iq = 0.0f;
            foc->foc.Target_Speed = 0.0f;
            foc->foc.Target_Pos = 0.0f;

            // 4.给零位电机位置
            foc->foc.sine = 0.0f;
            foc->foc.cosine = 1.0f;

            float Ualpha, Ubeta;
            ipark(&Ualpha, 
                &Ubeta, 0.0f, 0.0f, 
                0.0f, 1.0f);
            PWM_Tick(foc,
                (Ualpha/foc->foc.Real_VBUS),
                (Ubeta/foc->foc.Real_VBUS));
        }
        status = foc->status;
    }
}

// Status判断DISABLED状态机的切换
static void Status_Switch_STANDBY(FOC_System_STRUCT *foc,uint32_t *count){
    foc->status = MOTOR_STATUS_STANDBY;
    *count = 0;
    // 已失能状态持续“自定义”次控制周期后，自动切换退出已失能状态
}

// Status判断IDLE状态机的切换
static void Status_Switch_IDLE(FOC_System_STRUCT *foc,uint32_t *count){
    foc->status = MOTOR_STATUS_IDLE;
    *count = 0;
}

// Status判断过压保护状态
static void Status_VBUS_OVERVOLTAGE(FOC_System_STRUCT *foc,uint32_t *count){
    if (foc->foc.Real_VBUS > foc->safe.VBUS_MAX){
        Status_Switch_STANDBY(foc,count);
    }
}

// Status判断欠压保护状态
static void Status_VBUS_UNDERVOLTAGE(FOC_System_STRUCT *foc,uint32_t *count){
    if (foc->foc.Real_VBUS < foc->safe.VBUS_MIM){
        Status_Switch_STANDBY(foc,count);
    }
}

// Status判断过温保护状态
static void Status_Temp_OVERTEMPERATURE(FOC_System_STRUCT *foc,uint32_t *count){
    if (foc->foc.Real_Temp > foc->safe.Temp_MAX){
        Status_Switch_STANDBY(foc,count);
    }
}

// Status判断低温保护状态
static void Status_Temp_UNDERTEMPERATURE(FOC_System_STRUCT *foc,uint32_t *count){
    if (foc->foc.Real_Temp < foc->safe.Temp_MIN){
        Status_Switch_STANDBY(foc,count);
    }
}

// Status判断过流保护状态
static void Status_Current_OVERCURRENT(FOC_System_STRUCT *foc,uint32_t *count){
    if ((foc->current.Real_Id > foc->safe.Dcur_MAX) || 
        ((foc->current.Real_Iq > foc->safe.Qcur_MAX))){
        Status_Switch_STANDBY(foc,count);
    }
}

// Status判断并切换状态机
static void Status_Switch_Loop(FOC_System_STRUCT *foc){
    if (foc->status < MOTOR_STATUS_IDLE){
        return;
    }
    
    // ====== 安全状态(状态) ======
    if ((foc->status != MOTOR_STATUS_EMERGENCY_STOP) && 
        EMERGENCY_STOP_Signal()){
        foc->status = MOTOR_STATUS_EMERGENCY_STOP;
    }
    if ((foc->status != MOTOR_STATUS_DISABLED) && 
        DISABLED_Signal()){
        foc->status = MOTOR_STATUS_DISABLED;
    }

    // ====== 初始化与运行状态(状态) ======
    if ((foc->status != MOTOR_STATUS_STANDBY) && 
        STANDBY_Signal()){ // [重要]接收到待机信号后，解除锁定状态...进入待机状态
        foc->status = MOTOR_STATUS_STANDBY;
    }
    if ((foc->status == MOTOR_STATUS_EMERGENCY_STOP) || 
        (foc->status == MOTOR_STATUS_DISABLED) || (foc->status < 4)){
        return; // 紧急停止和失能状态优先级最高, 直接返回不执行后续状态判断
    }
    if ((foc->status != MOTOR_STATUS_START) && 
        START_Signal()){ // [重要]接收到开始信号后，切换到未初始化状态...准备初始化
        foc->status = MOTOR_STATUS_START;
    }

    // ====== 硬件相关错误(状态) ======
    // 1.电机母线电压VBUS状态机
    if (User_VBUS_DataGet() != Value_N_INF){
        if ((foc->status != MOTOR_STATUS_OVERVOLTAGE) && 
            foc->foc.Real_VBUS > foc->safe.VBUS_MAX){
            foc->status = MOTOR_STATUS_OVERVOLTAGE;
        }
        else if ((foc->status != MOTOR_STATUS_UNDERVOLTAGE) && 
            foc->foc.Real_VBUS < foc->safe.VBUS_MIM){
            foc->status = MOTOR_STATUS_UNDERVOLTAGE;
        }
    }
    // 2.驱动器物理温度Temp状态机
    if (User_Temperature_DataGet() != Value_N_INF){        
        if ((foc->status != MOTOR_STATUS_OVERTEMPERATURE) && 
            foc->foc.Real_Temp > foc->safe.Temp_MAX){
            foc->status = MOTOR_STATUS_OVERTEMPERATURE;
        }
        else if ((foc->status != MOTOR_STATUS_UNDERTEMPERATURE) && 
            foc->foc.Real_Temp < foc->safe.Temp_MIN){
            foc->status = MOTOR_STATUS_UNDERTEMPERATURE;
        }
    }
    // 3.过流保护
    if ((foc->status != MOTOR_STATUS_OVERCURRENT) && 
        ((foc->current.Real_Id > foc->safe.Dcur_MAX) || 
        (foc->current.Real_Iq > foc->safe.Qcur_MAX))){
        foc->status = MOTOR_STATUS_OVERCURRENT;
    }
    // 4.编码错误
    if ((foc->status != MOTOR_STATUS_ENCODER_ERROR) && 
        ENCODER_ERROR_Signal()){
        foc->status = MOTOR_STATUS_ENCODER_ERROR;
    }
    // 5.传感器错误
    if ((foc->status != MOTOR_STATUS_SENSOR_ERROR) && 
        SENSOR_ERROR_Signal()){
        foc->status = MOTOR_STATUS_SENSOR_ERROR;
    }
    if (foc->status >= 14 && foc->status < 21){
        return; // 硬件相关错误状态优先级高于运行状态, 直接返回不执行后续状态判断
    }

    // ====== 运行状态(当前反馈) ======
    // 1.力矩模式检测
    if ((CONFIG_MODE == MODE_Current_SINGLE) && 
        (foc->status != MOTOR_STATUS_TORQUE_CONTROL) && 
        (foc->current.Real_Iq > (foc->foc.Target_Iq - foc->safe.Current_limit)) && 
        (foc->current.Real_Iq < (foc->foc.Target_Iq + foc->safe.Current_limit))){
        foc->status = MOTOR_STATUS_TORQUE_CONTROL;
    }
    if ((CONFIG_MODE == MODE_Current_SINGLE) && 
        (foc->status != MOTOR_STATUS_TORQUE_INCREASING) && 
        (foc->current.Real_Iq < (foc->foc.Target_Iq + foc->safe.Current_limit))){
        foc->status = MOTOR_STATUS_TORQUE_INCREASING;
    }
    if ((CONFIG_MODE == MODE_Current_SINGLE) && 
        (foc->status != MOTOR_STATUS_TORQUE_DECREASING) && 
        (foc->current.Real_Iq > (foc->foc.Target_Iq - foc->safe.Current_limit))){
        foc->status = MOTOR_STATUS_TORQUE_DECREASING;
    }
    // 2.速度模式检测
    if (IS_Vel_MODE && 
        (foc->status != MOTOR_STATUS_CONST_SPEED) && 
        (foc->encoder.Real_Speed > foc->foc.Target_Speed - foc->safe.Speed_limit) && 
        (foc->encoder.Real_Speed < foc->foc.Target_Speed + foc->safe.Speed_limit)){
        foc->status = MOTOR_STATUS_CONST_SPEED;
    }
    if (IS_Vel_MODE && 
        (foc->status != MOTOR_STATUS_ACCELERATING) && 
        (foc->encoder.Real_Speed < foc->foc.Target_Speed + foc->safe.Speed_limit)){
        foc->status = MOTOR_STATUS_ACCELERATING;
    }
    if (IS_Vel_MODE && 
        (foc->status != MOTOR_STATUS_DECELERATING) && 
        (foc->encoder.Real_Speed > foc->foc.Target_Speed - foc->safe.Speed_limit)){
        foc->status = MOTOR_STATUS_DECELERATING;
    }
    // 3.位置模式检测
    if ((CONFIG_MODE == MODE_PosVelCur_THREE) && 
        (foc->status != MOTOR_STATUS_POSITION_HOLD) && 
        (foc->encoder.Real_Pos > foc->foc.Target_Pos - foc->safe.Position_limit) && 
        (foc->encoder.Real_Pos < foc->foc.Target_Pos + foc->safe.Position_limit)){
        foc->status = MOTOR_STATUS_POSITION_HOLD;
    }
    if ((CONFIG_MODE == MODE_PosVelCur_THREE) && 
        (foc->status != MOTOR_STATUS_POSITION_INCREASING) && 
        (foc->encoder.Real_Pos < foc->foc.Target_Pos + foc->safe.Position_limit)){
        foc->status = MOTOR_STATUS_POSITION_INCREASING;
    }
    if ((CONFIG_MODE == MODE_PosVelCur_THREE) && 
        (foc->status != MOTOR_STATUS_POSITION_DECREASING) && 
        (foc->encoder.Real_Pos > foc->foc.Target_Pos - foc->safe.Position_limit)){
        foc->status = MOTOR_STATUS_POSITION_DECREASING;
    }
}

// Status定时器中断调用的状态机运行函数
static void Status_RUN_Loop(FOC_System_STRUCT *foc){
    // 1.状态机失能DISABLED安全状态处理
    static uint32_t count = 0;
    count++;
    if (foc->status == MOTOR_STATUS_DISABLED){
        if (count > foc->safe.DISABLED_watchdog_limit){
            Status_Switch_STANDBY(foc,&count);
        }
    }
    // 2.电机电压异常_错误处理
    if (foc->status == MOTOR_STATUS_OVERVOLTAGE){
        if (count % foc->safe.VBUS_watchdog_limit == 0){
            Status_VBUS_OVERVOLTAGE(foc,&count);
        }
        if (count > foc->safe.VBUS_watchdog_limit*10){ 
            Status_Switch_IDLE(foc,&count);
            // 持续自定义的周期后，自动切换退出过压保护，并进入STANDBY
        }
    }
    if (foc->status == MOTOR_STATUS_UNDERVOLTAGE){
        if (count % foc->safe.VBUS_watchdog_limit == 0){
            Status_VBUS_UNDERVOLTAGE(foc,&count);
        }
        if (count > foc->safe.VBUS_watchdog_limit*10){ 
            Status_Switch_IDLE(foc,&count);
            // 持续自定义的周期后，自动切换退出过压保护，并进入STANDBY
        }
    }
    // 3.电机驱动器温度异常_错误处理
    if (foc->status == MOTOR_STATUS_OVERTEMPERATURE){
        if (count % foc->safe.Temp_watchdog_limit == 0){
            Status_Temp_OVERTEMPERATURE(foc,&count);
        }
        if (count > foc->safe.Temp_watchdog_limit*10){ 
            Status_Switch_IDLE(foc,&count);
            // 持续自定义的周期后，自动切换退出过压保护，并进入STANDBY
        }
    }
    if (foc->status == MOTOR_STATUS_UNDERTEMPERATURE){
        if (count % foc->safe.Temp_watchdog_limit == 0){
            Status_Temp_UNDERTEMPERATURE(foc,&count);
        }
        if (count > foc->safe.Temp_watchdog_limit*10){ 
            Status_Switch_IDLE(foc,&count);
            // 持续自定义的周期后，自动切换退出过压保护，并进入STANDBY
        }
    }
    // 4.电机过流保护OVERCURRENT错误处理
    if (foc->status == MOTOR_STATUS_OVERCURRENT){
        if (count % foc->safe.DQcur_watchdog_limit == 0){
            Status_Current_OVERCURRENT(foc,&count);
        }
        if (count > foc->safe.DQcur_watchdog_limit*10){ 
            Status_Switch_IDLE(foc,&count);
            // 持续自定义的周期后，自动切换退出过压保护，并进入STANDBY
        }
    }
    MotorStatus_Loop(&FOC.status);
}

// Printf电机数据正常发送
static void Printf_Normal_Loop(FOC_System_STRUCT *foc){
    #if !CONFIG_Printf
    // 发送数据到上位机
    Printf_TX_Loop(&FOC.txdata);
    #endif // CONFIG_Printf
}

// Printf电机调试信息发送
static void Printf_Debug_Loop(FOC_System_STRUCT *foc){
    #if CONFIG_Printf==1
    static uint8_t status = 0xFF;
    static uint32_t count = 0;
    if (foc->status != status){
        static const char* status_names[] = {
            // ====== 初始化与运行状态(状态) ======
            "待机MOTOR_STATUS_STANDBY",
            "未初始化MOTOR_STATUS_UNINITIALIZED",
            "初始化中MOTOR_STATUS_INITIALIZING",
            "校准MOTOR_STATUS_CALIBRATING",
            // ====== 运行状态(当前反馈) ======
            "空闲MOTOR_STATUS_IDLE",
            "力矩增大中~电流模式MOTOR_STATUS_TORQUE_INCREASING",
            "力矩减小中~电流模式MOTOR_STATUS_TORQUE_DECREASING",
            "力矩保持~电流模式MOTOR_STATUS_TORQUE_HOLDING",
            "加速中~速度模式MOTOR_STATUS_ACCELERATING",
            "减速中~速度模式MOTOR_STATUS_DECELERATING",
            "恒速保持~速度模式MOTOR_STATUS_CONST_SPEED",
            "位置增加中~位置模式MOTOR_STATUS_POSITION_INCREASING",
            "位置减少中~位置模式MOTOR_STATUS_POSITION_DECREASING",
            "位置保持~位置模式MOTOR_STATUS_POSITION_HOLD",
            // ====== 硬件相关错误(状态) ======
            "过压保护MOTOR_STATUS_OVERVOLTAGE",
            "欠压保护MOTOR_STATUS_UNDERVOLTAGE",
            "过温保护MOTOR_STATUS_OVERTEMPERATURE",
            "低温保护MOTOR_STATUS_LOWTEMPERATURE",
            "过流保护MOTOR_STATUS_OVERCURRENT",
            "编码器故障MOTOR_STATUS_ENCODER_ERROR",
            "传感器故障MOTOR_STATUS_SENSOR_ERROR",
            "PWM计算错误MOTOR_STATUS_PWM_ERROR",
            // ====== 安全状态(状态) ======
            "急停MOTOR_STATUS_EMERGENCY_STOP",
            "已失能MOTOR_STATUS_DISABLED"
        };
        
        printf("[电机控制,状态机更新%lu]: %s...状态机编号%d\n", 
                   count, 
                   status_names[foc->status], 
                   foc->status);
        count ++;
        status = foc->status;
    }
    #endif // CONFIG_Printf
}

// Printf电机抗齿槽数据发送(1800个数据，9列，每行10个数据)
static void Printf_Cogging_Loop(void){
    #if CONFIG_Printf==2
    const int total_num = 1800;    // 总数据个数
    const int col_per_line = 9;    // 每行打印9个
    const int rows_per_time = 10;  // 每次打印10行
    
    int printed_count = 0;
    
    for (int i = 0; i < total_num; i += rows_per_time*col_per_line){
        for (int row = 0; row < rows_per_time; row++){
            int start_idx = i + row * col_per_line;
            if (start_idx >= total_num)
                break;
                
            for (int col = 0; col < col_per_line; col++){
                int idx = start_idx + col;
                if (idx >= total_num)
                    break;
                
                int16_t value = Cogging_ReadIq_Tick((uint16_t)idx);
                if (col == 0) printf("0x%04X", (uint16_t)value);
                else printf(",0x%04X", (uint16_t)value);
            }
            printf("\n");
        }
        
        printed_count += rows_per_time*col_per_line;
        if (printed_count < total_num){
            User_Delay(1000);
        }
    }
    #endif // CONFIG_Printf
}

uint8_t count = 0;

// FOC计算电流和角度相关数据并运算控制器
static void Calculate_High_Loop(FOC_System_STRUCT *foc){
    // MOTOR_STATUS_INITIALIZING，部分权限开启
    // (SVPWM/SPWM使能，电流计算使能，编码器运算使能)
    if (foc->status >= MOTOR_STATUS_INITIALIZING){
        // 1.电机相线和各轴电流计算
        Current_Tick(foc);

        // 2.电机角度和角速度计算，并相位补偿
        Encoder_Tick[Value_set(CONFIG_MODE,
            MODE_Debug_HN,0)](foc);
        MIT_Protocol_FeedbackUpdate();
        #if CONFIG_AngleComp
        foc->value.AngleComp_Re = foc->encoder.Real_Re + 
                                AngleComp_Loop(
                                foc->encoder.Real_We, 
                                foc->value.AngleComp_Td, 
                                foc->value.AngleComp_Offset);
        fast_sin_cos(foc->value.AngleComp_Re,
                    &foc->foc.sine,
                    &foc->foc.cosine);          
        #else // CONFIG_AngleComp
        fast_sin_cos(foc->encoder.Real_Re,
                    &foc->foc.sine,
                    &foc->foc.cosine);
        #endif // CONFIG_AngleComp

        // 3.外载“无感观测器”
        Debug_Tick[Value_set(CONFIG_MODE,
            MODE_Debug_HN,0)](foc);
    }

    // MOTOR_STATUS_START->MOTOR_STATUS_INITIALIZING
    if ((foc->status == MOTOR_STATUS_START) || 
        (foc->status == MOTOR_STATUS_INITIALIZING)){
        // 4.电机零位钳制
        foc->foc.sine = 0.0f;
        foc->foc.cosine = 1.0f;
    }

    // MOTOR_STATUS_START->MOTOR_STATUS_CALIBRATING
    if ((foc->status >= MOTOR_STATUS_START) && 
        (foc->status <= MOTOR_STATUS_CALIBRATING)){
        // 5.偏置计算和电机参数辨识、电机极性辨识
        Init_Tick(foc);
    }

    // MOTOR_STATUS_IDLE，所有权限开启
    // (SVPWM/SPWM使能，电流计算使能)
    // (编码器运算使能，控制器使能)
    if (foc->status >= MOTOR_STATUS_IDLE){
        // 6.运算控制器输出
        MIT_Protocol_ControlLoop();
        Control_Tick[Value_set(CONFIG_MODE,
            MODE_Debug_HN,0)](foc);   
    }
    
    // MOTOR_STATUS_START，部分权限开启
    // (SVPWM/SPWM使能）
    if (foc->status >= MOTOR_STATUS_START){
        ipark(&foc->foc.Ualpha, 
            &foc->foc.Ubeta, 
            foc->foc.Ud_in, 
            foc->foc.Uq_in, 
            foc->foc.sine, 
            foc->foc.cosine);

        // 7.计算PWM调制量并执行
        PWM_Tick(foc,
            (foc->foc.Ualpha/foc->foc.Real_VBUS),
            (foc->foc.Ubeta/foc->foc.Real_VBUS));
    }
}

// FOC计算电机状态机切换时机
static void Calculate_Low_Loop(FOC_System_STRUCT *foc){
    MIT_Protocol_1msLoop();

    // 1.读取母线电压VBUS和温度数据Temp
    Status_Protection_Loop(foc);

    // 2.根据环境切换电机状态机
    Status_Switch_Loop(foc);
    
    // 3.根据电机状态机,运行不同任务
    Status_RUN_Loop(foc);
}

// FOC系统开始的核心文件，主任务初始化函数
static void Calculate_main_Loop(FOC_System_STRUCT *foc){
    if (foc->status == MOTOR_STATUS_START){
        // 1.MOTOR_STATUS_START，部分权限开启
        // (SVPWM/SPWM使能）
        // (此时，用户无电机控制权)
        Transfer_Init(foc);
        Offset_Current_Init(foc);
        // (Offset需要电机零位)
        Offset_Rad_Init[Value_set(CONFIG_MODE, 
            MODE_Debug_HN,0)](foc);
		
		// 如果强拖，这里定位零点
        foc->foc.Ud_in = 3.0f;
        User_Delay(1200);
        foc->foc.Ud_in = 0.0f;
        User_Delay(800);

        // 2.MOTOR_STATUS_INITIALIZING，部分权限开启
        // (SVPWM/SPWM使能，电流计算使能，编码器运算使能)
        // (此时，用户无电机控制权)
        foc->status = MOTOR_STATUS_INITIALIZING;
        // (Offset需要电机零位)
        // (Identify需要电机零位)
        // Offset_HFI_Init(foc);
        // Identify_ReadRs_Init(&foc->motor.identify);

        // 3.MOTOR_STATUS_CALIBRATING，所有权限开启
        // (SVPWM/SPWM使能，电流计算使能)
        // (编码器运算使能)
        // (电机零位钳制关闭)
        // (此时，用户无电机控制权)
        foc->status = MOTOR_STATUS_CALIBRATING;
        // (Identify无需电机零位)
        // Identify_ReadFlux_Init(&foc->motor.identify);

        // 4.MOTOR_STATUS_IDLE，所有权限开启
        // (SVPWM/SPWM使能，电流计算使能)
        // (编码器运算使能，控制器使能)
        // (此时，用户交接...拿到电机控制权)
        foc->status = MOTOR_STATUS_IDLE;
    }
}

/**
 * @description: FOC核心文件，定时中断服务函数(高频率电机载波)
 * @reminder: 10Khz或者更高定时中断中调用，任务优先级“最高”
 * @return {*}
 */
void FOC_High_Loop(void){
    static uint8_t PWM_watchdog_counter = 0;

    // ISR标志位开启，代表High_Loop正在执行
    // (如果Low_Loop抢断控制权，则对方会退出)
    FOC.flag.in_PWM_Calc_ISR = 1;
    if (!FOC.flag.PWM_Calc){
        // 1.“PWM计算中”标志位，如果计算正常则计算结束后赋值为0
        // (如果计算出错，则保持为1，counter就会记录出错次数)
        FOC.flag.PWM_Calc = 1;

        // 2.如果在初始化完成，进入函数
        // (如果计算超时，会更新错误状态并停用此线程)
        if ((FOC.status < MOTOR_STATUS_ENCODER_ERROR)){          
            // 计算编码器和电流，运算PID并执行SVPWM
            Calculate_High_Loop(&FOC);
        }

        // 3.如果出现错误状态，在此清零参数
        Status_Zero(&FOC);

        // 4.如果开启Debug，则在此对应电机实时数据
        #if !CONFIG_UART_MODE && (CONFIG_Printf==1)
        Printf_Debug_Loop(&FOC);
        #endif // CONFIG_Printf

        // 5.PWM_Calc，计算正常计算结束赋值为0
        // (同时清空出错次数记录，代表新的计算正常开始)
        FOC.flag.PWM_Calc = 0;
        PWM_watchdog_counter = 0;
    }
    else{
        // 6.计算出错一次则会记录一次
        // (同时恢复High_loop内部内容的计算，看后面是否还会出错)
        PWM_watchdog_counter++;
        FOC.flag.PWM_Calc = 0;

        // 7.出错次数达到上限，电机停机并更新状态机
        // (清空出错次数，为下次记录做准备)
        if (PWM_watchdog_counter > FOC.flag.PWM_watchdog_limit){
            FOC.status = MOTOR_STATUS_PWM_CALC_FAULT;
            PWM_watchdog_counter = 0;
        }
    }

    // ISR标志位关闭，代表High_Loop执行完毕
    // (如果Low_Loop抢断控制权，则对方可以执行)
    FOC.flag.in_PWM_Calc_ISR = 0;
}

/**
 * @description: FOC核心文件，定时中断服务函数(1ms周期数据更新)
 * @reminder: 1Khz或者更低定时中断中调用，任务优先级“中”
 * @return {*}
 */
void FOC_Low_Loop(void){
    // ISR标志位开启，PWM计算中断正在执行
    // (则会在此时跳过当前Low_Loop函数)
    if (FOC.flag.in_PWM_Calc_ISR) {
        return;
    }
    
    // 执行电机状态机切换函数和任务
    Calculate_Low_Loop(&FOC);
}

/**
 * @description: FOC核心文件，UART或者CAN接收完成中断服务函数
 * @reminder: 主循环函数调用，任务优先级“低”
 * @param {uint8_t} *data 接收到的数据
 * @param {uint16_t} length 数据长度
 * @return {*}
 */
void FOC_Printf_Loop(const uint8_t *data, uint16_t length){
    // 串口仅解析达妙控制模式切换命令，例如MODE=MIT?或MODE=3?
    Printf_RX_Loop(data,length);
}

/**
 * @description: FOC核心文件，主循环服务函数(主循环TXdata数据更新)
 * @reminder: 主循环函数调用，任务优先级“最低”
 * @return {*}
 */
void FOC_main_Loop(void){
    // 1.上电即初始化的函数
    // (包含printf的收发初始化和Initial_Init)
    static uint8_t count = 0;
    if (!count){
        Printf_TX_Init(&FOC.txdata);
        Printf_RX_Init();
        MIT_Protocol_Init();
        MIT_FDCAN_Init();
        User_Initial_Init();
        count = !count;
    }

    // 2.处理FDCAN接收的MIT命令
    MIT_FDCAN_MainLoop();

    // 3.接收到开启电机才初始化
    // (即电机状态机为FOC.status==1)
    Calculate_main_Loop(&FOC);

    // 4.正常运行时串口打印数据，或Cogging数据打印
    #if !CONFIG_UART_MODE
    if ((FOC.status >= MOTOR_STATUS_IDLE) &&
        (FOC.status < MOTOR_STATUS_ENCODER_ERROR)){
        #if CONFIG_Printf==0
        Printf_Normal_Loop(&FOC);
        #elif CONFIG_Printf==2
        Printf_Cogging_Loop();
        #endif // CONFIG_Printf
    }
    #endif
}
