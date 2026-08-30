#ifndef __USERDATA_USERCONTROL_H
#define __USERDATA_USERCONTROL_H
#include "FOC.h"
/* 电机控制User用户设置·实时参数控制页面 */

/**
 * @description: 1.用户的通信发送接口
 * @reminder: (此方函数->填写你需要修改的串口发送的数据)
 * @return {*}
 */
static inline void User_UserTX(void){
    // 传入串口printf要发送的数据，如txdata.fdata[0]，最多默认16个数值
    // 如需传入更多数值，请修改printf中的参数
    // ========== 组1: 状态与核心指令 (0-3) ==========
    FOC.txdata.fdata[0] = (float)FOC.status;        // 状态机当前状态码
    FOC.txdata.fdata[1] = FOC.foc.Target_Speed;     // 目标机械转速 (rad/s)
    FOC.txdata.fdata[2] = FOC.foc.Target_Iq;        // 目标Q轴电流 (A)
    FOC.txdata.fdata[3] = FOC.foc.Target_Id;        // 目标D轴电流 (A)

    // ========== 组2: 实际反馈 (4-7) ==========
    FOC.txdata.fdata[4] = FOC.encoder.Real_Speed;   // 实际机械转速 (rad/s)
    FOC.txdata.fdata[5] = FOC.current.Real_Iq;      // 实际Q轴电流 (A)
    FOC.txdata.fdata[6] = FOC.current.Real_Id;      // 实际D轴电流 (A)
    FOC.txdata.fdata[7] = FOC.encoder.Real_Pos;     // 实际机械角度 (rad)

    // ========== 组3: 控制器输出与中间变量 (8-14) ==========
    FOC.txdata.fdata[8]  = FOC.foc.Uq_in;           // Q轴电压指令 (V)
    FOC.txdata.fdata[9]  = FOC.foc.Ud_in;           // D轴电压指令 (V)
    FOC.txdata.fdata[10] = FOC.foc.Real_VBUS;       // 母线电压实际值 (V)
    FOC.txdata.fdata[11] = FOC.foc.Du;              // U相占空比 (0~1)
    FOC.txdata.fdata[12] = FOC.foc.Dv;              // V相占空比 (0~1)
    FOC.txdata.fdata[13] = FOC.foc.Dw;              // W相占空比 (0~1)

    // FOC.txdata.fdata[12] = FOC.encoder.Sensorless_Speed;
    // FOC.txdata.fdata[13] = FOC.encoder.Sensorless_Pos;
    // FOC.txdata.fdata[14] = FOC.encoder.Sensorless_We;
    // FOC.txdata.fdata[15] = FOC.encoder.Sensorless_Re;

    FOC.txdata.fdata[14] = FOC.current.Real_Ia;     // A相瞬时电流值 (A)

    // ========== 组4: 系统配置与辅助信息 (15) ==========
    FOC.txdata.fdata[15] = (float)CONFIG_MODE;        // 当前控制模式 (0-10)
}

/**
 * @description: 2.用户的通信接收接口
 * @reminder: (此方函数->填写浮点数赋值对象)
 * @reminder: (如果需要其他指令，可在printf.c中添加)
 * @param {float} data
 * @return {*}
 */
static inline void Handle_User0_Adjust(float data){
    /* Your code for Parameter set */
    // 接收到串口或者CAN的数据是User0=xx?
    // 收到指令后，会把数据赋值到data
}

/**
 * @description: 3.用户的通信接收接口
 * @reminder: (此方函数->填写浮点数赋值对象)
 * @param {float} data
 * @return {*}
 */
static inline void Handle_User1_Adjust(float data){
    /* Your code for Parameter set */
    // 接收到串口或者CAN的数据是User1=xx?
    // 收到指令后，会把数据赋值到data
}

/**
 * @description: 4.用户的通信接收接口
 * @reminder: (此方函数->填写浮点数赋值对象)
 * @param {float} data
 * @return {*}
 */
static inline void Handle_User2_Adjust(float data){
    /* Your code for Parameter set */
    // 接收到串口或者CAN的数据是User2=xx?
    // 收到指令后，会把数据赋值到data
}


#endif // USERDATA_USERCONTROL_H
