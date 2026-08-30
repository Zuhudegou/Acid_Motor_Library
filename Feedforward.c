#include "Feedforward.h"

/**
 * @description: D轴电流前馈量
 * @param {float} We 电角速度
 * @param {float} Lq Q轴电感
 * @param {float} Iq Q轴电流
 * @return {float}
 */
float Feedforward_CurrentD(float We,float Lq,float Iq){
    return -(We*Lq*Iq);
}

/**
 * @description: Q轴电流前馈量
 * @param {float} We 电角速度
 * @param {float} Ld D轴电感
 * @param {float} Id D轴电流
 * @param {float} Flux 磁链
 * @return {float}
 */
float Feedforward_CurrentQ(float We,float Ld,float Id,float Flux){
    return We*(Ld*Id + Flux);
}

/**
 * @description: 转速环速度前馈量
 * @param {float} Speed 机械角速度
 * @param {float} Ba 电机转速环带宽
 * @return {float}
 */
float Feedforward_Velocity(float Speed,float Ba){
    return -Speed*Ba;
}

/**
 * @description: 扰动观测器前馈量
 * @param {float} Fd 观测的扰动力矩
 * @param {float} Pn 电机极对数
 * @param {float} Flux 磁链
 * @return {float}
 */
float Feedforward_DOB(float Fd,float Pn,float Flux){
    return Fd/(1.5f*Pn*Flux);
}

