#ifndef __SPWM_H
#define __SPWM_H

/* FOC配置文件声明 */
#include "Config.h"

// 电机相关
void SPWM(float u_alpha, float u_beta, 
        float *d_u, float *d_v, float *d_w);


#endif // SPWM_H
