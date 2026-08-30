#ifndef __SVPWM_H
#define __SVPWM_H

/* FOC配置文件声明 */
#include "Config.h"

// 电机相关
void SVPWM(float u_alpha, float u_beta, 
        float *d_u, float *d_v, float *d_w);


#endif // SVPWM_H
