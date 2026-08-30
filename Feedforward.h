#ifndef __FEEDFORWARD_H
#define __FEEDFORWARD_H

/* FOC配置文件声明 */
#include "Config.h"

float Feedforward_CurrentD(float We,float Lq,float Iq);
float Feedforward_CurrentQ(float We,float Ld,float Id,float Flux);
float Feedforward_Velocity(float Speed,float Ba);
float Feedforward_DOB(float Fd,float Pn,float Flux);


#endif // FEEDFORWARD_H
