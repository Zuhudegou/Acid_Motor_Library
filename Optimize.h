#ifndef __OPTIMIZE_H
#define __OPTIMIZE_H

/* FOC配置文件声明 */
#include "Config.h"

void MTPA_Loop(float *Target_id, 
            float flux, 
            float Ld, 
            float Lq, 
            float iq);
float FW_Loop(void *fw, 
            float Ud, 
            float Uq, 
            float Real_Speed, 
            float Base_Speed, 
            float Percentage, 
            float Vbus);
void DeadZone_Loop(float *Ua_duty, 
            float *Ub_duty, 
            float *Uc_duty, 
            float Ia, 
            float Ib, 
            float Ic, 
            float Current_Min, 
            float Dead_Time);
float AngleComp_Loop(float We,float Td,float Offset);


#endif // OPTIMIZE_H
