#include "common.h"
#include <math.h>


#define rc_elev channels[0]
#define rc_aile channels[1]
#define rc_thro channels[2]
#define rc_rudd channels[3]
#define rc_gear channels[4]
#define rc_mix channels[5]
#define rc_aux channels[6]
uint8_t read_mode(void);
extern float offset_gx,offset_gy,offset_gz;
extern float offset_gyro[3];