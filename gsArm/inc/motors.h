#ifndef __MOTORS_H
#define __MOTORS_H

#include <stdint.h>

typedef struct motor
{
    // Store p as 64 bits to avoid rounding error, but only the highest 16 are
    // used to calculate position
    int32_t p;
    int32_t target_p;
    int32_t v;
} motor_t;

extern motor_t motor_x;
extern motor_t motor_y;
extern motor_t motor_z;

void motor_moveTo(motor_t* m, int32_t p, int32_t v);
void motor_update(motor_t* m);

#endif
