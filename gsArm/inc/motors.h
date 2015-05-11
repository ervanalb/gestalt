#ifndef __MOTORS_H
#define __MOTORS_H

#include <stdint.h>

typedef struct motor
{
    // All three of these are 16.16 fixed point
    int32_t p;
    int32_t target_p;
    int32_t v;
    uint8_t stop_on_limit;
} motor_t;

extern motor_t motor_x;
extern motor_t motor_y;
extern motor_t motor_z;

void motor_moveTo(motor_t* m, int32_t p, int32_t t);
void motor_jog(motor_t* m, int32_t v, int32_t t);
void motor_zero(motor_t* m, int32_t p);
void motor_setSoftUpperLimit(motor_t* m, int32_t p);
void motor_setSoftLowerLimit(motor_t* m, int32_t p);
void motor_stopOnLimit(motor_t* m, uint8_t limit);

void motor_update(motor_t* m, uint8_t limits);

#endif
