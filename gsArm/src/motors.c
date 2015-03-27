#include "motors.h"
#include "stm32f0xx.h"

#define TICKS_PER_SEC 32000

motor_t motor_x;
motor_t motor_y;
motor_t motor_z;

void motor_moveTo(motor_t* m, int32_t p, int32_t t)
{
    int32_t v;

    if(t > 0)
    {
        v = (p - m->p) / (((int64_t)TICKS_PER_SEC * t) >> 16); // Calculate velocity
    }
    else
    {
        v = 0;
    }
    __disable_irq();
    m->target_p = p;
    m->v = v;
    __enable_irq();
}

void motor_update(motor_t* m)
{
    if((m->target_p > m->p && m->v >= 0) ||
       (m->target_p < m->p && m->v <= 0)) // Not at position yet
    {
        m->p += m->v;
    }
    else // At or past position
    {
        m->p = m->target_p;
    }
}

