#include "motors.h"
#include "stm32f0xx.h"

#define TICKS_PER_SEC 32000

#define SECS_TO_TICKS(t) (int32_t)(((int64_t)(TICKS_PER_SEC) * (t)) >> 16)
#define PERSEC_TO_PERTICK(f) ((f) / (TICKS_PER_SEC))

motor_t motor_x;
motor_t motor_y;
motor_t motor_z;

void motor_moveTo(motor_t* m, int32_t p, int32_t t)
{
    int32_t v;

    if(t > 0)
    {
        v = (p - m->p) / SECS_TO_TICKS(t); // Calculate velocity
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

void motor_jog(motor_t* m, int32_t v, int32_t t)
{
    __disable_irq();
    m->target_p = m->p + (int32_t)(((int64_t)v * t) >> 16);
    m->v = PERSEC_TO_PERTICK(v);
    __enable_irq();
}

void motor_zero(motor_t* m, int32_t p)
{
    __disable_irq();
    // This is inelegant
    if(m == &motor_x) hal_changeX(p - m->p);
    if(m == &motor_y) hal_changeY(p - m->p);
    if(m == &motor_z) hal_changeZ(p - m->p);

    m->p = p;
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
        m->v = 0;
    }
}

