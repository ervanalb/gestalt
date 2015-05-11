#include "motors.h"
#include "stm32f0xx.h"
#include "hal.h"

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

void motor_stopOnLimit(motor_t* m, uint8_t limit_mask)
{
    m->stop_on_limit = limit_mask;
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

void motor_setSoftUpperLimit(motor_t* m, int32_t p)
{
    __disable_irq();

    if(m == &motor_x) hal_setUpperSoftLimitX(p);
    if(m == &motor_y) hal_setUpperSoftLimitY(p);
    if(m == &motor_z) hal_setUpperSoftLimitZ(p);

   __enable_irq();
}

void motor_setSoftLowerLimit(motor_t* m, int32_t p)
{
    __disable_irq();

    if(m == &motor_x) hal_setLowerSoftLimitX(p);
    if(m == &motor_y) hal_setLowerSoftLimitY(p);
    if(m == &motor_z) hal_setLowerSoftLimitZ(p);

   __enable_irq();
}

void motor_update(motor_t* m, uint8_t limits)
{
    if(m->v >= 0)
    {
        if(HAL_UPPER_LIMITS & limits & m->stop_on_limit)
        {
            m->v = 0;
        }
        else if(m->target_p > m->p)
        {
            m->p += m->v;
        }
        else
        {
            m->p = m->target_p;
            m->v = 0;
        }
    }
    else if(m->v <= 0)
    {
        if(HAL_LOWER_LIMITS & limits & m->stop_on_limit)
        {
            m->v = 0;
        }
        else if(m->target_p < m->p)
        {
            m->p += m->v;
        }
        else
        {
            m->p = m->target_p;
            m->v = 0;
        }
    }
}

