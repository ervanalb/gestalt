#include "motors.h"

motor_t motor_x;
motor_t motor_y;
motor_t motor_z;

void motor_moveTo(motor_t* m, int32_t p, int32_t v)
{
    m->target_p = p;
    m->v = v;
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
