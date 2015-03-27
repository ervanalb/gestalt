#ifndef __HAL_H
#define __HAL_H

#include <stdint.h>

void hal_init();
void hal_setXYCurrent(uint16_t c);
void hal_setZCurrent(uint16_t c);

#endif
