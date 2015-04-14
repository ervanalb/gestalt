#ifndef __HAL_H
#define __HAL_H

#include <stdint.h>

void hal_init();
void hal_setXYCurrent(uint16_t c);
void hal_setZCurrent(uint16_t c);

void hal_changeX(int32_t deltaX);
void hal_changeY(int32_t deltaY);
void hal_changeZ(int32_t deltaZ);

uint8_t hal_leftButton();
uint8_t hal_rightButton();

#endif
