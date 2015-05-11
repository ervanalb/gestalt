#ifndef __HAL_H
#define __HAL_H

#include <stdint.h>

void hal_init();
void hal_poll();

void hal_setXCurrent(uint16_t c);
void hal_setYCurrent(uint16_t c);
void hal_setZCurrent(uint16_t c);

void hal_changeX(int32_t deltaX);
void hal_changeY(int32_t deltaY);
void hal_changeZ(int32_t deltaZ);

void hal_setLowerSoftLimitX(int32_t p);
void hal_setLowerSoftLimitY(int32_t p);
void hal_setLowerSoftLimitZ(int32_t p);

void hal_setUpperSoftLimitX(int32_t p);
void hal_setUpperSoftLimitY(int32_t p);
void hal_setUpperSoftLimitZ(int32_t p);

uint8_t hal_leftButton();
uint8_t hal_rightButton();

void hal_setLED(uint16_t brightness);

#define HAL_LOWER_LIMITS     (0x0F)
#define HAL_HARD_LOWER_LIMIT (1 << 0)
#define HAL_SOFT_LOWER_LIMIT (1 << 1)

#define HAL_UPPER_LIMITS     (0xF0)
#define HAL_HARD_UPPER_LIMIT (1 << 4)
#define HAL_SOFT_UPPER_LIMIT (1 << 5)

uint8_t hal_getLimitsX();
uint8_t hal_getLimitsY();
uint8_t hal_getLimitsZ();

uint16_t hal_handleRAux1();
uint16_t hal_handleRAux2();
uint16_t hal_zForceSense();
uint8_t hal_zLimitSwitch();
void hal_setZForceSenseThreshold(uint16_t value);

#endif
