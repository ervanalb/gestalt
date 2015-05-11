// This file is required by the standard peripheral library.

#include "stm32f0xx_rcc.h"
#include "stm32f0xx_usart.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_misc.h"
#include "stm32f0xx_tim.h"
#include "stm32f0xx_i2c.h"
#include "stm32f0xx_adc.h"
#include "stm32f0xx_dma.h"

void assert_failed(const char* file, const int line);

#ifdef DEBUG
    #define assert_param(x) {if(!(x)) assert_failed(__FILE__, __LINE__);}
#else
    #define assert_param(x) {}
#endif
