// This file is required by the standard peripheral library.

// This line needed due to bug in standard peripheral library
#include "stm32f0xx_rcc.h"

void assert_failed(const char* file, const int line);

#ifdef DEBUG
    #define assert_param(x) {if(!(x)) assert_failed(__FILE__, __LINE__);}
#else
    #define assert_param(x) {}
#endif
