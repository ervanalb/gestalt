#include "stm32f0xx.h"
#include "hal.h"
#include "gsnode_hal.h"
#include "motors.h"

#define WATCHDOG_TIMEOUT (48 * GSNODE_TIMER_US / CL_PERIOD)

#define STEP_PERIOD 375 // Step output loop runs at 128 KHz
#define CL_PERIOD 1500 // Control loop runs at 32 KHz
#define CUR_PWM 1500 // PWM outputs at 32 KHz
#define STEP_PULSE_WIDTH 100 // ~STEP pulse width of ~2 us

static volatile uint32_t steps_fwd=0;
static volatile uint32_t steps_bak=0;

static int32_t xp;
static int32_t yp;
static int32_t zp;

static int32_t xp_soft_lower_limit;
static int32_t xp_soft_upper_limit;
static int32_t yp_soft_lower_limit;
static int32_t yp_soft_upper_limit;
static int32_t zp_soft_lower_limit;
static int32_t zp_soft_upper_limit;

static uint8_t x_limits;
static uint8_t y_limits;
static uint8_t z_limits;

static int32_t gsTimer;

#define NUM_ADC_CHANNELS 3
#define ADC_INDEX_HANDLE_AUX1 0
#define ADC_INDEX_HANDLE_AUX2 1
#define ADC_INDEX_FORCE_SENSE 2

static uint16_t adc_readings[NUM_ADC_CHANNELS];

static uint16_t z_force_sense_threshold;

void hal_init()
{
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    I2C_InitTypeDef I2C_InitStruct;
    TIM_OCInitTypeDef TIM_OCInitStruct;
    DMA_InitTypeDef DMA_InitStruct;
    ADC_InitTypeDef ADC_InitStruct;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB |
                          RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_GPIOF |
                          RCC_AHBPeriph_DMA1, ENABLE);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14 | RCC_APB1Periph_I2C1 |
                           RCC_APB1Periph_TIM3, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM16 |
                           RCC_APB2Periph_TIM17 | RCC_APB2Periph_USART1 |
                           RCC_APB2Periph_ADC1, ENABLE);

    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;

    // USART
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9,  GPIO_AF_1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);

    USART_InitStruct.USART_BaudRate = 115200;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStruct);

    USART_ClearITPendingBit(USART1, USART_IT_TC);

    NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 2;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
    
    USART_Cmd(USART1, ENABLE);

    // I2C
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Init(GPIOF, &GPIO_InitStruct);

    GPIO_PinAFConfig(GPIOF, GPIO_PinSource0, GPIO_AF_1);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource1, GPIO_AF_1);

    RCC_I2CCLKConfig(RCC_I2C1CLK_HSI);

    //I2C_InitStruct.I2C_Timing = 0x1045061D; // LM75
    I2C_InitStruct.I2C_Timing = 0x50330309; // How's that for magic constants
    I2C_InitStruct.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
    I2C_InitStruct.I2C_DigitalFilter = 0;
    I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
    //I2C_InitStruct.I2C_Mode = I2C_Mode_SMBusHost;
    I2C_InitStruct.I2C_OwnAddress1 = 0;
    I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &I2C_InitStruct);

    I2C_Cmd(I2C1, ENABLE);

    // Current pins and LED
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_11;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_2);

    TIM_TimeBaseInitStruct.TIM_Prescaler = 0;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStruct.TIM_Period = CUR_PWM - 1;
    TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
    TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStruct);

    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStruct.TIM_Pulse = 0;
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStruct.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStruct.TIM_OCNIdleState = TIM_OCNIdleState_Set;
    TIM_OC1Init(TIM1, &TIM_OCInitStruct);
    TIM_OC2Init(TIM1, &TIM_OCInitStruct);
    TIM_OC3Init(TIM1, &TIM_OCInitStruct);
    TIM_OC4Init(TIM1, &TIM_OCInitStruct);

    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);

    // STEP pins
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_2);

    TIM_TimeBaseInitStruct.TIM_Prescaler = 0;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStruct.TIM_Period = STEP_PULSE_WIDTH - 1;
    TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
    TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;

    TIM_TimeBaseInit(TIM3,  &TIM_TimeBaseInitStruct);
    TIM_TimeBaseInit(TIM16, &TIM_TimeBaseInitStruct);
    TIM_TimeBaseInit(TIM17, &TIM_TimeBaseInitStruct);

    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStruct.TIM_Pulse = 1;
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStruct.TIM_OCNPolarity = TIM_OCNPolarity_Low;
    TIM_OCInitStruct.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

    TIM_SelectOnePulseMode(TIM3,  TIM_OPMode_Single);
    TIM_SelectOnePulseMode(TIM16, TIM_OPMode_Single);
    TIM_SelectOnePulseMode(TIM17, TIM_OPMode_Single);

    //TIM_CtrlPWMOutputs(TIM3,  ENABLE);
    TIM_CtrlPWMOutputs(TIM16, ENABLE);
    TIM_CtrlPWMOutputs(TIM17, ENABLE);

    TIM_OC1Init(TIM3,  &TIM_OCInitStruct);
    TIM_OC1Init(TIM16, &TIM_OCInitStruct);
    TIM_OC1Init(TIM17, &TIM_OCInitStruct);

    // DIR pins
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_13;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
    GPIO_Init(GPIOF, &GPIO_InitStruct);

    // Microstep pins
    // 1/32 microstepping
    GPIO_SetBits(GPIOA, GPIO_Pin_4);
    GPIO_SetBits(GPIOB, GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_6 | GPIO_Pin_10 | GPIO_Pin_11);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_6 | GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Reset pin
    GPIO_SetBits(GPIOA, GPIO_Pin_15);
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // SysTick runs control loop
    SysTick->LOAD = CL_PERIOD - 1;
    NVIC_SetPriority(SysTick_IRQn, 0);
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;

    // TIM14 runs step outputting
    TIM_TimeBaseInitStruct.TIM_Prescaler = 0;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStruct.TIM_Period = STEP_PERIOD - 1;
    TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
    TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM14, &TIM_TimeBaseInitStruct);
    TIM_Cmd(TIM14, ENABLE);

    NVIC_InitStruct.NVIC_IRQChannel = TIM14_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 3;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    TIM_ClearITPendingBit(TIM14, TIM_IT_Update);
    TIM_ITConfig(TIM14, TIM_IT_Update, ENABLE);

    // Fault
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;
    GPIO_Init(GPIOF, &GPIO_InitStruct);

    // Handle buttons
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOC, &GPIO_InitStruct);

    // Z limit switch
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    // ADC DMA
    DMA_DeInit(DMA1_Channel1);
    DMA_InitStruct.DMA_BufferSize = NUM_ADC_CHANNELS;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStruct.DMA_M2M = DMA_M2M_Disable;
    DMA_InitStruct.DMA_MemoryBaseAddr = (uint32_t)adc_readings;
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStruct.DMA_Priority = DMA_Priority_Low;
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);
    DMA_Init(DMA1_Channel1, &DMA_InitStruct);
    DMA_Cmd(DMA1_Channel1, ENABLE);

    // ADC
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_TRGO;
    ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Left;
    ADC_InitStruct.ADC_ScanDirection = ADC_ScanDirection_Upward;
    ADC_Init(ADC1, &ADC_InitStruct);

    ADC_ChannelConfig(ADC1, ADC_Channel_1, ADC_SampleTime_239_5Cycles);
    ADC_ChannelConfig(ADC1, ADC_Channel_2, ADC_SampleTime_239_5Cycles);
    ADC_ChannelConfig(ADC1, ADC_Channel_3, ADC_SampleTime_239_5Cycles);

    ADC_GetCalibrationFactor(ADC1);
    ADC_Cmd(ADC1, ENABLE);
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADRDY));

    ADC_DMACmd(ADC1, ENABLE);
    ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular);

    ADC_StartOfConversion(ADC1);
}

void hal_changeX(int32_t deltaX)
{
    xp += deltaX >> 11;
    xp_soft_lower_limit += deltaX;
    xp_soft_upper_limit += deltaX;
}

void hal_changeY(int32_t deltaY)
{
    yp += deltaY >> 11;
    yp_soft_lower_limit += deltaY;
    yp_soft_upper_limit += deltaY;
}

void hal_changeZ(int32_t deltaZ)
{
    zp += deltaZ >> 11;
    zp_soft_lower_limit += deltaZ;
    zp_soft_upper_limit += deltaZ;
}

void hal_setXCurrent(uint16_t c)
{
    uint16_t pwm = ((uint32_t)CUR_PWM * c) >> 16;
    TIM1->CCR3 = pwm;
}

void hal_setYCurrent(uint16_t c)
{
    uint16_t pwm = ((uint32_t)CUR_PWM * c) >> 16;
    TIM1->CCR2 = pwm;
}

void hal_setZCurrent(uint16_t c)
{
    uint16_t pwm = ((uint32_t)CUR_PWM * c) >> 16;
    TIM1->CCR4 = pwm;
}

void hal_setLowerSoftLimitX(int32_t p)
{
    xp_soft_lower_limit = motor_x.p + p;
}

void hal_setLowerSoftLimitY(int32_t p)
{
    yp_soft_lower_limit = motor_y.p + p;
}

void hal_setLowerSoftLimitZ(int32_t p)
{
    zp_soft_lower_limit = motor_z.p + p;
}

void hal_setUpperSoftLimitX(int32_t p)
{
    xp_soft_upper_limit = motor_x.p + p;
}

void hal_setUpperSoftLimitY(int32_t p)
{
    yp_soft_upper_limit = motor_y.p + p;
}

void hal_setUpperSoftLimitZ(int32_t p)
{
    zp_soft_upper_limit = motor_z.p + p;
}

uint8_t hal_getLimitsX()
{
    return x_limits;
}

uint8_t hal_getLimitsY()
{
    return y_limits;
}

uint8_t hal_getLimitsZ()
{
    return z_limits;
}

uint8_t hal_leftButton()
{
    return !(GPIO_ReadInputData(GPIOC) & GPIO_Pin_14);
}

uint8_t hal_rightButton()
{
    return !(GPIO_ReadInputData(GPIOC) & GPIO_Pin_15);
}

uint16_t hal_handleRAux1()
{
    return adc_readings[ADC_INDEX_HANDLE_AUX1];
}

uint16_t hal_handleRAux2()
{
    return adc_readings[ADC_INDEX_HANDLE_AUX2];
}

uint16_t hal_zForceSense()
{
    return adc_readings[ADC_INDEX_FORCE_SENSE];
}

void hal_setLED(uint16_t brightness)
{
    uint16_t pwm = ((uint32_t)CUR_PWM * brightness) >> 16;
    TIM1->CCR1 = pwm;
}

uint8_t hal_zLimitSwitch()
{
    return !(GPIO_ReadInputData(GPIOB) & GPIO_Pin_0);
}

void hal_setZForceSenseThreshold(uint16_t value)
{
    z_force_sense_threshold = value;
}

// gsNode_hal functions
// (required for correct operation of Gestalt library)

void gsNode_hal_writeByte(uint8_t byte)
{
    USART_SendData(USART1, byte);
    USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
}

void gsNode_hal_flushOutput()
{
    USART_ITConfig(USART1, USART_IT_TC, ENABLE);
}

void gsNode_hal_readByte()
{
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

void gsNode_hal_setTimer()
{
    gsTimer = WATCHDOG_TIMEOUT + 1;
}

void gsNode_hal_cancelTimer()
{
    gsTimer = -1;
}

void USART1_IRQHandler()
{
    if(USART_GetITStatus(USART1, USART_IT_TXE))
    {
        USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
        gsNode_hal_byteWritten();
    }
    else if(USART_GetITStatus(USART1, USART_IT_RXNE))
    {
        USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
        gsNode_hal_byteRead(USART_ReceiveData(USART1));
    }
    else if(USART_GetITStatus(USART1, USART_IT_TC))
    {
        USART_ITConfig(USART1, USART_IT_TC, DISABLE);
        USART_ClearITPendingBit(USART1, USART_IT_TC);
        gsNode_hal_outputFlushed();
    }
}

void TIM14_IRQHandler()
{
    int32_t x = motor_x.p >> 11;
    int32_t y = motor_y.p >> 11;
    int32_t z = motor_z.p >> 11;

    if(x < xp)
    {
        GPIOB->BSRR = GPIO_Pin_7<<16; // Set direction
        TIM17->CR1 |= TIM_CR1_CEN;
        xp--;
    }
    else if(x > xp)
    {
        GPIOB->BSRR = GPIO_Pin_7; // Set direction
        TIM17->CR1 |= TIM_CR1_CEN;
        xp++;
    }

    if(y < yp)
    {
        GPIOB->BSRR = GPIO_Pin_13<<16; // Set direction
        TIM3->CR1 |= TIM_CR1_CEN;
        yp--;
    }
    else if(y > yp)
    {
        GPIOB->BSRR = GPIO_Pin_13; // Set direction
        TIM3->CR1 |= TIM_CR1_CEN;
        yp++;
    }

    if(z < zp)
    {
        GPIOF->BSRR = GPIO_Pin_6<<16; // Set direction
        TIM16->CR1 |= TIM_CR1_CEN;
        zp--;
    }
    else if(z > zp)
    {
        GPIOF->BSRR = GPIO_Pin_6; // Set direction
        TIM16->CR1 |= TIM_CR1_CEN;
        zp++;
    }

    TIM14->SR = ~TIM_FLAG_Update; // Clear interrupt
}

void hal_poll()
{
    if(gsTimer == 0)
    {
        gsTimer = -1;
        gsNode_hal_timerFired();
    }
}

static void update_motor_limits()
{
    x_limits = ((motor_x.p <= xp_soft_lower_limit) ? HAL_SOFT_LOWER_LIMIT : 0) |
               ((motor_x.p >= xp_soft_upper_limit) ? HAL_SOFT_UPPER_LIMIT : 0);

    y_limits = ((motor_y.p <= yp_soft_lower_limit) ? HAL_SOFT_LOWER_LIMIT : 0) |
               ((motor_y.p >= yp_soft_upper_limit) ? HAL_SOFT_UPPER_LIMIT : 0);

    z_limits = ((motor_z.p <= zp_soft_lower_limit) ? HAL_SOFT_LOWER_LIMIT : 0) |
               ((motor_z.p >= zp_soft_upper_limit) ? HAL_SOFT_UPPER_LIMIT : 0) |
               (hal_zForceSense() > z_force_sense_threshold ? HAL_HARD_LOWER_LIMIT : 0) |
               (hal_zLimitSwitch() ? HAL_HARD_UPPER_LIMIT : 0);
}

// Systick handles updating of motors
void SysTick_Handler()
{
    if(gsTimer > 0) gsTimer--;

    update_motor_limits();

    motor_update(&motor_x, x_limits);
    motor_update(&motor_y, y_limits);
    motor_update(&motor_z, z_limits);
}

