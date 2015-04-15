#include "stm32f0xx.h"
#include "gsnode_hal.h"
#include "motors.h"

#define WATCHDOG_TIMEOUT (48 * GSNODE_TIMER_US)

#define CL_PERIOD 1500 // Control loop runs at 32 KHz
//#define TICK_PERIOD 375 // 128 KHz
#define TICK_PERIOD 1500 // 32 KHz
#define CUR_PWM 1500 // 32 KHz

static volatile uint32_t steps_fwd=0;
static volatile uint32_t steps_bak=0;

static int32_t xp;
static int32_t yp;
static int32_t zp;

void hal_init()
{
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    I2C_InitTypeDef I2C_InitStruct;
    TIM_OCInitTypeDef TIM_OCInitStruct;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB |
                          RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_GPIOF, ENABLE);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14 | RCC_APB1Periph_I2C1, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_TIM16 |
                           RCC_APB2Periph_TIM17 | RCC_APB2Periph_USART1, ENABLE);

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

    // Timeout for Gestalt Communication
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);

    TIM_TimeBaseInitStruct.TIM_Prescaler = 0;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStruct.TIM_Period = WATCHDOG_TIMEOUT - 1;
    TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
    TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM14, &TIM_TimeBaseInitStruct);
    TIM_Cmd(TIM14, ENABLE);

    NVIC_InitStruct.NVIC_IRQChannel = TIM14_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 3;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    // Timebase for sending steps
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    TIM_TimeBaseInitStruct.TIM_Period = TICK_PERIOD - 1;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStruct);
    TIM_Cmd(TIM1, ENABLE);

    NVIC_InitStruct.NVIC_IRQChannel = TIM1_BRK_UP_TRG_COM_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);

    // I2C
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Init(GPIOF, &GPIO_InitStruct);

    GPIO_PinAFConfig(GPIOF, GPIO_PinSource0, GPIO_AF_1);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource1, GPIO_AF_1);

    I2C_InitStruct.I2C_Timing = 0x50330309; // How's that for magic constants
    I2C_InitStruct.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
    I2C_InitStruct.I2C_DigitalFilter = 0;
    I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStruct.I2C_OwnAddress1 = 0;
    I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &I2C_InitStruct);

    I2C_Cmd(I2C1, ENABLE);

    // Current pins
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_2);

    TIM_TimeBaseInitStruct.TIM_Prescaler = 0;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStruct.TIM_Period = CUR_PWM - 1;
    TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
    TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM16, &TIM_TimeBaseInitStruct);
    TIM_TimeBaseInit(TIM17, &TIM_TimeBaseInitStruct);

    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStruct.TIM_Pulse = 0;
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStruct.TIM_OCNPolarity = TIM_OCNPolarity_Low;
    TIM_OCInitStruct.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    TIM_OC1Init(TIM16, &TIM_OCInitStruct);
    TIM_OC1Init(TIM17, &TIM_OCInitStruct);

    TIM_CtrlPWMOutputs(TIM16, ENABLE);
    TIM_CtrlPWMOutputs(TIM17, ENABLE);
    TIM_Cmd(TIM16, ENABLE);
    TIM_Cmd(TIM17, ENABLE);

    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;

    // STEP pins
    GPIO_SetBits(GPIOA, GPIO_Pin_11);
    GPIO_SetBits(GPIOB, GPIO_Pin_5 | GPIO_Pin_12);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_12;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    // DIR pins
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_13;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
    GPIO_Init(GPIOF, &GPIO_InitStruct);

    // Microstep pins
    // 1/32 microstepping
    GPIO_SetBits(GPIOA, GPIO_Pin_8);
    GPIO_SetBits(GPIOB, GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_6 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_14 | GPIO_Pin_15);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_6 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Reset pin
    GPIO_SetBits(GPIOA, GPIO_Pin_15);
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // LED
    GPIO_SetBits(GPIOA, GPIO_Pin_0);
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // SysTick runs control loop
    SysTick->LOAD = CL_PERIOD - 1;
    NVIC_SetPriority(SysTick_IRQn, 0);
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;

    // Fault
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;
    GPIO_Init(GPIOF, &GPIO_InitStruct);

    // Handle buttons
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void hal_changeX(int32_t deltaX)
{
    xp += deltaX >> 11;
}

void hal_changeY(int32_t deltaY)
{
    yp += deltaY >> 11;
}

void hal_changeZ(int32_t deltaZ)
{
    zp += deltaZ >> 11;
}

void hal_setXYCurrent(uint16_t c)
{
    uint16_t pwm = ((uint32_t)CUR_PWM * c) >> 16;
    TIM17->CCR1 = pwm;
}

void hal_setZCurrent(uint16_t c)
{
    uint16_t pwm = ((uint32_t)CUR_PWM * c) >> 16;
    TIM16->CCR1 = pwm;
}

uint8_t hal_leftButton()
{
    return !(GPIO_ReadInputData(GPIOC) & GPIO_Pin_13);
}

uint8_t hal_rightButton()
{
    return !(GPIO_ReadInputData(GPIOC) & GPIO_Pin_14);
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
    TIM14->CNT = 0;
    TIM_ClearITPendingBit(TIM14, TIM_IT_Update);
    TIM_ITConfig(TIM14, TIM_IT_Update, ENABLE);
}

void gsNode_hal_cancelTimer()
{
    TIM_ITConfig(TIM14, TIM_IT_Update, DISABLE);
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
    TIM_ITConfig(TIM14, TIM_IT_Update, DISABLE);
    gsNode_hal_timerFired();
}

void SysTick_Handler()
{
    motor_update(&motor_x);
    motor_update(&motor_y);
    motor_update(&motor_z);
}

void TIM1_BRK_UP_TRG_COM_IRQHandler()
{
    int32_t x = motor_x.p >> 11;
    int32_t y = motor_y.p >> 11;
    int32_t z = motor_z.p >> 11;

    if(x < xp)
    {
        GPIOB->BSRR = GPIO_Pin_7<<16; // Set direction
        GPIOB->BSRR = GPIO_Pin_5; // Set step high
        xp--;
    }
    else if(x > xp)
    {
        GPIOB->BSRR = GPIO_Pin_7; // Set direction
        GPIOB->BSRR = GPIO_Pin_5; // Set step high
        xp++;
    }

    if(y < yp)
    {
        GPIOB->BSRR = GPIO_Pin_13<<16; // Set direction
        GPIOB->BSRR = GPIO_Pin_12; // Set step high
        yp--;
    }
    else if(y > yp)
    {
        GPIOB->BSRR = GPIO_Pin_13; // Set direction
        GPIOB->BSRR = GPIO_Pin_12; // Set step high
        yp++;
    }

    if(z < zp)
    {
        GPIOF->BSRR = GPIO_Pin_6<<16; // Set direction
        GPIOA->BSRR = GPIO_Pin_11; // Set step high
        zp--;
    }
    else if(z > zp)
    {
        GPIOF->BSRR = GPIO_Pin_6; // Set direction
        GPIOA->BSRR = GPIO_Pin_11; // Set step high
        zp++;
    }

    // Wait 1.9 us
    // TODO make Banks fix this
    static volatile uint8_t i;
    for(i=0; i<50; i++);

    // Set all steps low
    GPIOA->BSRR = GPIO_Pin_11<<16;
    GPIOB->BSRR = (GPIO_Pin_5 | GPIO_Pin_12)<<16;

    TIM1->SR = ~TIM_FLAG_Update;
}


