#include "stm32f0xx.h"
#include "gsnode_hal.h"
#include "motors.h"

#define WATCHDOG_TIMEOUT (48 * GSNODE_TIMER_US)

#define TICK_PERIOD 375 // 128 KHz
#define CUR_PWM 1500 // 32 KHz
#define CUR_PWM_PULSE 750 // 32 KHz

static int16_t xp;
static int16_t yp;
static int16_t zp;

void hal_init()
{
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    I2C_InitTypeDef I2C_InitStruct;
    TIM_OCInitTypeDef TIM_OCInitStruct;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB |
                          RCC_AHBPeriph_GPIOF, ENABLE);

    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;

    // GPIO: USART
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    // AF: USART
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_0);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_0);

    // USART
    USART_DeInit(USART1);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    USART_InitStruct.USART_BaudRate = 115200;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStruct);
    USART_Cmd(USART1, ENABLE);

    USART_ClearITPendingBit(USART1, USART_IT_TC);

    NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

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
    NVIC_InitStruct.NVIC_IRQChannelPriority = 2;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    // I2C
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Init(GPIOF, &GPIO_InitStruct);

    GPIO_PinAFConfig(GPIOF, GPIO_PinSource0, GPIO_AF_1);
    GPIO_PinAFConfig(GPIOF, GPIO_PinSource1, GPIO_AF_1);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    //RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
    //RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);

    I2C_InitStruct.I2C_Timing = 0x50330309; // How's that for magic constants
    I2C_InitStruct.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
    I2C_InitStruct.I2C_DigitalFilter = 0;
    I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStruct.I2C_OwnAddress1 = 0;
    I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &I2C_InitStruct);

    I2C_Cmd(I2C1, ENABLE);

    // Zero internal motor positions
    xp = 0;
    yp = 0;
    zp = 0;

    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;

    // STEP pins
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_13;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    // DIR pins
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_14;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Current pins
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_15;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_2);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM17, ENABLE);

    TIM_TimeBaseInitStruct.TIM_Prescaler = 0;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStruct.TIM_Period = CUR_PWM - 1;
    TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
    TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM17, &TIM_TimeBaseInitStruct);

    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStruct.TIM_Pulse = CUR_PWM_PULSE - 1;
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStruct.TIM_OCNPolarity = TIM_OCNPolarity_Low;
    TIM_OCInitStruct.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    TIM_OC1Init(TIM17, &TIM_OCInitStruct);

    TIM_CtrlPWMOutputs(TIM17, ENABLE);
    TIM_Cmd(TIM17, ENABLE);

    // TODO setup TIM1.3N for PB15 (Z current)

    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;

    // Microstep pins
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;
    GPIO_Init(GPIOF, &GPIO_InitStruct);

    // TEMP
    GPIO_SetBits(GPIOA, GPIO_Pin_15 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10);
    GPIO_SetBits(GPIOB, GPIO_Pin_3 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12);
    GPIO_SetBits(GPIOF, GPIO_Pin_7);

    // Reset pin
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    // TEMP
    GPIO_SetBits(GPIOB, GPIO_Pin_8);

    // LED
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    GPIO_SetBits(GPIOA, GPIO_Pin_0);

    // SysTick sees if steps need to be sent
    SysTick->LOAD = TICK_PERIOD - 1;
    NVIC_SetPriority(SysTick_IRQn, 1);
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;

    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
    GPIO_Init(GPIOF, &GPIO_InitStruct);
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
    int16_t x = motor_x.p >> 16;
    int16_t y = motor_y.p >> 16;
    int16_t z = motor_z.p >> 16;

    if(x < xp)
    {
        GPIOB->BSRR = GPIO_Pin_5<<16; // Set direction
        GPIOB->BSRR = GPIO_Pin_4;     // Send pulse
        xp--;
    }
    else if(x > xp)
    {
        GPIOB->BSRR = GPIO_Pin_5; // Set direction
        GPIOB->BSRR = GPIO_Pin_4; // Send pulse
        xp++;
    }

    if(y < yp)
    {
        GPIOA->BSRR = GPIO_Pin_12<<16; // Set direction
        GPIOA->BSRR = GPIO_Pin_11;     // Send pulse
        yp--;
    }
    else if(y > yp)
    {
        GPIOA->BSRR = GPIO_Pin_12; // Set direction
        GPIOA->BSRR = GPIO_Pin_11; // Send pulse
        yp++;
    }

    if(z < zp)
    {
        GPIOB->BSRR = GPIO_Pin_14<<16; // Set direction
        GPIOB->BSRR = GPIO_Pin_13;     // Send pulse
        zp--;
    }
    else if(z > zp)
    {
        GPIOB->BSRR = GPIO_Pin_14; // Set direction
        GPIOB->BSRR = GPIO_Pin_13; // Send pulse
        zp++;
    }

    // Wait 1.9 us
    // TODO make Banks fix this
    for(uint8_t i=0; i<9; i++)
    {
        __NOP;
        __NOP;
        __NOP;
        __NOP;
        __NOP;
        __NOP;
        __NOP;
        __NOP;
        __NOP;
    }

    // Clear pulses
    GPIOA->BSRR = GPIO_Pin_11<<16;
    GPIOB->BSRR = (GPIO_Pin_4 | GPIO_Pin_13)<<16;
}


