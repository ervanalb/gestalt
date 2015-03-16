#include "stm32f0xx.h"
#include "gsnode_hal.h"

#define WATCHDOG_TIMEOUT (48 * GSNODE_TIMER_US)

void hal_init()
{
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;

    // GPIO: USART
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // AF: USART
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);

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
