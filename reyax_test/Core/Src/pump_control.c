#include "pump_control.h"

  /*Configure GPIO pin : Pump_Control_Pin */
//  GPIO_InitStruct.Pin = Pump_Control_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(Pump_Control_GPIO_Port, &GPIO_InitStruct);

//#define Pump_LED_Pin GPIO_PIN_5
//#define Pump_LED_GPIO_Port GPIOA

#define Pump_Control_Pin GPIO_PIN_?
#define Pump_Control_GPIO_Port GPIO?

static uint8_t pump_state = 0;

void Pump_Init(void)
{
    Pump_Off();
}

void Pump_Toggle(void)
{
    if (pump_state)
        Pump_Off();
    else
        Pump_On();
}

void Pump_On(void)
{
    HAL_GPIO_WritePin(Pump_Control_GPIO_Port, Pump_Control_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(Pump_LED_GPIO_Port, Pump_LED_Pin, GPIO_PIN_SET);
    pump_state = 1;
}

void Pump_Off(void)
{
    HAL_GPIO_WritePin(Pump_Control_GPIO_Port, Pump_Control_Pin, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(Pump_LED_GPIO_Port, Pump_LED_Pin, GPIO_PIN_RESET);
    pump_state = 0;
}
