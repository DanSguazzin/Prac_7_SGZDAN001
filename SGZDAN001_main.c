//*                    MEC4126F C template                           *
//*==================================================================*
//* WRITTEN BY: Jesse Arendse                                        *
//* MODIFIED: Dylan Fanner, ChatGPT                                  *
//* DATE MODIFIED: 20/05/2025                                        *
//*==================================================================*
//* DESCRIPTION: Motor control with ADC, PWM, PI controller          *
//********************************************************************

// INCLUDE FILES
#define STM32F051

#define USE_ADC
#define USE_TIM6
#define USE_TIM3

#include "stm32f0xx.h"
#include "stdio.h"
#include "stdint.h"

//====================================================================
// GLOBAL CONSTANTS
//====================================================================
const float K_p = 50;         // Reduced proportional gain for finer control
const float K_i = 1;         // Reduced integral gain
const float T_s = 0.001;
const int DEADZONE = 3;       // Deadband threshold

//====================================================================
// GLOBAL VARIABLES
//====================================================================
volatile float prev_output = 0;
volatile float prev_input = 0;

//====================================================================
// FUNCTION DECLARATIONS
//====================================================================
void set_to_48MHz(void);
void init_ADC(void);
void init_timer6(void);
void init_timer3(void);
float PI_control(int32_t error);

//====================================================================
// MAIN FUNCTION
//====================================================================
int main(void)
{
    set_to_48MHz();
    init_ADC();
    init_timer3();  // PWM output
    init_timer6();  // Control loop

    while (1);  // Main loop does nothing; control is interrupt-driven
}

//====================================================================
// PI CONTROLLER FUNCTION
//====================================================================
float PI_control(int32_t error)
{
    float input = error * K_p;
    float output = (input * (2 + K_i * T_s) + prev_input * (K_i * T_s - 2) + 2 * prev_output) / 2;

    // Clamp output to 8-bit signed range
    if (output > 126) output = 126;
    if (output < -127) output = -127;

    prev_input = input;
    prev_output = output;

    return output;
}

//====================================================================
// TIMER6 ISR – 1 kHz control loop
//====================================================================
void TIM6_IRQHandler(void)
{

    
}

//====================================================================
// CLOCK CONFIGURATION – 48 MHz system clock
//====================================================================
void set_to_48MHz(void)
{
    RCC->CFGR |= (0b1010 << 18);    // PLL x12
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));
    RCC->CFGR |= RCC_CFGR_SW_PLL;
}

//====================================================================
// ADC INITIALISATION – PA6 (command), PA5 (feedback)
//====================================================================
void init_ADC(void)
{
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA->MODER |= GPIO_MODER_MODER5 | GPIO_MODER_MODER6;  // Analog mode

    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

    ADC1->CFGR1 |= ADC_CFGR1_RES_1;    // Set to 10-bit resolution
    ADC1->CFGR1 |= ADC_CFGR1_CONT;       // Enable continuous conversion mode
    ADC1->CFGR1 |= ADC_CFGR1_WAIT;

    ADC1->CR |= ADC_CR_ADEN;
    while (!(ADC1->ISR & ADC_ISR_ADRDY));  // Wait for ADC ready
}

//====================================================================
// TIMER3 INITIALISATION – 20 kHz PWM on PB4
//====================================================================
void init_timer3(void)
{


}

//====================================================================
// TIMER6 INITIALISATION – 1 kHz control loop
//====================================================================
void init_timer6(void)
{


}
