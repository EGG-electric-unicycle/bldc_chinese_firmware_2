/ * Encoder config * /


/ *
Main () calls the following 2 function, the encoder related configuration
TIM4_Configuration (); //
// extINT_init ();
* /

/ * Includes ----------------------------------------------- ------------------- * /
#include "stm32f10x.h"

/ * Private function prototypes --------------------------------------------- - * /

U8 prnt_opt = 0x00;

Void test_2DataOutput (s16, s16);
Void test_1DataOutput (s16);



// Tim4 encoder mode
Void Tim4_RCC_Configuration ()
{
// Step1. Turn on the TIM and the corresponding port clock
// Start GPIO
RCC_APB2PeriphClockCmd (RCC_APB2Periph_GPIOD, ENABLE);

RCC_APB1PeriphClockCmd (RCC_APB1Periph_TIM4, ENABLE);

// start AFIO
RCC_APB2PeriphClockCmd (RCC_APB2Periph_AFIO, ENABLE);
GPIO_PinRemapConfig (GPIO_Remap_TIM4, ENABLE);
}


Void Tim4_GPIO_Configuration ()
{// Configure TIM4, ch1, ch2 GPIO to make the corresponding settings for AF output
GPIO_InitTypeDef GPIO_InitStructure;
//PA.7 / port is set to TIM8_CH1N
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
GPIO_InitStructure.GPIO_Speed ​​= GPIO_Speed_50MHz;
GPIO_Init (GPIOD, & GPIO_InitStructure);

// CPU_LED == PD4
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4; // indicates that the update event is interrupted
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; // push-pull output mode
GPIO_InitStructure.GPIO_Speed ​​= GPIO_Speed_50MHz;
GPIO_Init (GPIOD, & GPIO_InitStructure);

}

Void TIM_4_Configuration ()
{
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
TIM_ICInitTypeDef TIM_ICInitStructure;
TIM_ICStructInit (& TIM_ICInitStructure);
// TIM_ICStructInit (& TIM_ICInitStructure);
//TIM_ICInitStructure.TIM_ICMode = TIM_ICMode_PWMI;
TIM_ICInitStructure.TIM_Channel = TIM_Channel_1 | TIM_Channel_2;
TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
TIM_ICInitStructure.TIM_ICFilter = 0x0;
TIM_ICInit (TIM4, & TIM_ICInitStructure);
The same time as the above-
The same time as the above-
The same time as the above-
The same time as the above-
The same time as the above-
TIM_TimeBaseStructure.TIM_Prescaler = 0;
The same time as the above-
TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
The same time as the above-
TIM_TimeBaseStructure.TIM_ClockDivision = 0;
The same time as the above-
TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
The same time as the above-
TIM_TimeBaseInit (TIM4, & TIM_TimeBaseStructure); // Time base configuration
The same time as the above-
The same time as the above-
The same time as the above-
The same time as the above-
The same time as the above-
// TIM_ETRConfig (TIM3, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);
// TIM_SelectInputTrigger (TIM4, TIM_TS_TI2FP2);
The same time as the above-
// TIM_SelectSlaveMode (TIM4, TIM_SlaveMode_External1);
// TIM_ETRClockMode1Config (TIM4, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_Inverted, 0x0);
TIM_EncoderInterfaceConfig (TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
TIM_SetCounter (TIM4, 0);
The same time as the above-
The same time as the above-
TIM_Cmd (TIM4, ENABLE);
}

/ * Call 1 * /
Void TIM__Configuration (void)
{
  Tim4_RCC_Configuration ();
Tim4_GPIO_Configuration ();
TIM_4_Configuration ();
}


// Tim4 4 multiplier mode
/ *
Void Tim4_RCC_Configuration ()
{

RCC_APB2PeriphClockCmd (RCC_APB2Periph_GPIOD, ENABLE);

RCC_APB1PeriphClockCmd (RCC_APB1Periph_TIM4, ENABLE);

// start AFIO
RCC_APB2PeriphClockCmd (RCC_APB2Periph_AFIO, ENABLE);
GPIO_PinRemapConfig (GPIO_Remap_TIM4, ENABLE);
}


Void Tim4_GPIO_Configuration ()
{// Configure TIM1, TIM8.GPIO to make the corresponding settings for AF output
GPIO_InitTypeDef GPIO_InitStructure;
//PA.7 / port is set to TIM8_CH1N
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
GPIO_InitStructure.GPIO_Speed ​​= GPIO_Speed_50MHz;
GPIO_Init (GPIOD, & GPIO_InitStructure);

}

// Step3. TIM1, TIM8 module initialization
Void TIM_4_Configuration ()
{
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
TIM_ICInitTypeDef TIM_ICInitStructure;
TIM_ICStructInit (& TIM_ICInitStructure);
// TIM_ICStructInit (& TIM_ICInitStructure);
//TIM_ICInitStructure.TIM_ICMode = TIM_ICMode_PWMI;
TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
TIM_ICInitStructure.TIM_ICFilter = 0x0;
TIM_ICInit (TIM4, & TIM_ICInitStructure);



TIM_TimeBaseStructure.TIM_Prescaler = 0;

TIM_TimeBaseStructure.TIM_Period = 0XFFFF;

TIM_TimeBaseStructure.TIM_ClockDivision = 0;

TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

TIM_TimeBaseInit (TIM4, & TIM_TimeBaseStructure); // Time base configuration




 
// TIM_ETRConfig (TIM3, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);
TIM_SelectInputTrigger (TIM4, TIM_TS_TI2FP2);
TIM_SelectSlaveMode (TIM4, TIM_SlaveMode_External1);
// TIM_ETRClockMode1Config (TIM4, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_Inverted, 0x0);
TIM_SetCounter (TIM4, 0);
    

TIM_Cmd (TIM4, ENABLE);


}

Void TIM__Configuration ()
{
  Tim4_RCC_Configuration ();
Tim4_GPIO_Configuration ();
TIM_4_Configuration ();
}
* /


/ * ExternalINT * /

/ ************************************************* *********************
* Name: RCC_Configuration ()
* Function: Configure the clock
* Entry parameters:
* Export parameters:
-------------------------------------------------- ---------------------
* Description: Use library functions
************************************************** ********************* /
Void extINT_RCC_Configuration (void)
{

  RCC_APB2PeriphCloseCmd (RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
  RCC_APB2Periph_GPIOF | RCC_APB2Periph_GPIOG | RCC_APB2Periph_AFIO, ENABLE); // Provide clocks to GPIOA and remapping, Note: Be sure to set RCC_APB2Periph_AFIO

}


/ ************************************************* *********************
* Name: GPIO_Configuration ()
* Function: Configure input and output
* Entry parameters:
* Export parameters:
-------------------------------------------------- ---------------------
* Description: use the library function, configure the IO port
************************************************** ********************* /
 Void extINT_GPIO_Configuration (void)
 {

  GPIO_InitTypeDef GPIO_InitStructure;
  / * Set D10 to input * /

/////// direction control positive and negative, INDEX
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; // turn count 2 octave pin
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
GPIO_Init (GPIOG, & GPIO_InitStructure);

GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; // Turn the INDEX signal
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
GPIO_Init (GPIOC, & GPIO_InitStructure);

GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11; // Steering the direction signal
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
GPIO_Init (GPIOF, & GPIO_InitStructure);

GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; // | GPIO_Pin_12; // left wheel encoder positive and negative, INDEX
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
GPIO_Init (GPIOD, & GPIO_InitStructure);


GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11; // right wheel encoder positive and negative, INDEX
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
GPIO_Init (GPIOB, & GPIO_InitStructure);

GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
GPIO_Init (GPIOB, & GPIO_InitStructure);




 }

/ ************************************************* *********************
* Name: EXTI_Configuration ()
* Features:
* Entry parameters:
* Export parameters:
-------------------------------------------------- ---------------------
* Description:
PD8 - left index - connected to external interrupt channel 8
PB10 -
PB11 -
************************************************** ********************* /
Void extINT_EXTI_Configuration (void)
{
  EXTI_InitTypeDef EXTI_InitStructure;


  / * Configure EXTI Line3 to generate an interrupt on falling edge * /
// EXTI_InitStructure.EXTI_Line = EXTI_Line2; // External interrupt channel 2
// EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
// EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; // rising edge edge trigger
// EXTI_InitStructure.EXTI_LineCmd = ENABLE;
// EXTI_Init (& EXTI_InitStructure);
// GPIO_EXTILineConfig (GPIO_PortSourceGPIOG, GPIO_PinSource2); // Connect PG2 to external interrupt channel 2
// EXTI_ClearITPendingBit (EXTI_Line2);
//
// EXTI_InitStructure.EXTI_Line = EXTI_Line3; // External interrupt channel 3
// EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
// EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // falling edge trigger
// EXTI_InitStructure.EXTI_LineCmd = ENABLE;
// EXTI_Init (& EXTI_InitStructure);
// GPIO_EXTILineConfig (GPIO_PortSourceGPIOC, GPIO_PinSource3); // Connect PC3 to external interrupt channel 3
// EXTI_ClearITPendingBit (EXTI_Line3);


  EXTI_InitStructure.EXTI_Line = EXTI_Line8; // External interrupt channel 8
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; // falling edge trigger
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init (& EXTI_InitStructure);
  GPIO_EXTILineConfig (GPIO_PortSourceGPIOD, GPIO_PinSource8); // Connect PD8 to external interrupt channel 8
  EXTI_ClearITPendingBit (EXTI_Line8);


// EXTI_InitStructure.EXTI_Line = EXTI_Line12; // External interrupt channel 12
// EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
// EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; // rising edge edge trigger
// EXTI_InitStructure.EXTI_LineCmd = ENABLE;
// EXTI_Init (& EXTI_InitStructure);
// GPIO_EXTILineConfig (GPIO_PortSourceGPIOD, GPIO_PinSource12); // connect PD12 to external interrupt channel 12
// EXTI_ClearITPendingBit (EXTI_Line12);


  EXTI_InitStructure.EXTI_Line = EXTI_Line11; // External interrupt channel 11
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; // rising falling edge trigger
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init (& EXTI_InitStructure);
  GPIO_EXTILineConfig (GPIO_PortSourceGPIOB, GPIO_PinSource11); // Connect PB11 to external interrupt channel 11
  EXTI_ClearITPendingBit (EXTI_Line11);

  EXTI_InitStructure.EXTI_Line = EXTI_Line10; // External interrupt channel 10
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; // falling edge trigger
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init (& EXTI_InitStructure);
  GPIO_EXTILineConfig (GPIO_PortSourceGPIOB, GPIO_PinSource10); // Connect PB10 to external interrupt channel 10
  EXTI_ClearITPendingBit (EXTI_Line10);



}

/ ************************************************* *********************
* Name: NVIC_Configuration ()
* Features:
* Entry parameters:
* Export parameters:
-------------------------------------------------- ---------------------
* Description: Configures the encoder index input interrupt
************************************************** ********************* /
 Void extINT_NVIC_Configuration (void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

// NVIC_PriorityGroupConfig (NVIC_PriorityGroup_0); // Preemptive priority is set to no preemption priority
// NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn; // specify the interrupt source
// NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; // specify the response priority level 2
// NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // Enable external interrupt channel 2
// NVIC_Init (& NVIC_InitStructure);
//
// NVIC_PriorityGroupConfig (NVIC_PriorityGroup_0); // Preemptive priority is set to no preemption priority
// NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn; // specify the interrupt source
// NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; // Specifies the response priority level 1
// NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // Enable external interrupt channel 3
// NVIC_Init (& NVIC_InitStructure);

// NVIC_PriorityGroupConfig (NVIC_PriorityGroup_0); // Preemptive priority is set to no preemption priority
// NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn; // specify the interrupt source
// NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; // specify the response priority level 2
// NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // Enable external interrupt channel 15_10
// NVIC_Init (& NVIC_InitStructure);
//
// NVIC_PriorityGroupConfig (NVIC_PriorityGroup_0); // Preemptive priority is set to no preemption priority
// NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn; // specify the interrupt source
// NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; // Specifies the response priority level 1
// NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // Enable external interrupt channel 9_5
// NVIC_Init (& NVIC_InitStructure);

// NVIC_InitTypeDef NVIC_InitStructure;
NVIC_PriorityGroupConfig (NVIC_PriorityGroup_1);
NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn; // update event
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // Preference priority 0 level
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; // from priority level 1
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // IRQ channel enabled
NVIC_Init (& NVIC_InitStructure);

// NVIC_PriorityGroupConfig (NVIC_PriorityGroup_1);
NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn; // update event
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // Preference priority 0 level
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; // from priority level 1
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // IRQ channel enabled
NVIC_Init (& NVIC_InitStructure);


}
/// *********************************************** ***********************
// * name: extINT_init (void)
// * Function: External interrupt initialization call
// * Entry parameters:
// * Export parameters:
// ------------------------------------------------ -----------------------
// * Description:
// ************************************************ *********************** /
Void extINT_init (void)
{
ExtINT_RCC_Configuration (); // configure the clock
ExtINT_GPIO_Configuration (); // Configure IO port
ExtINT_EXTI_Configuration (); // external interrupt configuration
ExtINT_NVIC_Configuration (); // interrupt configuration
}




// left motor INDEX interrupt handler

Void EXTI9_5_IRQHandler (void) // left motor INDEX
{
U8 opt;
Opt = prnt_opt;
If (EXTI_GetITStatus (EXTI_Line8)! = RESET)
    {
If (opt == 0x05)
Test_1DataOutput (TIM4-> CNT);
TIM_GenerateEvent (TIM4, TIM_EventSource_Update);
EXTI_ClearFlag (EXTI_Line8);
EXTI_ClearITPendingBit (EXTI_Line8);
    }
}

Void EXTI15_10_IRQHandler (void)
 {
If (EXTI_GetITStatus (EXTI_Line11)! = RESET) // right motor direction signal
    {
     // Add an interrupt handler
     EXTI_ClearFlag (EXTI_Line11);
     EXTI_ClearITPendingBit (EXTI_Line11);
     }

If (EXTI_GetITStatus (EXTI_Line10)! = RESET) // right motor INDEX
    {
     // Add an interrupt handler
     EXTI_ClearFlag (EXTI_Line10); // clear the interrupt flag (required)
     EXTI_ClearITPendingBit (EXTI_Line10);
     }

 }






