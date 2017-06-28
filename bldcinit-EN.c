/*bldcinit.c*/
// xx0410
/ ************************************************* ************************************************** ******
*
* File: bldcinit.c
* Hardware Environment: kech ver 1.0
* Build Environment: RealView MDK-ARM Version: 4.22
* Version: V1.0
* By: lihewen
*
* (C) Copyright 2012-, lihewen
* All Rights Reserved
*
************************************************** ************************************************** ***** /
/ ************************************************* ************************************************** *****
STM32_TIM_ADC_Configuration ()
Description: TIM1, TIM8, ADC1, ADC2, ADC3 initialization function
Purpose: Generates the drive motor PWM, which measures the input voltage of the specified port
Conditions of use: TIM, ADC initialization call;
TIM1 pin remapping, TIM1_CH1, TIM1_CH1N, TIM1_CH2, TIM1_CH2N, TIM1_CH3, TIM1_CH3N Complementary output
    TIM8_CH1, TIM8_CH1N, TIM8_CH2, TIM8_CH2N, TIM8_CH3, TIM8_CH3N Complementary outputs
TIM1 - GPIOE (Right Notor): PE8: PE9 = UL: UH, PE10: PE11 = VL: VH, PE12: PE13 = WL: WH;
TIM8 - GPIOx (Lift Notor): UL: VL: WL = PA7: PB0: PB1; UH: VH: WH = PC6: PC7: PC8;
Dead time set to zero, can be set by TIM_BDTRInitStructure.TIM_DeadTime = 0x00
ADC1, ADC2 work in synchronization rules and synchronous injection mode, rule group channel conversion initiated by software trigger, injection group channel conversion initiated by TIM1_TRGO Update event
(1) ADC1 Sampling 1 group of regular channels ADC_Channel_14
(2) ADC2 samples a set of regular channels ADC_Channel_15
(3) rule group conversion initiated by the software trigger, synchronous start ADC1, ADC2
ADC1 injection channel sampling ADC123_IN0, ADC2 injection channel sampling ADC123_IN3 left motor U, V phase current sampling at the same time, channel switching In the TIM1 update event interrupt,
               Interrupt event trigger injection group sampling start, wait for sampling is complete, switch sampling channel
ADC3 is operating independently and the software triggers the start. ADC3 once triggers conversion of 5 channels ADC3, ADC_Channel_5, ADC3, ADC_Channel_7,
         ADC3, ADC_Channel_10, ADC3, ADC_Channel_11, ADC3, ADC_Channel_13
ADC1, ADC3 initiated DMA transfer, the converted data from the ADC_DR register to the user-specified destination address. ADC1, ADC2 share DMA
            ADC2 regular channel data for ADC_DR is 16 bits high and 16 bits low for ADC1 data
ADC1, ADC2 Rule Group Channel Conversion Data In address 0X20000100 SRAM, the upper 16 bits are ADC2 data, the lower 16 bits are ADC1 data
ADC1, ADC2 injection group channel conversion data in ADC1-> JDR1, ADC2-> JDR1, respectively. Left motor, right motor data time division multiplexed ADC1-> JDR1, ADC2-> JDR1
ADC3 rule group channel conversion data at the start address of 0X20000130 SRAM data is 32 bits, the lower 12 bits are valid for a total of 5 groups of data (5 * 4, each data 4 bytes 32 bits)

Description:
ADC1, ADC2 injection group channel conversion, injection group channel conversion triggered by the TIM1_TRGO Update event ADC1, ADC2 simultaneously start a PWM cycle to start once.
    The first cycle sampling the left motor, the second cycle sampling the right motor, the third and sampling the left motor, so back
ADC1 LEFT_U_I_ADC ADC123_IN0 PA0
ADC2 LEFT_V_I_ADC ADC123_IN0 PA3
ADC1 RIGHT_U_I_ADC ADC123_IN0 PA1
ADC2 RIGHT_V_I_ADC ADC123_IN0 PA2
ADC1, ADC2 rule group channel conversion start software control, one time to start simultaneous conversion ADC1, ADC2 call function ADC_SoftwareStartConvCmd (ADC1, ENABLE); // start ADC1 ADC2 rule group conversion
ADC1 right bus current monitoring ADC12_IN14 PC4
ADC2 left bus current monitoring ADC12_IN15 PC5
ADC3 rule group channel conversion start software control, one start conversion 5 group call function ADC_SoftwareStartConvCmd (ADC3, ENABLE); // start ADC3 rule group conversion
Left motor temperature measurement ADC3_IN5 PF7
Right motor temperature measurement ADC3_IN7 PF9
3.3V power supply voltage monitoring ADC123_IN10 PC0
Bus voltage monitoring ADC123_IN11 PC1
Direction ADC ADC123_IN13 PC3
************************************************** ***************************** /
#include "stm32f10x.h" // This header file includes the definition of all peripheral registers, bits, and memory maps of the STM32F10x
#include "MC_Globals.h"
// u8 a = 0;
// ADC1-2, DMA cache ADC12_Buffer [15: 0] = ADC1, ADC12_Buffer [31:16] = ADC2, ADC2,
//
U32 ADC3_Buffer [5]; // __attribute __ ((at (0X2000f130))) = {0};
// ADC12_Buffer [15: 0] = ADC1, --PC4 right bus current
// ADC12_Buffer [31:16] = ADC2, - PC5 left bus current
// u32 * pADC12_Buffer = ADC12_Buffer; // = (u16) & ADC12_Buffer;
// u32 x, y; // x = L / R U phase Ia. Y = L / R V phase Ib.
U16 TimUpdateCount;
U8 Gyro_Data_ready;
/ * # Define Deadtime Value * / // (u16) ((unsigned long long) CKTIM / 2 * (unsigned long long) DEADTIME_NS / 1000000000uL)
// # define DEADTIME ((u16) 30) // define deadtime = 800ns //DTG[7:0]x27.778 ns. @ TIM_CKD_DIV2, (tDTS = 2 × tCK_INT); DTG [7: 5] = 0xx = > DT = DTG [7: 0] x tdtg with tdtg = tDTS.
         // DEADTIME * (1 / (72 (MHz) / 2)) = DEADTIME_NS => DEADTIME = (DEADTIME_NS (ns) // 1000) * (72 (MHz) / 2)
// # define DEADTIME ((u16) 36) // define deadtime = 1000ns
// # define DEADTIME ((u16) 43) // define deadtime = 1200ns
// definedeftime = 1500ns // When you start debugging, choose larger
// extern void SVPWM_CalcDutyCycles (void); // xx0410
// extern void uart_RxTx (void);
// extern void uart_Tx (void);
// extern void uart_Rx (void);
/ * Configuration of GPIO pins * /
// Select ADC channel 0 1 2 3 0 1 3 4 5 7 9, corresponding pin to PA0 PA1 PA2 PA3 PC0 PC1 PC3 PC4 PC5 PF7 PF9
// (PA0: PA1: PA2: PA3 = LEFT_U_I_ADC: RIGHT_U_I_ADC: RIGHT_V_I_ADC: LEFT_V_I_ADC) lihewen 2012-09-17
// ADC1 injection: left / right motor Ia detection (L: R = PA0: PA1 = CH0: CH1) // lihewen 2012-09-20
// ADC2 injection: left / right motor Ib detection (L: R = PA3: PA2 = CH3: CH2)
// ADC1 right bus current monitoring ADC12_IN14 PC4
// ADC2 left bus current monitoring ADC12_IN15 PC5
// left motor temperature measurement ADC3_IN5 PF7
// right motor temperature measurement ADC3_IN7 PF9
//3.3V power supply voltage monitoring ADC123_IN10 PC0
// bus voltage monitoring ADC123_IN11 PC1
// direction ADC ADC123_IN13 PC3
// void Tim_ADC_GPIO_Configuration (void) // xx0410
// {
// GPIO_InitTypeDef GPIO_InitStructure;
//// ADC_GPIO_Configuration
// // PA0 / 1/2/3 as the analog channel input pin
// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; // analog input pin
// GPIO_Init (GPIOA, & GPIO_InitStructure);
// // PC0 / 1/3/4/5 as the analog channel input pin
// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; // analog input pin
// GPIO_Init (GPIOC, & GPIO_InitStructure);
// // joystick: enc turn
//// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_6; // Turn to 2PULSE pin TURN_ENCODER_2PULSE
//// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
//// GPIO_Init (GPIOG, & GPIO_InitStructure);
////
//// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; // turn INDEX signal TURN_ENCODER_INDEX
//// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
//// GPIO_Init (GPIOC, & GPIO_InitStructure);
////
//// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; // Turn to DIRECT signal TURN_ENCODER_DIRECT
//// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
//// GPIO_Init (GPIOD, & GPIO_InitStructure);
//
// // PF / 7/9 as the analog channel input pin
// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_9;
// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; // analog input pin
// GPIO_Init (GPIOF, & GPIO_InitStructure);
// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_10; // right motor hard overcurrent
// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // pull-up the input pin
// GPIO_Init (GPIOF, & GPIO_InitStructure);
// // GPIOF, GPIO_Pin_10 - right foot switch
// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; // right footswitch
// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD // // pull-up the input pin
// GPIO_Init (GPIOF, & GPIO_InitStructure);
//
//// TIM_GPIO_Configuration
// // configure TIM1, TIM8.GPIO to make the corresponding settings for AF output
// // (TIM8 = Lift Motor PWM; UL: VL: WL = PA7: PB0: PB1; UH: VH: WH = PC6: PC7: PC8)
// //PA.7 / port is set to TIM8_CH1N
// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // push-pull multiplexed output mode
// GPIO_InitStructure.GPIO_Speed ​​= GPIO_Speed_50MHz;
// GPIO_Init (GPIOA, & GPIO_InitStructure);
//
// //PB.0/PB.1 port is set to TIM8_CH2N and TIM8_CH3N output
// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // push-pull multiplexed output mode
// GPIO_InitStructure.GPIO_Speed ​​= GPIO_Speed_50MHz;
// GPIO_Init (GPIOB, & GPIO_InitStructure);
//
// // PC.6/7/8 port set to TIM8_CH1, TIM8_CH2 and TIM8_CH3 output
// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // push-pull multiplexed output mode
// GPIO_InitStructure.GPIO_Speed ​​= GPIO_Speed_50MHz;
// GPIO_Init (GPIOC, & GPIO_InitStructure);
// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; // left motor hard overcurrent
// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // pull-up input mode
// GPIO_InitStructure.GPIO_Speed ​​= GPIO_Speed_50MHz;
// GPIO_Init (GPIOC, & GPIO_InitStructure);
//
// //PE.8/9/10/11/12/13 port set to TIM1_CH1, TIM1_CH2 and TIM1_CH3 // TIM1_CH1N, TIM1_CH2N and TIM1_CH3N output
// // (TIM1 - GPIOE (Right Notor): PE8: PE9 = UL: UH, PE10: PE11 = VL: VH, PE12: PE13 = WL: WH;
// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 |
// GPIO_Pin_12 | GPIO_Pin_13;
// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // push-pull multiplexed output mode
// GPIO_InitStructure.GPIO_Speed ​​= GPIO_Speed_50MHz;
// GPIO_Init (GPIOE, & GPIO_InitStructure);
// //PE.1=Beep;PE.2=PWM_RIGHT_SD; PE.7 = LED IND is set to output port
// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_2 | GPIO_Pin_1; // PWM_SD; // indicates that the update event is interrupted
// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; // push-pull output mode
// GPIO_InitStructure.GPIO_Speed ​​= GPIO_Speed_50MHz;
// GPIO_Init (GPIOE, & GPIO_InitStructure);
//
// BEEP_OFF;
//}
 

/ * Configure the system clock, enable each peripheral clock * /
// void Tim_ADC_RCC_Configuration (void) // xx0410
// {
//// SystemInit ();
// RCC_PCLK2Config (RCC_HCLK_Div1);
// / * Enable each peripheral clock * /
// RCC_APB2PeriphCloseCmd (RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOE |
// RCC_APB2Periph_GPIOF | RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 | RCC_APB2Periph_ADC3, ENABLE); // Enable ADC1 channel clock, each pin clock
// / * Configure ADCCLK such as ADCCLK = PCLK2 / 6 * /
// RCC_ADCCLKConfig (RCC_PCLK2_Div8); // 72M / 6 = 12, ADC maximum clock can not exceed 14M
//// RCC_AHBPeriphClockCmd (RCC_AHBPeriph_DMA1, ENABLE); // Enable DMA1 clock
// RCC_AHBPeriphClockCmd (RCC_AHBPeriph_DMA2, ENABLE); // Enable DMA2 clock
// // start AFIO
// RCC_APB2PeriphClockCmd (RCC_APB2Periph_AFIO, ENABLE);
// GPIO_PinRemapConfig (GPIO_FullRemap_TIM1, ENABLE); // TIM1 pin remapping
// // start TIM1, TIM8
// RCC_APB2PeriphClockCmd (RCC_APB2Periph_TIM1, ENABLE);
// RCC_APB2PeriphClockCmd (RCC_APB2Periph_TIM8, ENABLE);
//
//}
 
/ * Configure ADC1 * /
Void ADC_Configuration (void)
{
  ADC_InitTypeDef ADC_InitStructure;
 / * ADC1 configuration ---------------------------------------------- -------- * /
 ADC_InitStructure.ADC_Mode = ADC_Mode_RegInjecSimult; // ADC operating mode: ADC1 and ADC2 operate in synchronous and synchronous injection modes
 ADC_InitStructure.ADC_ScanConvMode = ENABLE; // Analog-to-digital conversion works in scan mode
 ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; // Analog-to-digital conversion works in continuous conversion mode
 ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // conversion initiated by software instead of external trigger
 ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; // ADC data right justified
 ADC_InitStructure.ADC_NbrOfChannel = 1; // Number of ADC channels for sequential conversion of rules
 ADC_Init (ADC1, & ADC_InitStructure); // Initialize the register of the ADCx according to the parameters specified in ADC_InitStruct
 // ADC1 regular channel configuration
 / * ADC1 regular channel configuration * /
 ADC_DiscModeChannelCountConfig (ADC1, 1); // break mode rule group channel counter has a value of 1
 // Right bus current detection ADC1, ADC_Channel_14
 ADC_RegularChannelConfig (ADC1, ADC_Channel_14, 1, ADC_SampleTime_1Cycles5); // _ 1Cycles5, _7Cycles5, _13Cycles5, _28Cycles5 ...
 ADC_DiscModeCmd (ADC1, ENABLE); // enable interrupt mode
 ADC_ExternalTrigConvCmd (ADC1, ENABLE); // Enable rule group external trigger
 // ADC1 injection channel configuration
 ADC_InjectedSequencerLengthConfig (ADC1, 1); // Transform group channel conversion sequence length 1
 ADC_InjectedChannelConfig (ADC1, ADC_Channel_1, 1, ADC_SampleTime_1Cycles5);
 ADC_InjectedDiscModeCmd (ADC1, ENABLE); // inject group break mode
 ADC_ExternalTrigInjectedConvConfig (ADC1, ADC_ExternalTrigInjecConv_T1_TRGO); // select the trigger output of Timer 1 as an injection to convert the external trigger
 ADC_ExternalTrigInjectedConvCmd (ADC1, ENABLE); // Enable external trigger
 
/ * ADC2 configuration ---------------------------------------------- -------- * /
 ADC_InitStructure.ADC_Mode = ADC_Mode_RegInjecSimult; // ADC operating mode: ADC1 and ADC2 operate in synchronous and synchronous injection modes
 ADC_InitStructure.ADC_ScanConvMode = ENABLE; // Analog-to-digital conversion works in scan mode
 ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; // Analog-to-digital conversion works in continuous conversion mode
 ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // conversion initiated by software instead of external trigger
 ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; // ADC data right justified
 ADC_InitStructure.ADC_NbrOfChannel = 1; // Number of ADC channels for sequential conversion of rules
 ADC_Init (ADC2, & ADC_InitStructure); // Initialize the register of the ADCx according to the parameters specified in ADC_InitStruct
 // ADC2 regular channel configuration
 ADC_DiscModeChannelCountConfig (ADC2, 1); // break mode rule group channel counter has a value of 1
 // left bus current detection ADC2, ADC_Channel_15
 ADC_RegularChannelConfig (ADC2, ADC_Channel_15, 1, ADC_SampleTime_1Cycles5); // _ 1Cycles5, _7Cycles5, _13Cycles5, _28Cycles5 ...
 ADC_ExternalTrigConvCmd (ADC1, ENABLE);
 ADC_DiscModeCmd (ADC2, ENABLE); // enable interrupt mode
 // ADC2 injection channel configuration
 ADC_InjectedSequencerLengthConfig (ADC2, 1); // Insert group channel conversion sequence length 4
 ADC_InjectedChannelConfig (ADC2, ADC_Channel_2, 1, ADC_SampleTime_1Cycles5); // the same channel is collected four times
 ADC_InjectedDiscModeCmd (ADC2, ENABLE); // inject group break mode
 ADC_ExternalTrigInjectedConvConfig (ADC2, ADC_ExternalTrigInjecConv_None); // injection conversion initiated by software instead of external trigger
 ADC_ExternalTrigInjectedConvCmd (ADC2, ENABLE); // Enable external trigger

 / * ADC3 configuration ---------------------------------------------- -------- * /
 ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; // Standalone mode
 ADC_InitStructure.ADC_ScanConvMode = ENABLE; // Analog-to-digital conversion works in scan mode
 ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; // Analog-to-digital conversion works in continuous conversion mode
 ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // conversion initiated by software instead of external trigger
 ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; // ADC data right justified
 ADC_InitStructure.ADC_NbrOfChannel = 5; // Number of ADC channels in order to convert rules
 ADC_Init (ADC3, & ADC_InitStructure); // initialize the register of the peripheral ADCx according to the parameters specified in ADC_InitStruct
 // ADC3 regular channel configuration
 ADC_DiscModeChannelCountConfig (ADC3, 5); // Intermittent mode rule group The channel counter has a value of 1
 ADC_RegularChannelConfig (ADC3, ADC_Channel_5, 1, ADC_SampleTime_1Cycles5); // _ 1Cycles5, _7Cycles5, _13Cycles5, _28Cycles5 ...
 ADC_RegularChannelConfig (ADC3, ADC_Channel_7, 2, ADC_SampleTime_1Cycles5);
 ADC_RegularChannelConfig (ADC3, ADC_Channel_10, 3, ADC_SampleTime_1Cycles5);
 ADC_RegularChannelConfig (ADC3, ADC_Channel_11, 4, ADC_SampleTime_1Cycles5);
 ADC_RegularChannelConfig (ADC3, ADC_Channel_13, 5, ADC_SampleTime_1Cycles5); // Direction ADC
 ADC_DiscModeCmd (ADC3, ENABLE); // enable interrupt mode
 ADC_ExternalTrigConvCmd (ADC3, ENABLE); // Enable rule group external trigger
 / / Open the ADC DMA support (to achieve DMA function, you need to configure the DMA channel and other parameters)
    ADC_DMACmd (ADC1, ENABLE);
 ADC_DMACmd (ADC3, ENABLE);
 
 / * Enable ADC1 / ADC2 * /
  ADC_Cmd (ADC1, ENABLE); // enable the specified ADC1
  ADC_Cmd (ADC2, ENABLE); // enable the specified ADC2
  ADC_Cmd (ADC3, ENABLE); // enable the specified ADC3
  / * Enable ADC1 reset calibaration register * /
 ADC_ResetCalibration (ADC1); // Reset the specified ADC1 calibration register
 / * Check the end of ADC1 reset calibration register * /
 While (ADC_GetResetCalibrationStatus (ADC1)); // Get the state of ADC1 reset calibration register, wait state
 
 / * Start ADC1 calibaration * /
 ADC_StartCalibration (ADC1); // Starts the calibration status of ADC1
 / * Check the end of ADC1 calibration * /
 While (ADC_GetCalibrationStatus (ADC1)); // Get the calibration procedure for the specified ADC1 and wait for the status
 
 
 / * Enable ADC2 reset calibaration register * /
 ADC_ResetCalibration (ADC2); // Reset the specified ADC2 calibration register
 / * Check the end of ADC2 reset calibration register * /
 While (ADC_GetResetCalibrationStatus (ADC2)); // Get the ADC2 reset register register status, wait state
 
 / * Start ADC2 calibaration * /
 ADC_StartCalibration (ADC2); // starts specifying the calibration status of ADC2
 / * Check the end of ADC2 calibration * /
 While (ADC_GetCalibrationStatus (ADC2)); // Get the calibration procedure for the specified ADC2 and wait for the status

 / * Enable ADC23 reset calibaration register * /
 ADC_ResetCalibration (ADC3); // Reset the specified ADC3 calibration register
 / * Check the end of ADC3 reset calibration register * /
 While (ADC_GetResetCalibrationStatus (ADC3)); // Get the state of the ADC3 reset calibration register, wait state
 
 / * Start ADC3 calibaration * /
 ADC_StartCalibration (ADC3); // starts specifying the calibration status of ADC3
 / * Check the end of ADC3 calibration * /
 While (ADC_GetCalibrationStatus (ADC3)); // Get the calibration procedure for the specified ADC3 and wait for the status
}
 
/ * Configure DMA * /
Void DMA_Configuration (void)
{
 
 / * ADC1 DMA1 Channel Config * /
 DMA_InitTypeDef DMA_InitStructure;
// DMA_DeInit (DMA1_Channel1); // reset the DMA1 channel 1 register to the default value
// DMA_InitStructure.DMA_PeripheralBaseAddr = (u32) & ADC1-> DR; // DMA peripherals ADC base address
// DMA_InitStructure.DMA_MemoryBaseAddr = (u32) ADC12_Buffer; // 0x20000100; // DMA memory base address
// DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; // memory as destination for data transfer
// DMA_InitStructure.DMA_BufferSize = 1; // DMA cache size of DMA cache
// DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; // Peripheral address register unchanged
// DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; // memory address register increment
// DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word; // Data width is 32 bits
// DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word; // Data width is 32 bits
// DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; // work in loop caching mode
// DMA_InitStructure.DMA_Priority = DMA_Priority_High // // DMA channel x has high priority
// DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; // DMA channel x is not set to memory to memory transfer
// DMA_Init (DMA1_Channel1, & DMA_InitStructure); // initialize the DMA channel according to the parameters specified in DMA_InitStruct
 / * ADC3 DMA1 Channel Config * /
 DMA_DeInit (DMA2_Channel5); // reset the DMA2 channel 5 register to the default value
 DMA_InitStructure.DMA_PeripheralBaseAddr = (u32) & ADC3-> DR; // DMA peripheral ADC base address
 DMA_InitStructure.DMA_MemoryBaseAddr = (u32) ADC3_Buffer; // 0x20000130; // DMA memory base address
 DMA_InitStructure.DMA_DIR = DMA_DIR_PERipheralSRC; // memory as destination for data transfer
 DMA_InitStructure.DMA_BufferSize = 5; // DMA cache size of DMA cache
 DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; // peripheral address register unchanged
 DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; // memory address register increment
 DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word; // Data width is 32 bits
 DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word; // Data width is 32 bits
 DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; // work in loop caching mode
 DMA_InitStructure.DMA_Priority = DMA_Priority_High; // DMA channel x has high priority
 DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; // DMA channel x is not set to memory to memory transfer
 DMA_Init (DMA2_Channel5, & DMA_InitStructure); // Initialize the DMA channel according to the parameters specified in DMA_InitStruct
}

// TIM configuration
 
Void TIM_NVIC_Configuration ()
{
 NVIC_InitTypeDef NVIC_InitStructure;
 NVIC_PriorityGroupConfig (NVIC_PriorityGroup_1);
 NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn; // update the event
 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // Preference priority 0 level
 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; // from priority level 1
 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // IRQ channel enabled
 NVIC_Init (& NVIC_InitStructure); // Initialize the peripheral NVIC register according to the parameters specified in NVIC_InitStruct
}
 

// Step3. TIM1, TIM8 module initialization
Void TIM_Configuration ()
{
    TIM_TimeBaseInitTypeDef TIM_BaseInitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
 TIM_DeInit (TIM1); // lihewen 2012-09-21
 TIM_TimeBaseStructInit (& TIM_BaseInitStructure); // lihewen 2012-09-21
    // TIM1,8 time base parameter configuration / * Time Base configuration * /
    // frequency = TIM1_CLK / (ARR + 1)
    TIM_BaseInitStructure.TIM_Period = PWM_PERIOD; // Automatically load half cycle value (2250 * 2/72000000) = 0.000062.5s = 16KHZ
    TIM_BaseInitStructure.TIM_Prescaler = 0x0; // The clock is 72MHZ
    TIM_BaseInitStructure.TIM_ClockDivision = 0x0; //// Dead time division
    TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned3; // TIM Center Alignment Mode 3 Count Mode
    TIM_BaseInitStructure.TIM_RepetitionCounter = 0x1; // Repeated counter value is 0 to generate 2 interrupts
 // TIM1 time base settings
    TIM_TimeBaseInit (TIM1, & TIM_BaseInitStructure); // call the initialization function to initialize TIM1
 TIM_ARRPreloadConfig (TIM1, ENABLE); // Enable the ARR shadow register (until the update event is changed to change the setting)
 // TIM8 time base settings
 TIM_TimeBaseInit (TIM8, & TIM_BaseInitStructure); // call initialization function to initialize TIM8
 TIM_ARRPreloadConfig (TIM8, ENABLE); // Enable the ARR shadow register (until the update event is changed to change the setting)

 TIM_OCStructInit (& TIM_OCInitStructure); // lihewen 2012-09-21
 / * Channel 1, 2,3 in PWM mode * /
    // TIM1_OC module parameter configuration
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; // TIM pulse width modulation mode 1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; // TIM output compare enable
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable; // TIM Complementary Output Compare Enable
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; // TIM Output Compare Polarity High
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High; // TIM Complementary Output Compare Polarity High
    TIM_OCInitStructure.TIM_Pulse = PWM_PERIOD >> 1; // 1125; // The value of the capture comparator to be loaded (this register sets the PWM duty cycle by 25%)
 TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset; // lihewen 2012-09-21
 TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset; // lihewen 2012-09-21
 // TIM1_OC module settings
    TIM_OC1Init (TIM1, & TIM_OCInitStructure);
 TIM_OC2Init (TIM1, & TIM_OCInitStructure);
 TIM_OC3Init (TIM1, & TIM_OCInitStructure);
    
 // TIM8_OC module settings
    TIM_OC1Init (TIM8, & TIM_OCInitStructure);
 TIM_OC2Init (TIM8, & TIM_OCInitStructure);
 TIM_OC3Init (TIM8, & TIM_OCInitStructure);

 / * Enables the Preload on CCx Register * /
    // TIM1_OCx module set to enable the shadow register of the CCRx register (until an update event is generated to change the setting)
    TIM_OC1PreloadConfig (TIM1, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig (TIM1, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig (TIM1, TIM_OCPreload_Enable);
    // TIM8_OCx module set to enable the shadow register of the CCRx register (until an update event is generated to change the setting)
    TIM_OC1PreloadConfig (TIM8, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig (TIM8, TIM_OCPreload_Enable);
 TIM_OC3PreloadConfig (TIM8, TIM_OCPreload_Enable);

    // dead zone settings
    TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable; // Enable TIM1 OSSR state
    TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable; // Enable TIM1 OSSI state
    TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF; // does not lock any bits
    TIM_BDTRInitStructure.TIM_DeadTime = DEADTIME; / / here to adjust the dead zone size 0-0x7f, can be achieved 0- -1777ns or take 0x80-0xc0 can achieve 1777ns - 3555ns
    TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable; // Disability TIM1 Brake Input
    TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High; // TIM1 Brake Input Pin Low Polarity
    TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable; // automatic output function enabled? 2012-09-17
    TIM_BDTRConfig (TIM1, & TIM_BDTRInitStructure);
    TIM_BDTRConfig (TIM8, & TIM_BDTRInitStructure);
 // Configure synchronous start TIM1, TIM8
 / * Select the Master Slave Mode: TIM1 * /
    TIM_SelectMasterSlaveMode (TIM1, TIM_MasterSlaveMode_Enable); // Configure TIM1 for master mode
 TIM_SelectOutputTrigger (TIM1, TIM_TRGOSource_Enable); // Enable selected as trigger output (TRGO)
 TIM_SelectSlaveMode (TIM1, TIM_SlaveMode_Trigger);
 TIM_SelectInputTrigger (TIM1, TIM_TS_TI1F_ED);
 TIM_SelectMasterSlaveMode (TIM1, TIM_MasterSlaveMode_Enable);
 / * Slave Mode selection: TIM8 * /
 TIM_SelectSlaveMode (TIM8, TIM_SlaveMode_Trigger); // Configure TIM8 as slave mode
    TIM_SelectInputTrigger (TIM8, TIM_TS_ITR0); // select TIM internal trigger 0
 // The software generates an update event that causes the preload register to be loaded into the shadow register
 TIM_GenerateEvent (TIM1, TIM_EventSource_Update);
 TIM_GenerateEvent (TIM8, TIM_EventSource_Update);
 // interrupt configuration
 TIM_ClearFlag (TIM1, TIM_FLAG_Update); // clear the interrupt flag
 TIM_ITConfig (TIM1, TIM_IT_Update, ENABLE); // Enable update interrupt
// TIM_ClearFlag (TIM8, TIM_FLAG_Update); // clear the interrupt flag
// TIM_ITConfig (TIM8, TIM_IT_Update, ENABLE); // Enable update interrupt
 TIM1-> CCR1 = 1125;
 TIM1-> CCR2 = 1125;
 TIM1-> CCR3 = 1125;
 TIM8-> CCR1 = 1125;
 TIM8-> CCR2 = 1125;
 TIM8-> CCR3 = 1125;
    TIM_Cmd (TIM1, ENABLE); // TIM1 is on
    TIM_Cmd (TIM8, ENABLE); // TIM8 on
}

Void STM32_TIM_ADC_Configuration (void)
{
// Tim_ADC_RCC_Configuration (); // xx0410
// Tim_ADC_GPIO_Configuration (); // xx0410
 ADC_Configuration ();
 DMA_Configuration ();
// TIM_NVIC_Configuration (); // xx0410
 TIM_Configuration ();
    / * Start ADC1 Software Conversion * /
// DMA_Cmd (DMA1_Channel1, ENABLE); // start the DMA channel
 DMA_Cmd (DMA2_Channel5, ENABLE); // start the DMA channel
 ADC_SoftwareStartConvCmd (ADC1, ENABLE);
 ADC_SoftwareStartConvCmd (ADC2, ENABLE);
 ADC_SoftwareStartConvCmd (ADC3, ENABLE);
// ADC_SoftwareStartInjectedConvCmd (ADC2, ENABLE); // 2012-11-27
 TIM_SelectOutputTrigger (TIM1, TIM_TRGOSource_Update); //? 2012-09-17
}



