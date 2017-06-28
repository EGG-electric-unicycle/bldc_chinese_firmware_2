 /*bldc_main.c*/

/ ************************************************* ************************************************** ******
*
* File: main.c
* Hardware Environment: kech ver 1.0
* Build Environment: RealView Mkd-ARM Version: 4.22
* Version: V1.0
* By: lihewen
*
* (C) Copyright 2012-, lihewen
* All Rights Reserved
*
************************************************** ************************************************** ***** /
/ * Includes ----------------------------------------------- ------------------- * /
#include "stm32f10x.h"
#include "MC_Globals.h"
/ ** @addtogroup STM32F10x_StdPeriph_Examples
  * @ {
  * /
/ ** @addtogroup TIM_7PWM_Output
  * @ {
  * /
/ * Private typedef ---------------------------------------------- ------------- * /
/ * Private define ---------------------------------------------- -------------- * /
/ * Private macro ---------------------------------------------- --------------- * /
/ * Private variables ---------------------------------------------- ----------- * /

// # define k1 ((s16) 1)
// # define k2 ((s16) 1)
#define Gyro_Data_ready_Error ((u16) 0x0001)
U16 ErrorInfo = 0;
Float Balcommand = 0, Yawcommand = 0;
S16 LMotorCmd = 0, RMotorCmd = 0;
Float pitchcmd
S16 test_tmp;
U8 GyroAcce_state = 0;
U8 Anglek_FLAG = 0;
U32 tCount = 0;
U8 outputEN = 0;
U8 FootSwitchStateCount = 0;
U8 FootSwitchStateChange = 0;
U16 whileCount = 0;
Static float softFactor = 0;
// static float KIfactor = 0.015;
U16 Vbus = 0;
U16 Vdd33 = 0;

U8 MaxAnglekProtect = 1;
// u16 RIbus = 0;
// u16 LIbus = 0;
 
// static u16 RIbus_LPF = 0, LIbus_LPF = 0;
 

/ * Extern function prototypes --------------------------------------------- - * /

Extern void system_init (void); // 0410 // system clock, IO, interrupt vector configuration and initialization // 0410
Extern u8 STM32_Gyro_init (void);
// extern void Read_Gyro_Data (void);
/ * Encoder extern function * /
// extern void TIM4_Configuration (void); //
// extern void extINT_init (void);
/ * Private function prototypes --------------------------------------------- - * /
Void RCC_Configuration (void);
// void GPIO_Configuration (void);
Void STM32_TIM_ADC_Configuration (void);

// void svpwm_init (void);
// void parameterInit (void);
// void SVPWM_CalcDutyCycles (void); // void SVPWM_CalcDutyCycles (u8);
Void uart_init (u32);
// void USART_SendByte (u8);
Extern void USART1_Send (u8 *, u8);
Extern void USART1_Receive (void);
Extern void Get_Gyro_Value (void);
Extern void Get_Acce_Value (void);
// extern void Gyro_Acce_Offset_Calibration (void);
// extern void Gyro_Acce_Offset_Calibration (s16 *);
Extern void LowPassFilterK1_2 (GyroAcceDataStruct *);
// extern void Gyro_Offset_Calibration (void);
Extern void delay_us (u32 nus);
Extern void GyroAcce_DataOutput (void);
Extern void GyroAcce_DataOutput875 (void);
Extern void Pitch_Yaw_Calc (void);
Extern s16 kalmanUpdate (const s16, const s16);
// extern float kalmanUpdate (const s16, const s16);

Extern s16 IntegralGyro (const s16, const s16);
// extern s32 offset_correction_integral (s16);
Extern s16 offset_correction_integral (s16);
// extern u16 svpwmCount;
Extern void test_3DataOutput (s16, s16, s16);
Extern void test_2DataOutput (s16, s16);
Extern void test_1DataOutput (s16);
// void DAC_Config (void);
// void HallCfg (void);
// float Anglek = 0, AngleI = 0;
S16 Anglek = 0;
// extern u8 LIbusHP1, LIbusHP2, RIbusHP1, RIbusHP2;
// extern u16 LIbusHPcount, RIbusHPcount;
// extern void PID_Vars_init (void);
// extern s16 PID_Regulator (s16, s16, PID_Vars_Structure *);
Extern u8 printsector;
Extern u16 Lsector, Rsector;
Extern void Gyro_Data_ready_check (void);
Extern void Beep (void);
Extern void key (void);
Extern void SPI2_Configuration (void);
// extern void OLED_Init (void);
// extern void OLED_ShowString (u8, u8, u8, const u8 *);
// extern void OLED_ShowNum (u8, u8, s32, u8, u8);
// extern void OLED_Refresh_Gram (void);
// extern void OLED_ShowChar (u8, s8, u8, u8, u8);
Extern void Joystick_offset_Calibration (void);
Extern void Joystick_Handle (void);
Extern void MCU_Flash_Configuration (void);
Extern void Offset_Calibration (void);
Extern void UpAngleOffsetSpeedcompensation (void);
Extern void L3G4200D_init (void);
S16 AkCount = 0, GkCount = 0, IkCount = 0, JitterTestCount = 0;
// float Ak = 4.0, Gk = 50.0, Ik = 0.015;
// float Ak = 0.25, Gk = 0.02, Ik = 0.015; // 2012-12-14 AM Ak +/- 10 * 0.001, Gk-17 ~ -19 * 0.001, Ik + 0 ~ 20 * 0.0005; JitterTestvalue-5 ~ 0 * 1
// float Ak = 0.24, Gk = 0.002, Ik = 0.025, GAk = 0.025; //GAk=0.04; // 2012-12-14 PM; Gk +/- 0.0001 // GAk = 0.02
// float Ak = 0.22, Gk = 0.008, Ik = 0.027, GAk = 0.016; // 2013-05-15
// * float Ak = 0.21, Gk = 0.015, Ik = 0.03, GAk = 0.018; // 2013-05-16
// float Ak = 0.5, Gk = 0.004, Ik = 0.025, GAk = 0.08;
// float Kpa = 0.25, Kpg = 0.002;
// float Kpa = 0.1, Kpg = 0.002; //Kpa=0.1, Kpg=0.001;
// float Ak = 0, Gk = 0, Ik = 0; //0.005; //0.009;
// float ek = 0;
// float ek_1 = 0;
Float iState = 0;
// float pidTerm = 0;

// float pk = 1.5, ik = 0.17, kd = 9.1; // where Anglek has been multiplied by 0.011375, Gdata * 0.009
// float pk = 0.25, ik = 0.005, kd = 0.009, GAk = 0; // 2013-06-26
// float pk = 0.35, ik = 0.015, kd = 0.016, GAk = 0.07;
// float pk = 0.33, ik = 0.017, kd = 0.013, GAk = 0.06; // 2013-06-28
// float pk = 1.0, ik = 0.02, kd = 0.013, GAk = 0.04; // 2013-06-28 debug instruction feedback experiment
// float pk = 0.05, ik = 0, kd = 0;

// 2013-06-29 After adding adjustable items: //
The same time as the above-
Float pk = 0.33, ik = 0.017, kd = 0.013, GAk = 0.06; // AGpid parameter
Float k1 = 1.0, k2 = 0.02, k3 = 0.04; // feedback scale parameter

// extern void BlanceCalc (void);
// extern s16 Apid (void);
Extern s16 AGpid (void);

U8 LIbus_Protect_Flag = 0, RIbus_Protect_Flag = 0;
U8 No_Gyro_Data_Count = 0;
U8 LowBattFlg = 0;

U16 Acc_testCount = 0;
U16 Acc_RDY_Last = 0, Acc_RDY = 0, Acc_RDY_Flag = 0;

S16 BalCmd [12] = {0,0,0,0,0,0,0,0,0,0,0};
U8 speedkeepi = 0;

S16 Upcomp = 0, UpcompOffset = 0, UpcompMax; // = 400;

Extern void IBusOffsetInit (void);

// extern void YawCalc (void);
// test:
// u16 swap_HLbyte (u16);
// void Pitch_Yaw_Calc (void);
/ * Private functions ---------------------------------------------- ----------- * /
/ **
  * @brief Main program
  * @param None
  * @retval None
  * /
Int main (void)
{
 U0 tab [10] = {'U', 'A', 'R', 'T', 0x20, 'O', 'K', '!', 0x0D, 0x0A}; // 0x0D = \ r; 0x0A = \ N
 U8 static uart_Tx_count = 0; //, No_Gyro_Data_Count = 0
 // s32 tmp;
 Float tmp1, tmp2, temp5;
 Static float tmp3 = 0, tmp4 = 0, tmp4_LPF = 0;
// u8 i; // opt;
 Static u16 Vbusdisplay = 0;
 S16 s16testdata

// static s16 LIbusMax = 0, RIbusMax = 0;
// s16 LI, RI;
// static s16 LrpmMax = 0, RrpmMax = 0;
// static s16 AnglekMax = 0;
// static s16 UpAngleOffsetMax = 0;
// s16 s16Anglek;

 Static u16 Gyro_Count = 0;

 U8 i;
 
 
  / *! <At this stage the microcontroller clock setting is already configured,
       This is done through SystemInit () function which is called from startup
       File (startup_stm32f10x_xx.s) before to branch to application main
       To reconfigure the default setting of SystemInit () function, refer to
       System_stm32f10x.c file
     * /
/*Step1.11111111111111111111111111111111111111111111111111111111111
// POWER ON INIT:
   / * System Clocks Configuration * /
   / * GPIO Configuration * /
   / * NVIC Configuration * /
   System_init (); // 0410 // clock \ IO \ interrupt vector initialization
     
// / * System Clocks Configuration * /
// RCC_Configuration ();
// / * GPIO Configuration * /

  / * SPI Configuration * /
  // spi_init ();
  SPI2_Configuration ();

  / * MCU_Flash_Config * /
  MCU_Flash_Configuration ();

  
 
/*Step2.22222222222222222222222222222222222222222222222222222222222222222222222222222222222*/
  
 
  / * TIMER & ADC Configuration * /
  STM32_TIM_ADC_Configuration ();
  
  // offset init:
  // Offset_init (); // Calculate Acc, Gyro, ICS, ADC 0 offset value; to be completed 2012-09-01
// HallCfg (); // setup before STM32_TIM_ADC_Configuration
  / * Encoder extern function * /
/// * OLED Configuration * /
// SPI2_Configuration (); // xx0410
/ * OLED welcome information * / // OLEDwelcome (); // include progress bar
// OLED_Init ();
// OLED_ShowString (32,16,16, "WELCOME!");
// OLED_ShowString (32,16,16, "WELCOME!");
//
// for (i = 0; i <128; i ++)
// {
// OLED_Refresh_Gram ();
//}
/*Step3.33333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333
// Self-checking & Show Results ........ Self-checking of All key items must be Done!
  
/ * UART1 test: * /
 Uart_init (Uart_Brr);
 Delay_us (1000);
 USART1_Send (tab, 10);
 Delay_us (5000);
 USART1_Send ("WELCOME! \ R \ n", 10);
 Delay_us (5000);
  
  CPU_LED_ON; // CPU_LED
// steering_offset_Calibration; // initialize the orientation position
// Joystick_offset_Calibration (); // get Joystick_offset

 Delay_us (200000); // 1s changed to 0.2s
 / ** /
  GyroAcce_state = STM32_Gyro_init (); // test0422
  If (GyroAcce_state == 3)
  {
   // OLED_ShowString (16,32,16, "MEMS Init OK!");
   USART1_Send ("MEMS Init OK! \ R \ n", 15);
   Delay_us (5000); // send up to 8 bytes per ms
  }
  Else
  {
// OLED_ShowString (16,32,16, "MEMS Init");
// OLED_ShowString (40,48,16, "ERROR!");
USART1_Send ("MEMS Init ERROR! \ R \ n", 18);
Delay_us (5000);
  }
// for (i = 0; i <128; i ++)
// {
// OLED_Refresh_Gram ();
//}
 While (GyroAcce_state! = 3) // test0422
  Delay_us (2000000);
// Offset_Calibration_Flag = 1;
// Gyro_Acce_Offset_Calibration ();
// Offset_Calibration_Flag = 0;
  LED1_ON;
  LED2_ON;
  LED3_ON;
  LED4_ON;
  LED5_ON;
  LED6_ON;
  LED7_ON;
  LED8_ON;

  BEEP_ON;
  Delay_us (100000);
  BEEP_OFF;
  Offset_Calibration ();

  LED5_OFF;
  LED6_OFF;
  LED7_OFF;
  LED8_OFF;

 
// PID_Vars_init ();
  CPU_LED_ON; // CPU_LED
  // PWM_SD
  // MOT_SD_OFF; // Lmotor SD off
// RU_SD_OFF;
// RV_SD_OFF;
// RW_SD_OFF;
// LU_SD_OFF;
// LV_SD_OFF;
// LW_SD_OFF;
  // GPIO_ResetBits (GPIOE, GPIO_Pin_2); // Rmotor SD off
// OLED_Init ();
// LIbus_Protect_Flag = 0; // 0410 Clear the current protection value before stabilization
// RIbus_Protect_Flag = 0;

IBusOffsetInit ();
// delay_us (1000);
// LIbus_Protect_Flag = 0; // 0410 Clear the current protection value before stabilization
// RIbus_Protect_Flag = 0;

While (1)
{

 WhileCount ++;
 If ((whileCount == 0x0000) && (TestKeyNo == 0x0))
 {
// GPIO_ResetBits (GPIOF, GPIO_Pin_14); // oled reset
  LED4_ON;
  If (LowBattFlg)
  LED1_ON;
  If (LIbus_Protect_Flag)
  LED3_ON;
  If (RIbus_Protect_Flag)
  LED2_ON;
  
 }
 Else if ((whileCount == 0x7FFF) && (TestKeyNo == 0x0))
 {
// GPIO_SetBits (GPIOF, GPIO_Pin_14);
// OLED_Init ();
   LED4_OFF;
   If (LowBattFlg)
    LED1_OFF;
   Else
LED1_ON;
   If (LIbus_Protect_Flag)
  LED3_OFF;
   Else
  LED3_ON;
   If (RIbus_Protect_Flag)
  LED2_OFF;
   Else
  LED2_ON;
 }

// ADC_SoftwareStartConvCmd (ADC1, ENABLE);
// ADC_SoftwareStartConvCmd (ADC2, ENABLE);
 ADC_SoftwareStartConvCmd (ADC3, ENABLE);
  
 If (outputEN == 0) // SD output delay
 {
  TCount ++;
  If (tCount> 1000)
  {
   OutputEN = 1;
   // MOT_SD_ON; // motor ON, SD = 0
// RU_SD_ON;
// RV_SD_ON;
// RW_SD_ON;
// LU_SD_ON;
// LV_SD_ON;
// LW_SD_ON;
MaxAnglekProtect = 0; // open sd
TIM_Cmd (TIM1, ENABLE); // TIM1 is on
    TIM_Cmd (TIM8, ENABLE); // TIM8 on
   // GPIO_SetBits (GPIOE, GPIO_Pin_2); // Rmotor ON SD = 0
  }
  CPU_LED_ON; // LED
 }
 
 
 FootSwitchState = FSW_Read_L1;
 FootSwitchState << = 1;
 FootSwitchState | = FSW_Read_L2;
 FootSwitchState << = 1;
 FootSwitchState | = FSW_Read_R1;
 FootSwitchState << = 1;
 FootSwitchState | = FSW_Read_R2;
 
  If ((FootSwitchState & 0x0C) == 0x0C) // Footswitch indication
 {
  // OLED_ShowChar (64,0, 'A', 16,1);
// OLED_ShowString (64,0,16, "OFF OFF");
  LED6_OFF;
 }
 Else
 LED6_ON;

 If ((FootSwitchState & 0x03) == 0x03)
 {
// OLED_ShowString (64,0,16, "OFF ON");
  LED5_OFF;
 }
 Else
 LED5_ON;



 USART1_Receive ();
 Beep ();
 Key ();

// OLED_ShowNum (0,0, testkeydata, 3,16);
// if (Redrawi> = 0x80)

  If ((whileCount & 0x0FFF) == 0x0000) // 3sec reflash vbus display
  {
    Vbusdisplay = Vbus / 20; //
    If (Vbusdisplay <= 75) // low voltage alarm!
    {
   // Beep ();
// LED4_OFF;
// LED3_OFF;
// LED2_OFF;
LowBattFlg = 1;
}
Else if (TestKeyNo == 0x0)
{
LowBattFlg = 0;
If (Vbusdisplay> = 79)
{
// LED4_ON;
// LED3_ON;
// LED2_ON;
LED1_ON;
}
Else if (Vbusdisplay> = 78)
{
// LED4_OFF;
// LED3_ON;
// LED2_ON;
LED1_ON;
}
Else if (Vbusdisplay> = 77)
{
// LED4_OFF;
// LED3_OFF;
// LED2_ON;
LED1_ON;
}
Else if (Vbusdisplay> = 76)
{
// LED4_OFF;
// LED3_OFF;
// LED2_OFF;
LED1_ON;
}
}
  }
// if (Lrpm> LrpmMax)
// LrpmMax = Lrpm;
// if (Rrpm> RrpmMax)
// RrpmMax = Rrpm;
 {// x, y
// OLED_ShowNum (0,0, Vbusdisplay, 4,16); // 32);
// OLED_ShowNum (0,16, AkCount, 4,16); // 32);
// OLED_ShowNum (0,32, GkCount, 4,16); // 32);
// OLED_ShowNum (0,48, IkCount, 4,16); // 32);
// OLED_ShowNum (0,16, AnglekMax, 4,16); // 32); // show the maximum dip
// s16Anglek = (s16) Anglek;
// s16Anglek >> = 2;
// test: show the acceleration of the original data (tilt), // show the acceleration of the original data (tilt)
// OLED_ShowNum (0,16, s16Anglek, 4,16); // 32); // show the inclination
// OLED_ShowNum (0,32, LrpmMax, 4,16); // 32); // show the maximum speed value
// OLED_ShowNum (0,48, RrpmMax, 4,16); // 32)
  // OLED_ShowNum (64,0, Joystick_data, 4,16); //
  // OLED_ShowNum (64,0, Joystick_Value, 4,16); //
// OLED_ShowNum (64,32, JitterTestCount, 4,16); // 32);
  // OLED_ShowNum (64, 16, ErrorInfo, 4, 16);
  // OLED_ShowNum (64,16, JitterTestCount, 4,16);
  // if (UpAngleOffset> UpAngleOffsetMax)
  // UpAngleOffsetMax = UpAngleOffset;
// OLED_ShowNum (64,16, UpAngleOffsetMax >> 3,4,16);
// Redrawi = 0;
 }


// test maximum current display:
// LI = LIbus_LPF >> 3;
// RI = RIbus_LPF >> 3;
// if (LI> LIbusMax)
// LIbusMax = LI;
// if (RI> RIbusMax)
// RIbusMax = RI;

// OLED_ShowNum (64,32, (LIbusMax), 4,16); // left bus current display
// OLED_ShowNum (64,48, (RIbusMax), 4,16); // right bus current display
// OLED_ShowChar (64,32, 'A', 16,1);
 
// if (Offset_Calibration_Flag)
// Gyro_Acce_Offset_Calibration ();
 
// test Acc: // 0410
Acc_RDY = Get_Acce_RDY;
If (Acc_RDY! = Acc_RDY_Last)
{
Acc_testCount ++;
If (Acc_testCount> = 100)
{
Acc_RDY_Flag ++;
Acc_testCount = 0;
If (Acc_RDY_Flag & 0x0001)
{
// LED4_ON;
LED8_ON;
}
Else
{
// LED4_OFF;
LED8_OFF;
}
}
}
Acc_RDY_Last = Acc_RDY;

 Gyro_Data_ready_check ();
 
// Gyro_Data_ready = 0; // test2013-04-07
 If (Gyro_Data_ready)
 {
 Gyro_Count ++; // signal detection indication
If (Gyro_Count == 100)
LED7_ON;
If (Gyro_Count> = 200)
{
LED7_OFF;
Gyro_Count = 0;
}
The same time as the above-
    
  UpAngleOffsetSpeedcompensation (); // angle compensation calculation
  Get_Gyro_Value ();
  Get_Acce_Value ();
  // LowPassFilterK1_2 (pGyroData);
  // LowPassFilterK1_2 (pAcceData);
  // Gyro_Acce_process ();
  Pitch_Yaw_Calc (); // Execution time 1.833us
  // Gyro_Data_ready = 0;
// pGyroData -> xdatapu875 >> = 4; // 2; // 5;
// pAcceData -> xdatapu875 >> = 4; // 2; // 5;
  Anglek = kalmanUpdate (pGyroData -> xdatapu875, pAcceData -> xdatapu875); // Execution time 7.278us



  // if (Anglek> AnglekMax) // Save the maximum inclination
  // AnglekMax = Anglek;

  / * Angle protection; tentative 30 °, protection after the implementation of the need to shut down to restart the normal use *
// if ((Anglek> 760) || (Anglek <-760)) // The protection angle is set incorrectly
  // if ((Anglek> 3430) || (Anglek <-3430)) // + / - 30 °
// if ((Anglek> 2285) || (Anglek <-2285)) // + 40 °, -40 ° jysPCB // test0422
  If ((Anglek> 4580) || (Anglek <-4580)) // + 40 °, -40 ° jysPCB // 0425
  {
  // PWM_SD
  // MOT_SD_OFF; // motor SD off
// RU_SD_OFF;
// RV_SD_OFF;
// RW_SD_OFF;
// LU_SD_OFF;
// LV_SD_OFF;
// LW_SD_OFF;
MaxAnglekProtect = 1;
  }
  // anglek = kalmanUpdate (pGyroData -> xdatapu875, pAcceData -> ydatapu875); // kalman for ext.board
  // Anglek = IntegralGyro (pGyroData -> xdatapu875, pAcceData -> xdatapu875);
  // AngleI = (s16) offset_correction_integral (pGyroData -> xdatapu875);
  // AngleI = (s16) offset_correction_integral (Anglek);
// Anglek >> = 2;
// pitchcmd = pGyroData -> xdatapu875 >> 2;
  // pitchcmd = 8 * Anglek + pGyroData -> xdatapu875; // >> 6; // k2 * pGyroData -> xdatapu875;
// Balcommand = 8 * Anglek + pGyroData -> xdatapu875 >> 2; // card
// Balcommand = 8 * (Anglek + (s16) offset) + pGyroData -> xdatapu875 >> 2 + AngleI; //
// pitchcmd = 5 * anglek; + pGyroData -> xdatapu875 / 4; // too big burner
// tmp1 = Anglek / Ak; // 3; // 2; / >> 4; /// 10; // 30 ° corresponds to 3428
// tmp2 = pGyroData -> xdatapu875 / gk; // 256; /// 192; // 128; >> 8;
  // UpAngleOffset = 0; // test: mask upturned
  // * tmp1 = (Anglek + (float) UpAngleOffset) * Ak; // (kalman angle + upturn angle) * Angle scale factor
// tmp1 = Anglek * Ak; // mask upturned


  // tmp2 = pGyroData -> xdatapu875; // * Gk; //0.02;
  // tmp2 = offset_correction_integral (pGyroData -> xdatapu875);
// s16testdata = offset_correction_integral (pGyroData -> xdatapu875);
// tmp2 = s16testdata;
  // tmp3 = tmp2;

  // increase the number of differential items, increase the number of points in the integral items or increase in the outside, to be experimentally decided!
  // The original gyroscope differential term has been multiplied by the Gk coefficient before it will reduce the differential effect;
  // modified to calculate without the Gk factor before the need to modify GAk = 0.04 for GAk = 0.04?
  // * tmp1 + = (tmp2-tmp3) * GAk; // differential term is added in the integral term; + differential term * differential coefficient
// tmp1 = (tmp2-tmp3) * 0.01; // 2013-06-13
  // tmp2 + = tmp3 * 3/4; // GyroData LPF
// tmp2 + = tmp3 * 3;
// tmp2 / = 4;
// tmp3 = tmp2;

  // tmp2 * = GAk; // kd; //0.009; // GAk; //0.0105; //1.3: 1.2*8.75/1000

// ek_1 = ek;
// ek = Anglek + (float) UpAngleOffset; //

  / / Start lean forward compensation program five:
// if (LRrpm> 800) //> 500) / / before the acceleration compensation; 500 turn will start upturned
// {
// tmp = LPF_AcceLRrpm * GAk;
// Upcomp = (s16) tmp;
// if (Upcomp <UpcompMax)
// Upcomp = UpcompMax; // Keep the highest compensation value
// else else
// UpcompMax = Upcomp; // Save the highest compensation value
//
// if (LPF_AcceLRrpm <-10) // Deceleration threshold
// {
// UpcompMax = 0;
// if (Upcomp> 10) // error! Fix
// Upcomp - = 10;
// else if (Upcomp> 0)
// Upcomp = 0;
//}
//}
// else if (LRrpm <-800)
// {
// tmp = LPF_AcceLRrpm * GAk;
// Upcomp = (s16) tmp;
// if (Upcomp> UpcompMax)
// Upcomp = UpcompMax; // Keep the highest compensation value
// else else
// UpcompMax = Upcomp; // Save the highest compensation value
//
// if (LPF_AcceLRrpm> 10) // slow down the threshold
// {
// UpcompMax = 0;
// if (Upcomp <-10) // error! Fix
// Upcomp + = 10;
// else if (Upcomp <0)
// Upcomp = 0;
//}
//}
// else if (((LRrpm <400) && (LRrpm> -400)))
// {
// UpcompMax = 0;
// if (Upcomp> 0) // error!
// Upcomp -;
// else if (Upcomp <0)
// Upcomp ++;
//}
//
// if (Upcomp> 600) // maximum limit
// Upcomp = 600;
// else if (Upcomp <-600)
// Upcomp = -600;
  // start the forward compensation program five end
//
// if ((LRrpm> 1300) || (LRrpm <-1300)) // start forward compensation scheme four, acceleration compensation
// {// 1300, the initial compensation speed of 2.55km / h, is high?
// tmp = LPF_AcceLRrpm * GAk;
// Upcomp = (s16) tmp;
//
// if (Upcomp> 600) // maximum limit
// Upcomp = 600;
// else if (Upcomp <-600)
// Upcomp = -600;
//}
// else else
// {
// // Upcomp = 0;
// // if (tmp> 10) error!
// if (Upcomp> 10) // error! Fix
// Upcomp - = 10;
// else if (Upcomp> 0)
// Upcomp = 0;
// if (Upcomp <-10)
// Upcomp + = 10;
// else if (Upcomp <0)
// Upcomp = 0;
//}

  // ek + = (float) Upcomp;

// if (ek> 50) // start the forward compensation program three
// {
// if ((upLRCC> 120) && (upLRCC <320))
// {
// if (Upcomp <600)
// Upcomp + = 10;
//}
// else if (Upcomp> 10)
// Upcomp- = 10;
//}
// else if (ek <-50)
// {
// if ((upLRCC <-120) && (upLRCC> -320))
// {
// if (Upcomp> -600)
// Upcomp- = 10;
//}
// else if (Upcomp <-10)
// Upcomp + = 10;
//}
  
// ek + = (float) Upcomp * GAk; // start forward compensation program three end

// ek + = (float) Upcomp * GAk; // start forward compensation program two
// ek + = UpcompOffset;
// if (ek> UpcompOffset) // start forward compensation program one
// Upcomp ++;
// else if ((ek> 0) && (Upcomp> 1))
// Upcomp--;
// if (ek <-UpcompOffset)
// Upcomp--;
// else if ((ek <0) && (Upcomp <-1))
// Upcomp ++;
// if (Upcomp> UpcompMax)
// Upcomp = UpcompMax;
// else if (Upcomp <-UpcompMax)
// Upcomp = -UpcompMax;
//
// ek + = Upcomp; // start forward compensation program one end

  //ek*=0.011375; // inclination error //1.3: 1.2*8.75/1000
  // ek + = tmp2; // angular velocity error
// ek + = tmp1; // angular acceleration error //1.3: 1.2*8.75/1000
  //Kpa=0.25, Kpg=0.002;
  // temp5 = (Anglek + (float) UpAngleOffset) * Kpa + tmp3 * Kpg; // add Kp items
// if (tmp3> 6000) / / anti-Meng pressure - angular speed> 6000 * 8.75 / 1000 = 52.5dps
// {
// Gk = 0.004;
//}
// else else
// {
// Gk = 0.002;
//}
// tmp2 + = tmp1;
// tmp3 * = 3;
// tmp3 + = tmp2;
// tmp3 / = 4;
  // * tmp1 + = tmp3 * Gk; // The gyro factor is moved here
// tmp4 + = tmp3 * Ik;
  // * tmp4 + = tmp1 * Ik; // pair (kalman angle * Ak + gyroscope * Gk + gyroscope differential * GAk) integral
  // 0426, where the need to increase the angle, angular velocity proportional items, to be studied PID and experiment:
  // or after filtering
  // BlanceCalc ();
  // tmp4 = pidTerm; // speed increase will fall
// tmp4 = Apid ();
// tmp4 + = tmp2;
// tmp4 + = GyroData_Self_offset * kd;
  // tmp4 + = pidTerm; / / difficult to adjust, whether the convergence to be analyzed
  Tmp4 = AGpid ();
  // tmp4_LPF * = 3; // 2012-12-13
  Tmp4_LPF + = tmp4;
// // * tmp4_LPF + = tmp1;
  (PgyroData -> xdatapu875 * Gk) + Σ (anglek * Ak + LPF (pGyroData -> xdatapu875 * Gk)) * Ik] / hmp4_LPF / = 2; // 4 / hmp4_LPF = LPF [
  // tmp4_LPF = tmp4;
   //0.015; // accumulate X '//0.0047, 0.002, 0.006, 0.01, 0.2
  
  //// interval judgment, foot switch release emergency stop, add the following soft Kai, invalid !! 2012-12-10
  // tmp4 * = softFactor;
  Tmp4_LPF * = softFactor;
  // tmp3 * = softFactor;
  IState * = softFactor;

  // temp5 + = tmp4_LPF;
  Temp5 = tmp4_LPF;
// temp5 * = softFactor;

  If (temp5> 10000) // data type before conversion protection
  {
     Temp5 = 10000;
  }
  Else if (temp5 <-10000)
  {
  Temp5 = -10000;
  }
// pitchcmd = (s16) (tmp1 + tmp2 + tmp4_LPF);
  // pitchcmd = (s16) temp5; // tmp4_LPF; // (tmp3 + tmp4_LPF);
  For (i = 0; i <11; i ++)
  {
  BalCmd [i] = BalCmd [i + 1];
  }
  // BalCmd [11] = (s16) temp5;
  // Y = k1 * X (k) + k2 * Y (k-1)
  
  If (speedkeepi> 7) // debug delay time
  {Speedkeepi = 0;}
  Pitchcmd = (BalCmd [speedkeepi] + BalCmd [speedkeepi + 1] + BalCmd [speedkeepi + 2] + BalCmd [speedkeepi + 3]);
  Pitchcmd * = k2; // Y = k2 * Y (k-1)

  Pitchcmd + = temp5 * k1; // Y + = k1 * X (k)
  
  // pitchcmd = temp5 + ((BalCmd [speedkeepi] + BalCmd [speedkeepi + 1] + BalCmd [speedkeepi + 2] + BalCmd [speedkeepi + 3]) 2);
/ / BalCmd [speedkeepi + 1] + BalCmd [speedkeepi + 2] + BalCmd [speedkeepi + 3]) >> 2) ;
  // BalCmd [11] = (s16) pitchcmd; // oscillation but convergent, need further study
  // temp5 = LPF_LRrpm * 0.03; // GAk; //0.03; // 2013-06-13
  // pitchcmd = (s16) temp5; // 2013-06-13
  // pitchcmd + = BalCmd [11]; // 2013-06-13
  // pitchcmd = BalCmd [11];

  Pitchvd + = BalCmd [11]; // Balcommand; 2013-06-28 test slow down problems, 29 modified // * 3; // LPF
  Pitchcmd / = 2; // 4; // increase the speed of the filter after the dead zone caused by the jitter reduction! 2013-06-14

  BalCmd [11] = (s16) pitchcmd; // save Y (k)

  Tmp3 + = LPF_LRrpm * k3; // increase speed to keep and filter
  Tmp3 + = pitchcmd;
  Tmp3 / = 2;
  Balcommand = tmp3;
  // Balcommand = pitchcmd; // PID_Regulator (0, pitchcmd, pPitchPID); // + pGyroData -> xdatapu875 >> 2;
  // Balcommand = -pitchcmd; // for ext Gyro board
// Balcommand = Anglek + pGyroData -> xdatapu875 >> 5;
  // Balcommand = AngleI >> 2; // PID_Regulator (pitchcmd, 0, pPitchPID);
  Yawcommand = Joystick_Value; // JoystickRateSet * 3; // PID_Regulator (s16 hReference, s16 hPresentFeedback, pPitchPID);

  Yawcommand * = softFactor;
// Anglek_FLAG = 0;
ERPM_LPF = 0; // mask steering compensation test test2013-05-30
  If (outputEN == 1)
  {
   If (FootSwitchState <0x0F)
   {
    If (softFactor <= 0.994)
     SoftFactor + = 0.006; //1.5sce//0.005; // 1sec
If (softFactor> = 0.994)
     SoftFactor = 1.0;

// while (Anglek_FLAG); // Avoid interrupt instruction execution when modifying motor instruction data
// These two wait instructions sometimes cause the motor command to not assign a new value
// if ((LIbus_Protect_Flag) || (RIbus_Protect_Flag))
// {
// softFactor * = 0.95;
//}
  
    Tmp1 = (Balcommand + Yawcommand + eRPM_LPF) * softFactor;
    Tmp2 = (Balcommand-Yawcommand-eRPM_LPF) * softFactor;
    // LMotorCmd = Balcommand-Yawcommand;
    // RMotorCmd = Balcommand + Yawcommand;
If (Anglek_FLAG == 0)
{
    LMotorCmd = (s16) tmp1;
    RMotorCmd = (s16) tmp2;
}
    FootSwitchStateCount = 0;
   }
   Else
   {
   
    If (softFactor> = 0.002)
     SoftFactor - = 0.002; //1.5sce//0.005; // 1sec
If (softFactor <= 0.002)
     SoftFactor = 0;

// while (Anglek_FLAG); // These two wait instructions sometimes cause the motor command to not assign a new value
    Tmp1 = (Balcommand + Yawcommand + eRPM_LPF) * softFactor;
    Tmp2 = (Balcommand-Yawcommand-eRPM_LPF) * softFactor;
If (Anglek_FLAG == 0)
{
LMotorCmd = (s16) tmp1;
RMotorCmd = (s16) tmp2;
}
         // interval judgment, foot switch release emergency stop !! 2012-12-10
    If (softFactor == 0) //((softFactor>-0.003)|(softFactor<0.003))
    {
      Tmp4 = 0; // reset
      Tmp4_LPF = 0;
// Anglek = 0;
// softFactor = 0;
      // tmp3 = 0;
UpAngleOffset = 0;

// eRPM_Integ = 0;
// eRPM_LPF = 0;
      
    }
    
   }
 
    
 
  }
  Else // Not outputEN == 1
  {
    If (Anglek_FLAG == 0)
{
LMotorCmd = 0;
RMotorCmd = 0;
    }
    Tmp4 = 0; // reset
    Tmp4_LPF = 0;
    // tmp3 = 0;
UpAngleOffset = 0;
ERPM_Integ = 0;
ERPM_LPF = 0;
  }
  Anglek_FLAG = 1;
  // debug output:
  // pGyroData -> xdatapu875 = (s16) offset_correction_integral (pGyroData -> xdatapu875);
  // pGyroData -> xdatapu875 = (s16) offset_correction_integral (pGyroData -> ydatapu875);
  // pGyroData -> xdatapu875 = (s16) offset_correction_integral (pGyroData -> zdatapu875);
  // pGyroData -> xdatapu875 = kalmanUpdate (pGyroData -> xdatapu875,0);
  Uart_Tx_count ++; //
  If (GyroAcce_DataOutput_Flag)
  {
   If (uart_Tx_count == 1) // Outputs 1 group for every 10 sets of data
   {
    // gyroAcce_DataOutput (); // max output 8 bytes per ms, output completed 1 set data need 6ms
    // GyroAcce_DataOutput875 ();
    // test_1DataOutput ((s16) Balcommand);
// test_2DataOutput (LIbus, RIbus);
// test_2DataOutput (LIbus_LPF, RIbus_LPF);
// test_2DataOutput (LIbus_LPF, Lsector);
// test_2DataOutput (pGyroData -> xdatapu875, s16testdata);
Test_2DataOutput (pGyroData -> xdatapu875, pGyroData-> x_LPF_Previous);
The same time as the above-
    Uart_Tx_count = 0;
    // Gyro corresponds to the uart output calibration value test:
    // Gyro_Offset_Calibration ();
The same time as the above-
   }
   ;
   
  }
  Gyro_Data_ready = 0;
  No_Gyro_Data_Count = 0;
  // test_2DataOutput (LiA_sensor, LiB_sensor);
  // test_2DataOutput (LIbus, RIbus);
  // test_2DataOutput (LIbus_LPF, RIbus_LPF);
  // test_3DataOutput (AkCount, GkCount, IkCount);
  // test_1DataOutput (Joystick_data);
// test_2DataOutput (Joystick_data, Joystick_Value);
// test_2DataOutput (5,55);
 }
 Else // Not Gyro_Data_ready
 {
// if (No_Gyro_Data_Count <200) // 40ms count
// No_Gyro_Data_Count ++;
// if (No_Gyro_Data_Count> = 160)
// {
//// Get_Gyro_Value ();
//// Get_Acce_Value ();
// L3G4200D_init ();
//}
  If (Get_Acce_RDY == 1)
  {
  Get_Acce_Value ();
  }
  If (No_Gyro_Data_Count> = 200) //
  {
  If (LMotorCmd> 0) // Gyro_Data If the error occurs, turn off the motor
{LMotorCmd--;}
If (LMotorCmd <0)
{LMotorCmd ++;}
If (RMotorCmd> 0)
{RMotorCmd--;}
If (RMotorCmd <0)
{RMotorCmd ++;}

// LMotorCmd = 0;
// RMotorCmd = 0;
SoftFactor = 0;
ErrorInfo | = Gyro_Data_ready_Error;
Anglek_FLAG = 1;
No_Gyro_Data_Count = 120;
  }
 }
 // delay_us (5000);
// if (GyroAcce_DataOutput_Flag)
// {
// GyroAcce_DataOutput ();
//}
 
// if ((TimUpdateCount% 40) == 0)
// {
// test_2DataOutput (LMparam.I_bus, RMparam.I_bus);
//}
 // left and right motor hard overcurrent test
// LIbusHP2 = GPIO_ReadInputDataBit (GPIOC, GPIO_Pin_9); // left motor hard overcurrent
// RIbusHP2 = GPIO_ReadInputDataBit (GPIOF, GPIO_Pin_3); // right motor hard overcurrent
//
// if (LIbusHP1! = LIbusHP2)
//// if (LIbusHP2)
// {
// LIbusHPcount ++;
//}
//
// if (RIbusHP1! = RIbusHP2)
//// if (RIbusHP2)
// {
// RIbusHPcount ++;
//}
//
// LIbusHP1 = LIbusHP2;
// RIbusHP1 = RIbusHP2;
 // test:
// if (printsector == 1)
// {
// // test_1DataOutput (Lsector);
// // test_1DataOutput (Rsector);
// test_2DataOutput (Lsector, Lrpm);
//
// printsector = 0;
//}
// for (z = 0; z <128; z ++)
 Vdd33 = (u16) (ADC3_Buffer [0]);
 Vdd33 & = 0x0FFF;
 Vbus = (u16) (ADC3_Buffer [1]);
 Vbus & = 0x0FFF;

 Joystick_data = (s16) (ADC3_Buffer [4]) - Joystick_offset;
 Joystick_Handle ();
// yawCalc ();

// RIbus = (u16) (ADC1-> DR);
// RIbus & = 0x0FFF;
// LIbus = (u16) (ADC1-> DR >> 16);
// LIbus & = 0x0FFF;
// LIbus_LPF * = 15;
// LIbus_LPF + = LIbus;
// LIbus_LPF >> = 4;
// if (LIbus_LPF> 1920) // 24A 1600) // 20A protection> 2400) // 30A
// {
// LIbus_Protect_Flag = 1;
//}
//// else if (LIbus_LPF <800)
//// {
//// LIbus_Protect_Flag = 0; // 10A protection is released
////}
// RIbus_LPF * = 15;
// RIbus_LPF + = RIbus;
// RIbus_LPF >> = 4;
// if (RIbus_LPF> 1600) // 20A protection 2400) // 30A 1920) // 24A
// {
// RIbus_Protect_Flag = 1;
//}
//// else if (RIbus_LPF <800)
//// {
//// RIbus_Protect_Flag = 0; // 10A protection is released
////}
//
// / * current protection, protection after the implementation of the need to shut down to restart the normal use *
//
// if (RIbus_Protect_Flag | LIbus_Protect_Flag) // current protection alarm!
// {
// // PWM_SD
//// MOT_SD_OFF; // Lmotor SD off
//// GPIO_ResetBits (GPIOE, GPIO_Pin_2); // Rmotor SD off
// Beep ();
//}

// OLED_Refresh_Gram ();
  } // end while

} // end main
 

/ **
  * @brief Configures the different system clocks.
  * @param None
  * @retval None
  * /
Void RCC_Configuration (void)
{
// SetSysClockTo72 (); // lihewen 2012-07-31 test, this function can not be called at here?
   / * PCLK1 = HCLK / 4 * /
// RCC_PCLK1Config (RCC_HCLK_Div8);
// RCC_PCLK2Config (RCC_HCLK_Div8);
  / * TIM1, GPIOA, GPIOB, GPIOE and AFIO clocks enable * / // lihewen | RCC_APB2Periph_GPIOC
  RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOE |
                         RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
}

//
// # define STM32F10X_CL
/ *
TIM1 - GPIOE:
Pin8 = UL
Pin9 = UH
Pin10 = VL
Pin11 = VH
Pin12 = WL
Pin13 = WH
* /
#ifdef USE_FULL_ASSERT
/ **
  * @brief Reports the name of the source file and the source line number
  * Where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval None
  * /
Void assert_failed (uint8_t * file, uint32_t line)
{
  / * User can add his own implementation to report the file name and line number,
     Ex: printf ("Wrong parameters value: file% s on line% d \ r \ n", file, line) * /
  While (1)
  {}
}
#endif

// s16 Apid (void)
// {
//// float static sttmp1 = 0, sttmp2 = 0;
// float ftmp;
// s16 tmp;
//
// tmp = Anglek + UpAngleOffset; // Aek
// if ((tmp <10) && (tmp> -10))
// tmp = 0;
//
// iState + = tmp * ik; // Aik
//
// if (iState> 1000)
// iState = 1000;
// else if (iState <-1000)
// iState = -1000;
//
//
//// sttmp2 * = 3;
//// sttmp2 + = (Anglek-sttmp1) * kd; // dTerm
//// sttmp2 / = 4; // dTerm LPF
//// sttmp1 = Anglek;
//
// ftmp = tmp * pk; // A pTerm
//// tmp + = sttmp2; // A dTerm LPF
// ftmp + = iState; // A iTerm
// return (s16) ftmp;
//}

S16 AGpid (void)
{
//pk=0.33,ik=0.017,kd=0.013, GAk=0.06;
Float ftmp;
 S16 tmp;
S32 s32tmp;
Static s16 sstmp, gdLpf;
// Anglek // integral
Tmp = Anglek + UpAngleOffset; // Aek
The same time as the above-
// if ((tmp <5) && (tmp> -5))
// tmp = 0;
  If (tmp> 5) // 2013-06-28
Tmp - = 5;
Else if (tmp <-5)
Tmp + = 5;
Else
Tmp - = 0;

  IState + = tmp * ik; // Aik

  If (iState> 1000)
   IState = 1000;
  Else if (iState <-1000)
   IState = -1000;
// Anglek // proportion
Ftmp = tmp * pk;
Ftmp + = iState;
The same time as the above-
// pGyroData -> xdatapu875 // differential + proportion
S32tmp = sstmp * 3 + pGyroData -> xdatapu875; // LPF
S32tmp / = 4;
Tmp = (s16) s32tmp;
Ftmp + = tmp * kd;

// pGyroData // differential
S32tmp = GdLpf + (tmp-sstmp); // LPF
S32tmp / = 2; // 2013-06-28 change 4 order for the second order
GdLpf = (s16) s32tmp;
// ftmp + = (tmp-sstmp) * GAk;
Ftmp + = GdLpf * GAk;
Sstmp = tmp; // save LPF
The same time as the above-
Return ((s16) ftmp);

}

// void BlanceCalc (void)
// {
// iState + = ek; // points
// if (iState> 6000) // 1250) // pwm 50% of maximum modulation pulse width tentative * limit = 95% duty value / ik
// iState = 6000; // The increase in speed may be due to the limited value that is too small
// else if (iState <-6000) // - 1000) //6000=1020/0.17
// iState = -6000; // - 1000;
//
// pidTerm = iState * ik; // iTerm
// pidTerm + = (ek-ek_1) * kd; // dTerm
// pidTerm + = ek * pk; // pTerm
//}

/////////////////////////////////////////
//ek*=0.011375; tmp2 * = 0.009;
// float pk = 1.0, ik = 0.10, kd = 10.5;
// equivalent to:
//ek=A*0.0113758+G*0.009;
//pTerm=ek*pk=A*0.0113758+G*0.009;
// iTerm + = ek * ik = (A * 0.0113758 + G * 0.009) * 0.1;
// + = A * 0.00113758 + G * 0.0009;
//-->:ikA=pk*0.0047/TiA=0.00113758-->TiA= 4.1316s
//-->:ikG=pk*0.0047/TiG=0.0009 -> TiG = 5.2s
// ->: G points + 0.0047 * G == A;
// ->: G points + = G * 0.0009 = 0.19A;
//-->:pTermA==A*0.0113758+0.19A=0.20A;

//pTermG=G*0.009+d(A*0.0113758)*kd=G*0.009+dA*0.1194459;
// ->: pTermG = 0.214 * G;
// ->: dpTermG = d (G * 0.009) * 10.5 = dG * dG * 0.0945;

// iTerma + = ekA * ik = A * 0.0113758 * 0.10;
// ->: iTerma + = A * 0.00113758;

// dTerm = dek * kd = d (A * 0.0113758 + G * 0.009) * 10.5 = dA * 0.1194459 + dG * 0.0945;

//Ak=A*0.0113758*(pk=1.0)=A*0.0113758;
//Gk=G*0.009;+dA*0.0113758*kd=G*0.009+dA*0.1194459;
// GAk = dG * 0.009 * kd +


