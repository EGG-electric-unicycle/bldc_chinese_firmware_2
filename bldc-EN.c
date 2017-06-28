/*BLDC.c*/
/ ************************************************* ************************************************** ******
*
* File: BLDC.c
* Hardware Environment: kech ver 1.0
* Build Environment: RealView MDK-ARM Version: 4.22
* Version: V1.0
* By: lihewen
*
* (C) Copyright 2012-, lihewen
* All Rights Reserved
*
************************************************** ************************************************** ***** /
#include "stm32f10x.h"
#include "MC_Globals.h"
#define DUTY50 ((u16) 0x0465) // 50% duty
// (1125 + 1035) / 2250 = 96%; (1125 + 900) / 2250 = 90%
(/ S16) 1080) // 98% DUTY (s16)
// () () () () () () () () () () () () () () -1080) //
(S16) 30) // ((s16) 35) // ((s16) 27) // ((s16) 33) // ((s16) 30) // ((s16) 60) / / ((S16) 25) // motor torque dead zone
// ((s16) -30) // ((s16) -35) // ((s16) -27) // ((s16) -33) // ((s16) -30) // (( S16) -60) // ((s16) -25)
() () () () () () () () () () () () () () () () ()
#define JITTER_N ((s16) -10) // ((s16) -15) // ((s16) -5) //
S16 jitterTestvalue = 15; // 10;
#define FWRW_IMPP ((s16) 50) // forward / reverse commutation setting
#define FWRW_IMPN ((s16) -50)
 
S16 Lsector, Rsector, SectorGPIOread; // hall position
S16 Lsector_Last, Rsector_Last;

U16 pwm_U = DUTY50, pwm_V = DUTY50, pwm_W = DUTY50; // 50% duty init
// u16 pwm_RU = DUTY50, pwm_RV = DUTY50, pwm_RW = DUTY50;
S16 LTq = 0, RTq = 0; // torque of L, R motor given by blance need

S16 LTqABS = 0, RTqABS = 0;
S16 Last_LTq = 0, Last_RTq = 0;
U16 pwmH = 0, pwmL = 0, pwmS = 0;
S16 LpwmH_L = 0; // LpwmL_H = 0; // up-> down, down-> up
S16 RpwmH_L = 0; //, RpwmL_H = 0;
S16 TupFlag = 3;
U32 RPerCommutCount, RPerHallCount, LPerCommutCount, LPerHallCount;
U16 Last_RPerCommutCount, Last_LPerCommutCount;
S16 Lrpm = 0, Rrpm = 0;
// s16 Lslope = 0, Rslope = 0;
// u16 LslopeCount = 0, RslopeCount = 0;
U8 Lfirstin = 0, Rfirstin = 0;
// u16 static LLast_pwm_U = DUTY50, LLast_pwm_V = DUTY50, LLast_pwm_W = DUTY50;
// u16 static RLast_pwm_U = DUTY50, RLast_pwm_V = DUTY50, RLast_pwm_W = DUTY50;
U8 CommutMode = 0;
U8 LIbusHP1 = 0, LIbusHP2 = 0, RIbusHP1 = 0, RIbusHP2 = 0;
U16 LIbusHPcount = 0, RIbusHPcount = 0;
MOTOR_PARAM LMparam, RMparam;
U8 Sector_test = 0;
S16 testLTq = 0, testRTq = 0;
U8 writeCmd_EN = 0;
S16 Zhuanxiang_Counter;
U8 Zhuanxiang_State;
Extern u8 outputEN;
U8 printsector = 0;
// s16 LiA_sensor, LiB_sensor;
// s16 RiA_sensor, RiB_sensor;
U8 Gyro_Data_ready_PinState1 = 0, Gyro_Data_ready_PinState2 = 0;
U8 Gyro_Data_ready_PinState = 0, Gyro_Data_ready_PinState_Prev = 0;

S16 RIbus = 0;
S16 LIbus = 0;
S16 RIbus_Offset;
S16 LIbus_Offset;
S16 RIbus_LPF = 0, LIbus_LPF = 0;
// u8 Sector_pp = 0;

// ADC1-2, DMA cache ADC12_Buffer [15: 0] = ADC1, ADC12_Buffer [31:16] = ADC2, ADC2,
U32 ADC3_Buffer [5]; // __attribute __ ((at (0X2000f130))) = {0};
U16 TimUpdateCount;
U8 Gyro_Data_ready;

S16 upLCommutCount = 0, upRCommutCount = 0, upLRCC = 0;
U8 upLRCommutCount_Flg = 0; //, upRCommutCount_Flg = 0;


U16 Timer100ms;
U16 perLTpwmcount, perRTpwmCount;
U16 LTpwmCount, RTpwmCount;

U8 LHallCommutCount = 0, RHallCommutCount = 0;

S32 tmps32 = 0;
S16 Lsign = 0, Rsign = 0; // speed symbol

S16 LRrpm = 0; // two rounds of speed and
S16 R_Lrmp = 0; // two rounds of speed difference
S16 AcceLRrpm = 0; // acceleration
S16 Last_LRrpm = 0; // Calculate the acceleration and save the previous speed

S16 LPF_LRrpm, LPF_R_Lrmp, LPF_AcceLRrpm;
U8 rpm_get_ready = 0;
U8 IbusOffsetRdy = 0;
 
// CommutMode:
// 0: normal LR231546 commutation
// 1: normal LR546231 commutation
// 2: normal L546231, R231546 commutation
// 3: normal L231546, R546231 commutation
// 4: LRT
// 5: LT, R231546
// 6: L546231 RT
// 7: LRT (546231)
// 2012-11-28
// 1, T trapezoidal wave to reduce commutation current impact test, compare trapezoidal wave and square wave
// The trapezoidal wave noise is slightly smaller and the current is slightly larger
// 2, compare 546231 and 231546 sector order drive, basically the same
Extern void uart_Tx (void);
Extern void Joystick (void);
Void JoystickConversion (void);
Extern void delay_us (u32 nus);

Void TIM1_UP_IRQHandler (void) // TIM1 Update interrupt service function
{
 S16 temp;
 CPU_LED_ON;
 TIM_ClearITPendingBit (TIM1, TIM_FLAG_Update); // clear interrupt flag bit
 // test
// GPIO_SetBits (GPIOE, GPIO_Pin_7); // interrupt event processing time test 22us 2012-12-06
 // interrupt job instructions
 If ((TimUpdateCount & 0x1FFF) == 0x0000) // = 4096 = 0.256sec. // Cycle 0.512sec
 {
  // GPIO_SetBits (GPIOD, GPIO_Pin_4); // CPU_LED
  // CPU_LED_ON; // CPU_LED
  // GPIO_SetBits (GPIOE, GPIO_Pin_7); // for indicate
 }
 If ((TimUpdateCount & 0x1FFF) == 0x1000)
 {
// CPU_LED_OFF;
  If (outputEN == 1)
  {
   // GPIO_ResetBits (GPIOD, GPIO_Pin_4);
   // CPU_LED_OFF;
   ;
   // GPIO_ResetBits (GPIOE, GPIO_Pin_7); // for indicate
  }
 }
 // Bus current detection:
 While (ADC_GetFlagStatus (ADC1, ADC_FLAG_JEOC) == 0); // wait for ADC conversion to complete
 CPU_LED_OFF;
 ADC_ClearFlag (ADC1, ADC_FLAG_JEOC);
  RIbus = (s16) ADC1-> JDR1; // IN0 ADC1
  LIbus = (s16) ADC2-> JDR1; // IN1 ADC2
// filter, protect the processing to be tidied here
// ADC_Cmd (ADC1, ENABLE); // enable the specified ADC1
// ADC_Cmd (ADC2, ENABLE); // enable the specified ADC2
 
 Temp = LIbus-LIbus_Offset;
 If (Last_LTq <0)
 Temp = -temp;
 LIbus_LPF * = 7; // 15;
 LIbus_LPF + = temp; // LIbus;
 LIbus_LPF >> = 3; // 4;
 // 29A) // 24A 2780) // 20A protection> 3080) // 26A
// if ((LIbus_LPF> (1580 + LI_limt)) || (LIbus_LPF <- (1580 + LI_limt)))
 If ((LIbus_LPF> (1630 + LI_limt)) || (LIbus_LPF <- (1630 + LI_limt)))
 {
  LIbus_Protect_Flag = 1;
 }

 Temp = RIbus_Offset-RIbus;
 If (Last_RTq <0)
 Temp = -temp;
 RIbus_LPF * = 7; // 15;
 RIbus_LPF + = temp; // RIbus;
 RIbus_LPF >> = 3; // 4;
 // if (RIbus_LPF <(220-RI_limt)) // 820) //> 2780) // 620 24A // 220 32A
// if ((RIbus_LPF> (1580 + RI_limt)) || (RIbus_LPF <- (1580 + RI_limt)))
 (RIbus_LPF> (1630 + RI_limt)) || (RIbus_LPF <- (1630 + RI_limt)))
 {
  RIbus_Protect_Flag = 1;
 }
// if (IbusOffsetRdy == 0)
// {
// LIbus_Protect_Flag = 0;
// RIbus_Protect_Flag = 0;
//}

 
 
 / * BLDC commutation calculation lihewen 2012-11-22 * /
// SectorGPIOread = GPIOD-> IDR; // LEFT_HALL A-PD11, B-PD10, C-PD9 = (ABC) = ORG YHPCB
 SectorGPIOread = GPIOF-> IDR; // LEFT_HALL A-PF3, B-PF4, C-PF5 = (CBA) = JYSPCB
 SectorGPIOread >> = 0x0003;
 SectorGPIOread & = 0x0007;
 Switch (SectorGPIOread)
 {
  Case 2:
   Lsector = 1;
  Break
  Case 6: // 3:
   Lsector = 2;
  Break
  Case 4: // 1:
   Lsector = 3;
  Break
  Case 5:
   Lsector = 4;
  Break
  Case 1: // 4:
   Lsector = 5;
  Break
  Case 3: // 6:
   Lsector = 6;
  Break
  Default:
  Break
 }

// SectorGPIOread = GPIOE-> IDR; // RIGHT_HALL A-PE5, B-PE4, C-PE3 = (ABC)
 SectorGPIOread = GPIOG-> IDR; // RIGHT_HALL A-PG6, B-PG7, C-PG8 = (CBA)
 SectorGPIOread >> = 0x0006;
 SectorGPIOread & = 0x0007;
 Switch (SectorGPIOread)
 {
  Case 2:
   Rsector = 1;
  Break
  Case 6: // 3:
   Rsector = 2;
  Break
  Case 4: // 1:
   Rsector = 3;
  Break
  Case 5:
   Rsector = 4;
  Break
  Case 1: // 4:
   Rsector = 5;
  Break
  Case 3: // 6:
   Rsector = 6;
  Break
  Default:
  Break
 }
 // 50ms Timing speed: / / If you change to 100ms once, the speed calculation product factor is also * = 80000;
 // 50ms timing speed, encountered commutation time about 50ms in the middle of the timing cycle,
 // sometimes can not detect more than 50% of the timing cycle of the commutation pulse width;
 // Therefore, for reliable, choose 100ms timing check!
 Timer100ms ++;
 PerLTpwmCount ++; // left PWM cycle count
 PerRTpwmcount ++; // right PWM cycle count

 If (Timer100ms> = 1600) / / If you change to 100ms once, here changed to 1600;
 {
 Timer100ms = 0; // timer reset
If ((LHallCommutCount> = 2) && (LTpwmCount> 400))
{
// if (LTpwmCount <10)
// LTpwmCount = 10;
Tmps32 = (LHallCommutCount-1);
Tmps32 * = 80000; / / If you change to 100ms once, here is still * = 160000;
Tmps32 / = LTpwmCount;

Lrpm = (s16) tmps32;
Lrpm * = Lsign;
}
Else
Lrpm = 0;

If ((RHallCommutCount> = 2) && (RTpwmCount> 400))
{
// if (RTpwmCount <100)
// RTpwmCount = 100;
Tmps32 = (RHallCommutCount-1);
Tmps32 * = 80000;
Tmps32 / = RTpwmCount;

Rrpm = (s16) tmps32;
Rrpm * = Rsign;
}
Else
Rrpm = 0;

LHallCommutCount = 0; // left commutation count count reset
RHallCommutCount = 0; // right commutation count count reset
LTpwmCount = 0; // left PWM period count count reset
RTpwmCount = 0; // right PWM period count count reset
Rpm_get_ready = 1;
The same time as the above-
The same time as the above-
 }

 If (Lsector_Last! = Lsector)
 {
 LHallCommutCount ++;
 If (LHallCommutCount == 1) // detected the first commutation signal
{
PerLTpwmCount = 0;
}
// else if (LHallCommutCount> = 2)
LTpwmount = perLTpwmCount; // Save the number of PWM cycles for commutation
If ((Lsector_Last-Lsector) == 1) || ((Lsector-Lsector_Last) == 5))
Lsign = -1;
Else
Lsign = 1;
 }

 If (Rsector_Last! = Rsector)
 {
 RHallCommutCount ++;
 If (RHallCommutCount == 1) // detected the first commutation signal
{
PerRTpwmCount = 0;
}
// else if (RHallCommutCount> = 2)
RTpwmCount = perRTpwmCount; // Save the commutation of the PWM cycle count value
If ((Rsector-Rsector_Last) == 1) || ((Rsector_Last-Rsector) == 5))
Rsign = -1;
Else
Rsign = 1;
 }

 

// // Speed: RPerCommutCount, RPerHallCount, LPerCommutCount, LPerHallCount;
// if (RPerCommutCount <80001)
// RPerCommutCount ++;
// if (LPerCommutCount <80001)
// LPerCommutCount ++;
// // u16 Last_RPerCommutCount, Last_LPerCommutCount;
// // s16 Last_LTq = 0, Last_RTq = 0;
// if (Lsector_Last! = Lsector)
// {
// // print sector
// printsector = 1;
// // left motor trapezoidal wave:
// Lfirstin = 1;
// Last_LPerCommutCount = LPerCommutCount; // Save left motor last sector cycle count
//// Last_LTq = LTq; // Save the left motor sector commutation before the torque value
// // left motor speed: (advance 123456)
// if (LPerCommutCount == 0) // except 0 value protection
// LPerCommutCount = 1;
// lrpm = (s16) (80000 / LPerCommutCount); // rpm: rpm
//
// if (LPerCommutCount> 160)
// LPerCommutCount = 0;
// upLCommutCount = (s16) LPerCommutCount;
// upLRCommutCount_Flg = 1;
//
//
// if (((Lsector_Last-Lsector) == 1) || ((Lsector-Lsector_Last) == 5))
// {
// Lrpm * = -1;
// upLCommutCount = -upLCommutCount;
//}
// LPerCommutCount = 0;
//}
// else if (LPerCommutCount> 80000)
// {// eliminate dead speed
// Lrpm = 0;
//}
//
// if (Rsector_Last! = Rsector)
// {
// // right motor trapezoidal wave:
// Rfirstin = 1;
// // printsector = 1;
//
// Last_RPerCommutCount = RPerCommutCount; // save the right motor last sector cycle count
// // Last_RTq = RTq; // Save the right motor sector commutation torque value
// // right motor speed:
// if (RPerCommutCount == 0) // except 0 protection
// RPerCommutCount = 1;
// Rrpm = (u16) (80000 / RPerCommutCount);
//
// if (RPerCommutCount> 160)
// RPerCommutCount = 0;
// upRCommutCount = (s16) RPerCommutCount;
// upLRCommutCount_Flg = 1;
//
// if ((Rsector-Rsector_Last) == 1) || ((Rsector_Last-Rsector) == 5))
// {
// Rrpm * = -1;
// upRCommutCount = -upRCommutCount;
//}
//
// RPerCommutCount = 0;
//}
// else if (RPerCommutCount> 8000)
// {// eliminate dead speed
// Rrpm = 0;
//}
// if (upLRCommutCount_Flg)
// {
// upLRCC * = 7;
// upLRCC + = (upLCommutCount + upRCommutCount);
// upLRCC >> = 3;
//// if (upLRCC> 120)
//// Upcomp = (upLRCC-120) * 3;
//// else if (upLRCC <-120)
//// Upcomp = (upLRCC + 120) * 3;
//}

 Lsector_Last = Lsector;
 Rsector_Last = Rsector;
 
 // Read the torque reference
// if (LTqok) // given the calculated value output is valid
// if (RTqok)
// s16 LMotorCmd = 0; RMotorCmd = 0;
// writeCmd_EN = 0;
 If (Anglek_FLAG) // avoid duplicate assignments
 {
// if (TupFlag> = 3) // test the motor // test0422
// {
TestRTq = RMotorCmd; // >> 2; // test0619
TestLTq = LMotorCmd; // >> 2; // test0619
//}
  Anglek_FLAG = 0;
 }
 RTq = testRTq;
 LTq = testLTq;
 

  
 // left motor commutation calculation
 // 5 (101) 4 (100) 6 (110) 2 (010) 3 (011) 1 (001)
 // A + B- A + C- B + C- B + A-C + A-C + B-
  // eliminate peristaltic noise [-15, +15]
  If (RTq> = JitterTestvalue) // JITTER_P)
   RTq - = JitterTestvalue; // JITTER_P;
  Else if (RTq <= - jitterTestvalue) // JITTER_N)
   RTq + = JitterTestvalue; // JITTER_P;
  Else
   RTq = 0;
  If (LTq> = JitterTestvalue) // JITTER_P)
   LTq - = JitterTestvalue; // JITTER_P;
  Else if (LTq <= - jitterTestvalue) // JITTER_N)
   LTq + = JitterTestvalue; // JITTER_P;
  Else
   LTq = 0;
  // motor torque dead zone elimination TqDEADBAND_P TqDEADBAND_N
  / / If the left and right motor torque dead zone, you should define their own dead zone
  If (RTq> 0)
   RTq + = TqDEADBAND_P;
  Else if (RTq <0)
   RTq + = TqDEADBAND_N;
  Else
  {
   RTq = 0;
  }
  If (LTq> 0)
   LTq + = TqDEADBAND_P;
  Else if (LTq <0)
   LTq + = TqDEADBAND_N;
  Else
  {
   LTq = 0;
  }
  // Eliminate positive and negative reversal

  Last_LTq = LTq;
  Last_RTq = RTq;
  // the most protected
  If (RTq> TqMAX)
   RTq = TqMAX;
  If (RTq <TqMIN)
   RTq = TqMIN;
  If (LTq> TqMAX)
   LTq = TqMAX;
  If (LTq <TqMIN)
   LTq = TqMIN;
 

  // if ((LTq> -190) && (LTq <190)) // current test experiment 2013-06-20
  If ((LTq> -300) && (LTq <300)) // slow ride with switch sound 2013-06-29 modify
  CommutMode = 0;
  Else
  CommutMode = 1;
  PwmH = DUTY50 + LTq;
  PwmL = DUTY50 - LTq; //
// pwmS = DUTY50 + LpwmH_L;
// CommutMode:
// 0: normal LR231546 commutation
// 1: LRT (231546) commutation
  Switch (Lsector)
  {// Press the manual motor counterclockwise, Hall order 546231
   // positive torque left motor reversal, right motor forward (equivalent to positive torque back), due to Hall level reversal problem, ~ 010 = 101 cause
   // modified to positive torque left motor forward, right motor reverse (equivalent to positive torque) 2012-11-27
   // hall value = ~ actual sector value:
   Case 0x0001: // 0x0002: // 0x0005: // A + B- 010 = ~ 101
   Pwm_U = pwmH;
     Pwm_V = pwmL;
     Pwm_W = DUTY50;
   // CommutMode:
    If (CommutMode == 0)
    {
LU_SD_ON;
LV_SD_ON;
LW_SD_ON;
    }
Else
{
LU_SD_ON;
LV_SD_ON;
LW_SD_OFF;
}

   Break
 
   Case 0x0002: // 0x0003: // 0x0004: // A + C- 100 = ~ 011
     Pwm_U = pwmH;
     Pwm_V = DUTY50;
     Pwm_W = pwmL;
   // CommutMode:
    If (CommutMode == 0)
    {
LU_SD_ON;
LV_SD_ON;
LW_SD_ON;
    }
Else
{
LU_SD_ON;
LV_SD_OFF;
LW_SD_ON;
}
   Break
 
   Case 0x0003: // 0x0001: // 0x0006: // B + C- 110 = ~ 001
     Pwm_U = DUTY50;
     Pwm_V = pwmH;
     Pwm_W = pwmL;
   // CommutMode:
    If (CommutMode == 0)
    {
LU_SD_ON;
LV_SD_ON;
LW_SD_ON;
    }
Else
{
LU_SD_OFF;
LV_SD_ON;
LW_SD_ON;
}
   Break
 
   Case 0x0004: // 0x0005: // 0x0002: // B + A- 010 = ~ 101
     Pwm_U = pwmL;
     Pwm_V = pwmH;
     Pwm_W = DUTY50;
    // CommutMode:
    If (CommutMode == 0)
    {
LU_SD_ON;
LV_SD_ON;
LW_SD_ON;
    }
Else
{
LU_SD_ON;
LV_SD_ON;
LW_SD_OFF;
}
   Break
 
   Case 0x0005: // 0x0004: // 0x0003: // C + A 011 = ~ 100
     Pwm_U = pwmL;
     Pwm_V = DUTY50;
     Pwm_W = pwmH;
// CommutMode:
    If (CommutMode == 0)
    {
LU_SD_ON;
LV_SD_ON;
LW_SD_ON;
    }
Else
{
LU_SD_ON;
LV_SD_OFF;
LW_SD_ON;
}
   Break
 
   Case 0x0006: // 0x0001: // C + B - 001 = ~ 110
     Pwm_U = DUTY50;
     Pwm_V = pwmL;
     Pwm_W = pwmH;
   // CommutMode:
    If (CommutMode == 0)
    {
LU_SD_ON;
LV_SD_ON;
LW_SD_ON;
    }
Else
{
LU_SD_OFF;
LV_SD_ON;
LW_SD_ON;
}
   Break
 
   Default:
     Pwm_U = DUTY50;
     Pwm_V = DUTY50;
     Pwm_W = DUTY50;
   Break
  }
// LLast_pwm_U = pwm_U;
// LLast_pwm_V = pwm_V;
// LLast_pwm_W = pwm_W;
  TIM8-> CCR1 = pwm_U; // pwm_W;
  TIM8-> CCR2 = pwm_V;
  TIM8-> CCR3 = pwm_W; // pwm_U;

  CPU_LED_ON;
 // right motor commutation calculation
// if ((RTq> -190) && (RTq <190)) // current test experiment 2013-06-20
  If ((RTq> -300) && (RTq <300)) // slow ride with switching sound 2013-06-29 modify
  CommutMode = 0;
  Else
  CommutMode = 1;
  PwmH = DUTY50 + RTq;
  PwmL = DUTY50 - RTq;
  Switch (Rsector)
  {// Press the manual motor counterclockwise, Hall order 546231
   // positive torque left motor reversal, right motor forward (equivalent to positive torque back), due to Hall level reversal problem, ~ 010 = 101 cause
   // modified to positive torque left motor forward, right motor reverse (equivalent to positive torque) 2012-11-27
   // hall value = ~ actual sector value:
   Case 0x0001: // 0x0002: // 0x0005: // A + B- 010 = ~ 101
     Pwm_U = pwmH;
     Pwm_V = pwmL;
     Pwm_W = DUTY50;
   // CommutMode:
    If (CommutMode == 0)
    {
RU_SD_ON;
RV_SD_ON;
RW_SD_ON;
    }
Else
{
RU_SD_ON;
RV_SD_ON;
RW_SD_OFF;
}
   Break
 
   Case 0x0002: // 0x0003: // 0x0004: // A + C- 100 = ~ 011
     Pwm_U = pwmH;
     Pwm_V = DUTY50;
     Pwm_W = pwmL;
// CommutMode:
If (CommutMode == 0)
    {
RU_SD_ON;
RV_SD_ON;
RW_SD_ON;
    }
Else
{
RU_SD_ON;
RV_SD_OFF;
RW_SD_ON;
}
   Break
 
   Case 0x0003: // 0x0001: // 0x0006: // B + C- 110 = ~ 001
     Pwm_U = DUTY50;
     Pwm_V = pwmH;
     Pwm_W = pwmL;

// CommutMode:
If (CommutMode == 0)
    {
RU_SD_ON;
RV_SD_ON;
RW_SD_ON;
    }
Else
{
RU_SD_OFF;
RV_SD_ON;
RW_SD_ON;
}
   Break
 
   Case 0x0004: // 0x0005: // 0x0002: // B + A- 010 = ~ 101
     Pwm_U = pwmL;
     Pwm_V = pwmH;
     Pwm_W = DUTY50;
// CommutMode:
If (CommutMode == 0)
    {
RU_SD_ON;
RV_SD_ON;
RW_SD_ON;
    }
Else
{
RU_SD_ON;
RV_SD_ON;
RW_SD_OFF;
}
   Break
 
   Case 0x0005: // 0x0004: // 0x0003: // C + A 011 = ~ 100
     Pwm_U = pwmL;
     Pwm_V = DUTY50;
     Pwm_W = pwmH;
// CommutMode:
If (CommutMode == 0)
    {
RU_SD_ON;
RV_SD_ON;
RW_SD_ON;
    }
Else
{
RU_SD_ON;
RV_SD_OFF;
RW_SD_ON;
}
   Break
 
   Case 0x0006: // 0x0001: // C + B - 001 = ~ 110
     Pwm_U = DUTY50;
     Pwm_V = pwmL;
     Pwm_W = pwmH;
// CommutMode:
If (CommutMode == 0)
    {
RU_SD_ON;
RV_SD_ON;
RW_SD_ON;
    }
Else
{
RU_SD_OFF;
RV_SD_ON;
RW_SD_ON;
}
   Break
 
   Default:
     Pwm_U = DUTY50;
     Pwm_V = DUTY50;
     Pwm_W = DUTY50;
   Break
  }
  //
// RLast_pwm_U = pwm_U;
// RLast_pwm_V = pwm_V;
// RLast_pwm_W = pwm_W;
  TIM1-> CCR1 = pwm_U; // pwm_W; // TIM1 right motor, torque + = clockwise
  TIM1-> CCR2 = pwm_V;
  TIM1-> CCR3 = pwm_W; // pwm_U;
  // CPU_LED_ON;
 // current protection:
 If (LIbus_Protect_Flag | RIbus_Protect_Flag | MaxAnglekProtect)
 {
TIM8-> CCR1 = DUTY50; // TIM8 left motor
TIM8-> CCR2 = DUTY50;
TIM8-> CCR3 = DUTY50;
The same time as the above-
TIM1-> CCR1 = DUTY50; // TIM1 right motor
TIM1-> CCR2 = DUTY50;
TIM1-> CCR3 = DUTY50;
// Timing 10ms off SD
RU_SD_OFF;
RV_SD_OFF;
RW_SD_OFF;
LU_SD_OFF;
LV_SD_OFF;
LW_SD_OFF;
 }
 // uart_Tx ();
 If ((TimUpdateCount & 0x1) == 0) // 2 pwm cycles to send a character
 {
 Uart_Tx ();
 }
 TimUpdateCount ++;

 // key check timing:
 If (keycount <TimeOut_100ms) // short press time
  Keycount ++;
 Else if ((keycount <TimeOut_2000ms) && (keyState == 3)) // long press
  Keycount ++;

// GPIO_ResetBits (GPIOE, GPIO_Pin_7); // interrupt event processing time test
No_Gyro_Data_Count ++;
 CPU_LED_OFF;
}


Void IBusOffsetInit (void) // 2013-01-15
{
Static s32 itempL, itempR;
U16 n;
While (RIbus <1500); // wait for current sensor to detect normal
While (RIbus <1500);
IbusOffsetRdy = 0;
LIbus_Offset = 1800;
RIbus_Offset = 1800;
Delay_us (100);
LIbus_Protect_Flag = 0; // 0615 Clear the current protection value before stabilization
RIbus_Protect_Flag = 0;
For (n = 0; n <256; n ++)
{
ItempL + = LIbus;
ItempR + = RIbus;
Delay_us (100);
}
ItempL / = 256;
LIbus_Offset = (s16) itempL;
LIbus_Offset- = 7;
ItempR / = 256;
RIbus_Offset = (s16) itempR;
// RIbus_Offset - = 50;

IbusOffsetRdy = 1;
Delay_us (1000);
LIbus_Protect_Flag = 0; // 0615 Clear the current protection value before stabilization
RIbus_Protect_Flag = 0;
}

 
/ *
Corresponding to the motor rotation direction, Hall position signal and power tube conduction relationship:
  The HaHbHc turns counterclockwise
(5) 101 A + B-B + A-
(4) 100 A + C- C + A-
(6) 110 B + C-C + B-
(2) 010 B + A-A + B-
(3) 011 C + A- A + C-
(1) 001 C + B-B + C-
* /
 
