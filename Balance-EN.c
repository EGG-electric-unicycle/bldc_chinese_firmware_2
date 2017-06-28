/ *
Balance.c

* /
#include "stm32f10x.h"
#include "MC_Globals.h"

PID_Vars_Structure PitchPID, * pPitchPID = & PitchPID;
PID_Vars_Structure YawPID, * pYawPID = & YawPID;

// s16 Balcommand = 0, Yawcommand = 0;
//
// s16 LMotorCmd = 0; RMotorCmd = 0;

U8 FootSwitchState = 0;

Void PID_Vars_init (void)
{
// 32000; // 16000; // 12000,8000,6000,5000, & lt; RTI ID = 0.0 & gt;
PPitchPID-> qKi_Gain = 400; // 400 // 250; // 200
// 2, 000, // 10000; // 10000; // 6000,5000,4000,3000,2000,1600,800,400,300,200 // pkitchPID-> qKd_Gain = 0; // 200; // 16000; // 10000;
PPitchPID-> qLower_Limit_Output = ((s16) (- 600));
PPitchPID-> qUpper_Limit_Output = ((s16) (600));
PPitchPID-> wLower_Limit_Integral = ((s32) (- 19660800));
PPitchPID-> wUpper_Limit_Integral = ((s32) (19660800));
PPitchPID-> wIntegral = 0;

PPitchPID-> wPreviousError = 0;
// PitchPID. =;
}

/ *

Typedef struct // moved to struct of MOTOR_SVPWM_PARAM 2012-09-01
{
  S16 qKp_Gain;
// u16 hKp_Divisor;
  S16 qKi_Gain;
// u16 hKi_Divisor;
  S16 qLower_Limit_Output; // Lower Limit for Output limitation
  S16 qUpper_Limit_Output; // Lower Limit for Output limitation
  S32 wLower_Limit_Integral // // Lower Limit for Integral term limitation
  S32 wUpper_Limit_Integral // // Lower Limit for Integral term limitation
  S32 wintegral
  S16 qKd_Gain;
// u16 hKd_Divisor;
  S32 wPreviousError;
  S16 LPFYk_1;
} PID_Vars_Structure;
* /

/ *
Balance algorithm
Input: Anglek - kalman filter output Pitch angle;
Output: MotorSpeed ​​- motor speed command signal (control motor signal PWM pulse width);
Control algorithm: control tilt PitchError = Pitch-PitchAngleRef = 0;
* /
// PitchBalancePID ()
// {
//// PID PitchBalance control: 1.Keep PitchAngle = BalanceAngle; 2. At the same time the angular velocity PitchRate = 0.
//// T = k1 (θ + θ0) + k2θ '+ k3 (x + x0) + K4x'
//// θ0 - produce a specified offset -θ0; Adjust θI to change the tilt of the tilt
////
//// MotorSpeedError = Kmc * PitchError; // Motor speed deviation corresponding inclination deviation
//// P:
//// I:
//// D:
//
//;
//
//}


S16 PID_Regulator (s16 hReference, s16 hPresentFeedback, PID_Vars_Structure * PID_Struct)
{// current PID parameters: qKp_Gain = 4000, qKi_Gain = 380, qKd_Gain = 0;
  S32 wError, wProportional_Term, wIntegral_Term, wOutput_32;
  S64 dwAux;

  S32 wDifferential_Term;
  S32 wtemp;

 // error computation
  // wError = (s32) (hReference - hPresentFeedback); // q14: [(- 16384-16383), (16383 - (- 16384))]
  WError = (s32) (hPresentFeedback-hReference); // [- 32767,32767]
  // Proportional term computation
  WProportional_Term = PID_Struct-> qKp_Gain * wError; // s16q15 (4000) * s16q14 = s32q29

  // Integral leations
  If (PID_Struct-> qKi_Gain == 0)
  {
    PID_Struct-> wIntegral = 0;
  }
  Else
  {
    WIntegral_Term = PID_Struct-> qKi_Gain * wError; // s16q15 (4000) * s16q14 = s32q29
    DwAux = (s64) (PID_Struct-> wIntegral) + (s64) (wIntegral_Term);
    / / Integral limit: ~ (-32768, +32767) * 4000 = 131068000
    If (dwAux> PID_Struct-> wUpper_Limit_Integral) // 131068000) // integral limit is set too small to be investigated 2012-09-01
    {
      PID_Struct-> wIntegral = PID_Struct-> wUpper_Limit_Integral;
    }
    Else if (dwAux <PID_Struct-> wLower_Limit_Integral // // 131068000) //
    {
    PID_Struct-> wIntegral = PID_Struct-> wLower_Limit_Integral;
}
Else
{
PID_Struct-> wIntegral = (s32) (dwAux);
}
  }

  // Differential difference computation
  If (PID_Struct-> qKd_Gain == 0)
  {
  WDifferential_Term = 0;
  }
  Else
  {

// s32 wtemp;
  
    Wtemp = wError - PID_Struct-> wPreviousError;
    WDifferential_Term = PID_Struct-> qKd_Gain * wtemp; // q29
    PID_Struct-> wPreviousError = wError; // store value
  }
  
// wOutput_32 = (wProportional_Term / PID_Struct-> qKp_Divisor +
// PID_Struct-> wIntegral / PID_Struct-> qKi_Divisor +
// wDifferential_Term / PID_Struct-> qKd_Divisor);
  WOutput_32 = (wProportional_Term +
                PID_Struct-> wIntegral +
                WDifferential_Term);
  WOutput_32 >> = 15; // q14

/ *
Low pass filter
Yk = K * Xn + (1-K) * Yk_1; fL ≈ K / (2πT)
// K = i / k; K takes 1/4, 1/2, 5/8, 3/4, 7/8 ....
* /
The same time as the above-
// wOutput_32 + = (s32) (PID_Struct-> LPFYk_1); // * 0x0007); // 0x001F);
// wOutput_32 >> = 1; // 3; // * 1/8 // >> = 5;

  // Output Limits: ~ (-16384, +16383) // 2012-11-09
  If (wOutput_32> = PID_Struct-> qUpper_Limit_Output)
  {
      // return (PID_Struct-> qUpper_Limit_Output);
WOutput_32 = PID_Struct-> qUpper_Limit_Output;
  }
  Else if (wOutput_32 <PID_Struct-> qLower_Limit_Output)
  {
      // return (PID_Struct-> qLower_Limit_Output);
WOutput_32 = PID_Struct-> qLower_Limit_Output;
  }
// else else
// {
// return ((s16) (wOutput_32));
//}
/ *
Low pass filter
Yk = K * Xn + (1-K) * Yk_1; fL ≈ K / (2πT)
// K = i / k; K takes 1/4, 1/2, 5/8, 3/4, 7/8 ....
* /
// PID_Struct-> LPFYk_1 = (s16) (wOutput_32); // save Yk-1

Return ((s16) (wOutput_32));
}

// void checkFootSwitch (void)
// {
//
// GPIO_ReadInputDataBit (GPIOE, GPIO_Pin_14);
//
//}

