/ * Joystick * /
#include "stm32f10x.h"
#include "MC_Globals.h"

#define JoystickDEADBAND 2;
S16 JoystickRateSet = 0;
U8 Joystick_State;
S16 Joystick_Counter = 0, Joystick_data = 0;
S16 Joystick_Value = 0;
S16 JoystickCount;
S16 JoystickRate;
S16 LastJoystickCount;
U8 BeginnerMODE = 0;

S16 Joystick_offset = 0;
Extern void delay_us (u32 nus);

Void joystick_offset_Calibration (void)
{
  S32 temp
 U16 i;
 For (i = 0; i <256; i ++) // 512 times to 256 times
 {
  ADC_SoftwareStartConvCmd (ADC3, ENABLE);
  // while (ADC_GetFlagStatus (ADC3, ADC_FLAG_EOC) == 0);
  Delay_us (25);
  Temp + = (u16) (ADC3_Buffer [4]);
 }
 Temp >> = 8; // 512 times to 256 times
// temp - = 20;
 Temp + = 70;
 Joystick_offset = (s16) temp;
}
Void joystick_Handle (void)
{
 Static s32 tmpLPF = 0;
 S16 tmp;
 TmpLPF * = 31; // 31; // 63; 64 to 32
 TmpLPF + = Joystick_data;
 TmpLPF >> = 5; // 5; // 6; 64 to 32
 Tmp = (s16) tmpLPF;
 Tmp >> = 2;
 If (tmp> 150)
  Tmp = 150;
 If (tmp <-150)
  Tmp = -150;
 If ((tmp> -20) && (tmp <20)) // dead zone setting // or +/- 15
  Tmp = 0; // dead zone setting 25 to 20
 
 If (tmp> = 20) // dead zone transition + / 130
 Tmp - = 20;
 If (tmp <= - 20)
 Tmp + = 20;
 Joystick_Value = tmp;
}



