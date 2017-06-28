/*Joystick*/
#include "stm32f10x.h"
#include "MC_Globals.h"

#define JoystickDEADBAND 2;
s16 JoystickRateSet=0;
u8 Joystick_State;
s16 Joystick_Counter=0,Joystick_data=0;
s16 Joystick_Value=0;
s16 JoystickCount;
s16 JoystickRate;
s16 LastJoystickCount;
u8 BeginnerMODE=0;

s16 Joystick_offset=0;
extern void delay_us(u32 nus);

void Joystick_offset_Calibration(void)
{
  s32 temp;
 u16 i;
 for(i=0;i<256;i++)	 //512次改为256次
 {
  ADC_SoftwareStartConvCmd(ADC3, ENABLE);
  //while(ADC_GetFlagStatus(ADC3, ADC_FLAG_EOC) == 0);
  delay_us(25);
  temp += (u16)(ADC3_Buffer[4]);
 }
 temp >>=8;	  //512次改为256次
// temp -=20;
 temp +=70;
 Joystick_offset = (s16)temp;
}
void Joystick_Handle(void) 
{
 static s32 tmpLPF=0;
 s16 tmp;
 tmpLPF *=31;	//31;  //63;64级改为32级
 tmpLPF +=Joystick_data;
 tmpLPF >>=5;	//5; //6; 64级改为32级
 tmp = (s16)tmpLPF;
 tmp >>= 2;
 if(tmp>150)
  tmp=150;
 if(tmp<-150)
  tmp=-150;
 if((tmp>-20)&&(tmp<20))	 //死区设置//or +/-15
  tmp=0;								 //死区设置25改为20
 
 if(tmp>=20)	  //死区过渡 +/130
 	tmp -=20;
 if(tmp<=-20)
 	tmp +=20;
 Joystick_Value=tmp;	
}




