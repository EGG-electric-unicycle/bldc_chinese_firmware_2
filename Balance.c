/*
Balance.c

*/
#include "stm32f10x.h" 
#include "MC_Globals.h"

PID_Vars_Structure PitchPID,*pPitchPID=&PitchPID;
PID_Vars_Structure YawPID, *pYawPID=&YawPID;

//s16 Balcommand=0,Yawcommand=0;
//
//s16 LMotorCmd=0; RMotorCmd=0;

u8 FootSwitchState=0;

void PID_Vars_init(void)
{
	pPitchPID->qKp_Gain=1000; //32000;//16000;	 //12000,8000,6000,5000,
	pPitchPID->qKi_Gain=400; //400//250;	 //200
	pPitchPID->qKd_Gain=0; //200;//16000; //10000;	 //6000,5000,4000,3000,2000,1600,800,400,300,200
	pPitchPID->qLower_Limit_Output=((s16)(-600));
	pPitchPID->qUpper_Limit_Output=((s16)(600));
	pPitchPID->wLower_Limit_Integral=((s32)(-19660800));
	pPitchPID->wUpper_Limit_Integral=((s32)(19660800));	
	pPitchPID->wIntegral=0;

	pPitchPID->wPreviousError=0;
//	PitchPID.=;
}

/*

typedef struct 	  //moved to struct of MOTOR_SVPWM_PARAM  2012-09-01
{  
  s16 qKp_Gain;
//  u16 hKp_Divisor;
  s16 qKi_Gain;
//  u16 hKi_Divisor;  
  s16 qLower_Limit_Output;     //Lower Limit for Output limitation
  s16 qUpper_Limit_Output;     //Lower Limit for Output limitation
  s32 wLower_Limit_Integral;   //Lower Limit for Integral term limitation
  s32 wUpper_Limit_Integral;   //Lower Limit for Integral term limitation
  s32 wIntegral;
  s16 qKd_Gain;
//  u16 hKd_Divisor;
  s32 wPreviousError;
  s16 LPFYk_1;
} PID_Vars_Structure;
*/

/*
平衡算法
input: Anglek--kalman滤波输出Pitch角度;
output: MotorSpeed--电机速度指令信号(控制电机信号PWM脉宽);
control algorithm:  控制倾角PitchError = Pitch-PitchAngleRef = 0;
*/
//PitchBalancePID()
//{
////PID PitchBalance control: 1.Keep PitchAngle=BalanceAngle;2.同时角速度PitchRate=0.
////T = k1(θ+θ0) + k2θ' + k3(x+x0) + K4x'
////θ0--produce a specified offset -θ0; 调节θ0改变倾角的偏置
//// 
////MotorSpeedError = Kmc*PitchError;	//电机速度偏差对应倾角偏差
////P:
////I:
////D:
//
// ;
//
//}


s16 PID_Regulator(s16 hReference, s16 hPresentFeedback, PID_Vars_Structure *PID_Struct)
{//电流PID参数:qKp_Gain=4000,qKi_Gain=380, qKd_Gain=0;
  s32 wError, wProportional_Term, wIntegral_Term, wOutput_32;
  s64 dwAux; 

  s32 wDifferential_Term;
  s32 wtemp;

 // error computation
  //wError= (s32)(hReference - hPresentFeedback);	  //q14:[(-16384-16383),(16383-(-16384))]
  wError= (s32)(hPresentFeedback-hReference); 												  //[-32767,32767]
  // Proportional term computation
  wProportional_Term = PID_Struct->qKp_Gain * wError;  //s16q15(4000)*s16q14=s32q29

  // Integral term computation
  if (PID_Struct->qKi_Gain == 0)
  {
    PID_Struct->wIntegral = 0;
  }
  else
  { 
    wIntegral_Term = PID_Struct->qKi_Gain * wError;	//s16q15(4000)*s16q14=s32q29
    dwAux =(s64)(PID_Struct->wIntegral) + (s64)(wIntegral_Term);
    //积分限幅:  ~(-32768, +32767)*4000	= 131068000
    if (dwAux > PID_Struct->wUpper_Limit_Integral)  //131068000) 	  //积分限幅是否设置的太小，待查2012-09-01
    {
      PID_Struct->wIntegral = PID_Struct->wUpper_Limit_Integral;
    }
    else if (dwAux < PID_Struct->wLower_Limit_Integral)  //-131068000) //
    { 
    	PID_Struct->wIntegral = PID_Struct->wLower_Limit_Integral;
	}
	else
	{
		PID_Struct->wIntegral = (s32)(dwAux);
	}
  }

  // Differential term computation  
  if(PID_Struct->qKd_Gain == 0)
  {
  		wDifferential_Term=0;	
  }
  else
  {

//    s32 wtemp;
  
    wtemp = wError - PID_Struct->wPreviousError;
    wDifferential_Term = PID_Struct->qKd_Gain * wtemp;	  //q29
    PID_Struct->wPreviousError = wError;    // store value 
  }
  
//  wOutput_32 = (wProportional_Term/PID_Struct->qKp_Divisor+ 
//                PID_Struct->wIntegral/PID_Struct->qKi_Divisor + 
//                wDifferential_Term/PID_Struct->qKd_Divisor);
  wOutput_32 = (wProportional_Term+ 
                PID_Struct->wIntegral+ 
                wDifferential_Term);
  wOutput_32 >>= 15;			//q14

/*
低通滤波
Yk=K*Xn + (1-K)*Yk_1; fL ≈ K/(2πT)
//K=i/k; K取1/4, 1/2, 5/8, 3/4, 7/8....
*/
	
//	wOutput_32 +=(s32)(PID_Struct->LPFYk_1);// * 0x0007); //0x001F);
//	wOutput_32 >>=1;//3; //*1/8	//>>=5;	

  //输出限幅:  ~(-16384, +16383) //2012-11-09
  if (wOutput_32 >= PID_Struct->qUpper_Limit_Output)
  {
      //return(PID_Struct->qUpper_Limit_Output);
	  wOutput_32 = PID_Struct->qUpper_Limit_Output;		  			 	
  }
  else if (wOutput_32 < PID_Struct->qLower_Limit_Output)
  {
      //return(PID_Struct->qLower_Limit_Output);
	  wOutput_32 = PID_Struct->qLower_Limit_Output;
  }
//  else 
//  {
//      return((s16)(wOutput_32)); 		
//  }
/*
低通滤波
Yk=K*Xn + (1-K)*Yk_1; fL ≈ K/(2πT)
//K=i/k; K取1/4, 1/2, 5/8, 3/4, 7/8....
*/
//	PID_Struct->LPFYk_1 = (s16)(wOutput_32);   //save Yk-1

	return((s16)(wOutput_32));
}

//void checkFootSwitch(void)
//{
//
//	GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_14);
//
//}


