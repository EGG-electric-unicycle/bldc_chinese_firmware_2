 /*bldc_main.c*/

/*********************************************************************************************************
*
* File                : main.c
* Hardware Environment: kech ver 1.0
* Build Environment   : RealView Mkd-ARM  Version: 4.22
* Version             : V1.0
* By                  : lihewen
*
*                                  (c) Copyright 2012-, lihewen
*                                          All Rights Reserved
*
*********************************************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "MC_Globals.h"
/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */
/** @addtogroup TIM_7PWM_Output
  * @{
  */ 
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

//#define k1 ((s16)1)
//#define k2 ((s16)1)
#define Gyro_Data_ready_Error ((u16)0x0001)
u16 ErrorInfo=0;
float Balcommand=0,Yawcommand=0;
s16 LMotorCmd=0, RMotorCmd=0;
float pitchcmd;
s16 test_tmp;
u8 GyroAcce_state=0;
u8 Anglek_FLAG=0;
u32 tCount = 0;
u8  outputEN=0;
u8 FootSwitchStateCount=0;
u8 FootSwitchStateChange=0;
u16 whileCount=0;
static float softFactor=0;
//static float KIfactor =0.015;
u16 Vbus = 0;
u16 Vdd33 = 0;

u8 MaxAnglekProtect=1;
//u16 RIbus = 0;
//u16 LIbus = 0;
 
//static u16 RIbus_LPF=0,LIbus_LPF = 0;
 

/* extern function prototypes -----------------------------------------------*/

extern void system_init(void);	//0410//系统时钟、IO、中断向量配置及初始化//0410
extern u8 STM32_Gyro_init(void);
//extern void Read_Gyro_Data(void);
/*encoder extern function*/
//extern void TIM4_Configuration(void); //
//extern void extINT_init(void);
/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
//void GPIO_Configuration(void);
void STM32_TIM_ADC_Configuration(void);

//void svpwm_init(void);
//void parameterInit(void);
//void SVPWM_CalcDutyCycles(void); //void SVPWM_CalcDutyCycles(u8);
void uart_init(u32);
//void USART_SendByte(u8);
extern void USART1_Send(u8 *, u8);
extern void USART1_Receive(void);
extern void Get_Gyro_Value(void);
extern void Get_Acce_Value(void);
//extern void Gyro_Acce_Offset_Calibration(void);
//extern void Gyro_Acce_Offset_Calibration(s16 *);
extern void LowPassFilterK1_2(GyroAcceDataStruct *);
//extern void Gyro_Offset_Calibration(void);
extern void delay_us(u32 nus);
extern void GyroAcce_DataOutput(void);
extern void GyroAcce_DataOutput875(void);
extern void Pitch_Yaw_Calc(void);
extern s16 kalmanUpdate(const s16 , const s16);
//extern float kalmanUpdate(const s16 , const s16);

extern s16 IntegralGyro(const s16, const s16);
//extern s32 offset_correction_integral(s16);
extern s16 offset_correction_integral(s16);
//extern u16 svpwmCount;
extern void test_3DataOutput(s16, s16,s16);
extern void test_2DataOutput(s16, s16);
extern void test_1DataOutput(s16);
//void DAC_Config(void);
//void HallCfg(void);
//float Anglek=0,AngleI=0;
s16 Anglek=0;
//extern u8 LIbusHP1,LIbusHP2,RIbusHP1,RIbusHP2;
//extern u16 LIbusHPcount,RIbusHPcount;
//extern void PID_Vars_init(void);
//extern s16 PID_Regulator(s16, s16, PID_Vars_Structure *);
extern u8 printsector;
extern u16 Lsector,Rsector;
extern void Gyro_Data_ready_check(void);
extern void Beep(void);
extern void key(void);
extern void SPI2_Configuration(void);
//extern void OLED_Init(void);
//extern void OLED_ShowString(u8,u8,u8,const u8 *);
//extern void OLED_ShowNum(u8,u8,s32,u8,u8);
//extern void OLED_Refresh_Gram(void);
//extern void OLED_ShowChar(u8,s8,u8,u8,u8);
extern void Joystick_offset_Calibration(void);
extern void Joystick_Handle(void);
extern void MCU_Flash_Configuration(void);
extern void Offset_Calibration(void);
extern void UpAngleOffsetSpeedcompensation(void);
extern void L3G4200D_init(void);
s16 AkCount=0,GkCount=0,IkCount=0,JitterTestCount=0;
//float Ak=4.0,Gk=50.0,Ik=0.015;
//float Ak=0.25, Gk=0.02,Ik=0.015;   //2012-12-14 AM  Ak+/-10*0.001,Gk-17~-19*0.001,Ik+0~20*0.0005;JitterTestvalue-5~0*1
//float Ak=0.24, Gk=0.002,Ik=0.025,GAk=0.025; //GAk=0.04;   //2012-12-14 PM; Gk+/-0.0001//GAk=0.02
//float Ak=0.22, Gk=0.008,Ik=0.027,GAk=0.016; //2013-05-15
	//*float Ak=0.21, Gk=0.015,Ik=0.03,GAk=0.018; //2013-05-16
//float Ak=0.5, Gk=0.004,Ik=0.025,GAk=0.08;
//float Kpa=0.25,Kpg=0.002;
//float Kpa=0.1,Kpg=0.002;  //Kpa=0.1,Kpg=0.001;
//float Ak=0, Gk=0,Ik=0; //0.005;  //0.009;
//float ek=0;
//float ek_1=0;
float iState=0;
//float pidTerm=0;

//float pk=1.5,ik=0.17,kd=9.1;	//此处Anglek已乘0.011375, Gdata*0.009
//float pk=0.25,ik=0.005,kd=0.009,GAk=0; //2013-06-26
//float pk=0.35,ik=0.015,kd=0.016,GAk=0.07;
//float pk=0.33,ik=0.017,kd=0.013,GAk=0.06; //2013-06-28
//float pk=1.0,ik=0.02,kd=0.013,GAk=0.04; //2013-06-28调试指令反馈实验
//float pk=0.05,ik=0,kd=0;

//2013-06-29增加可调式项目后：//
	 
float pk=0.33, ik=0.017, kd=0.013, GAk=0.06; //AGpid参数
float k1 =1.0, k2 =0.02,k3 =0.04; //反馈比例参数

//extern void BlanceCalc(void);
//extern s16 Apid(void);
extern s16 AGpid(void);

u8 LIbus_Protect_Flag=0,RIbus_Protect_Flag=0;
u8 No_Gyro_Data_Count=0;
u8 LowBattFlg=0;

u16 Acc_testCount=0;
u16 Acc_RDY_Last =0, Acc_RDY=0, Acc_RDY_Flag =0;

s16 BalCmd[12]={0,0,0,0,0,0,0,0,0,0,0,0};
u8 speedkeepi=0;

s16 Upcomp=0,UpcompOffset=0,UpcompMax; //=400;

extern void IBusOffsetInit(void);

//extern void YawCalc(void);
//test:
//u16 swap_HLbyte(u16);
//void Pitch_Yaw_Calc(void);
/* Private functions ---------------------------------------------------------*/
/**
  * @brief   Main program
  * @param  None
  * @retval None
  */
int main(void)
{
 u8 tab[10] ={'U','A','R','T',0x20,'O','K','!',0x0D,0x0A};	//0x0D=\r;0x0A=\n
 u8 static uart_Tx_count = 0;//,No_Gyro_Data_Count =0
 //s32 tmp;
 float tmp1,tmp2,temp5;
 static float tmp3=0,tmp4=0,tmp4_LPF=0;
// u8 i; //opt;
 static u16 Vbusdisplay=0;
 s16 s16testdata;

// static s16 LIbusMax=0, RIbusMax=0;
// s16 LI,RI;
// static s16 LrpmMax=0,RrpmMax=0;
// static s16 AnglekMax=0;
// static s16 UpAngleOffsetMax=0;
// s16  s16Anglek;

 static u16 Gyro_Count=0;

 u8 i;
 
 
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */ 
/*Step1.1111111111111111111111111111111111111111111111111111111111111111111111111111111111*/
// POWER ON INIT: 
   /* System Clocks Configuration */
   /* GPIO Configuration */
   /* NVIC Configuration*/
   system_init();  //0410 //时钟\IO\中断向量初始化 
     
//  /* System Clocks Configuration */
//  RCC_Configuration();
//  /* GPIO Configuration */

  /*SPI Configuration */
  //spi_init();
  SPI2_Configuration();

  /*MCU_Flash_Config */
  MCU_Flash_Configuration();

  
 
/*Step2.22222222222222222222222222222222222222222222222222222222222222222222222222222222222*/
  
 
  /* TIMER &ADC Configuration */
  STM32_TIM_ADC_Configuration();
  
  //offset init:
  //Offset_init(); //计算Acc, Gyro, ICS, ADC 0偏置值;待完成2012-09-01
//  HallCfg();  //setup before STM32_TIM_ADC_Configuration  
  /*encoder extern function*/
///*OLED Configuration */
// SPI2_Configuration(); //xx0410
/*OLED welcome information */ //OLEDwelcome();   //include progress bar
// OLED_Init();
// OLED_ShowString(32,16,16,"WELCOME!");
// OLED_ShowString(32,16,16,"WELCOME!");
//
// for(i=0;i<128;i++)
// {
//  OLED_Refresh_Gram();
// }
/*Step3.3333333333333333333333333333333333333333333333333333333333333333333333333333333333333*/
//Self-checking & Show Results ........Self-checking of All key items must be Done! 
  
/*UART1 test：*/
 uart_init(Uart_Brr);
 delay_us(1000);
 USART1_Send(tab, 10);
 delay_us(5000);
 USART1_Send("WELCOME!\r\n", 10);
 delay_us(5000);
  
  CPU_LED_ON;	//CPU_LED
//Steering_offset_Calibration; //初始化方向位置
// Joystick_offset_Calibration();  //get Joystick_offset

 delay_us(200000); //1s改为0.2s
 /**/
  GyroAcce_state = STM32_Gyro_init();	 //test0422
  if(GyroAcce_state==3)
  {
   //OLED_ShowString(16,32,16,"MEMS Init OK!");
   USART1_Send("MEMS Init OK!\r\n", 15);
   delay_us(5000); //每ms最多发8个Bytes	   
  }
  else
  {
//	   OLED_ShowString(16,32,16,"MEMS Init");
//	   OLED_ShowString(40,48,16,"ERROR!");
	   USART1_Send("MEMS Init ERROR!\r\n", 18);
	   delay_us(5000);
  }
// for(i=0;i<128;i++)
// {
//  OLED_Refresh_Gram();
// }
 while(GyroAcce_state!=3)	//test0422
  delay_us(2000000);
//  Offset_Calibration_Flag=1;
//  Gyro_Acce_Offset_Calibration();
//  Offset_Calibration_Flag=0;
  LED1_ON;
  LED2_ON;
  LED3_ON;
  LED4_ON;
  LED5_ON;
  LED6_ON;
  LED7_ON;
  LED8_ON;

  BEEP_ON;
  delay_us(100000);
  BEEP_OFF;
  Offset_Calibration();

  LED5_OFF;
  LED6_OFF;
  LED7_OFF;
  LED8_OFF;

 
//  PID_Vars_init(); 
  CPU_LED_ON;   //CPU_LED
  //PWM_SD
  //MOT_SD_OFF;  //Lmotor SD off
//  RU_SD_OFF;
//	RV_SD_OFF;
//	RW_SD_OFF;
//	LU_SD_OFF;
//	LV_SD_OFF;
//	LW_SD_OFF;
  //GPIO_ResetBits(GPIOE, GPIO_Pin_2);  //Rmotor SD off
//  OLED_Init();
//LIbus_Protect_Flag=0;  //0410 清除稳定之前的电流保护值
//RIbus_Protect_Flag=0;

IBusOffsetInit();
//delay_us(1000);
//LIbus_Protect_Flag=0;  //0410 清除稳定之前的电流保护值
//RIbus_Protect_Flag=0;

while (1)
{

 whileCount++;
 if((whileCount==0x0000)&&(TestKeyNo==0x0))
 {
//  GPIO_ResetBits(GPIOF, GPIO_Pin_14);  //oled复位
  LED4_ON;
  if(LowBattFlg)
  	LED1_ON;
  if(LIbus_Protect_Flag)
  	LED3_ON;
  if(RIbus_Protect_Flag)
  	LED2_ON;
  
 }
 else if((whileCount==0x7FFF)&&(TestKeyNo==0x0))
 {
//   GPIO_SetBits(GPIOF, GPIO_Pin_14);
//   OLED_Init();
   LED4_OFF;
   if(LowBattFlg)
    LED1_OFF;
   else
	LED1_ON;
   if(LIbus_Protect_Flag)
  	LED3_OFF;
   else
  	LED3_ON;
   if(RIbus_Protect_Flag)
  	LED2_OFF;
   else
  	LED2_ON;
 }

// ADC_SoftwareStartConvCmd(ADC1, ENABLE);
// ADC_SoftwareStartConvCmd(ADC2, ENABLE);
 ADC_SoftwareStartConvCmd(ADC3, ENABLE);
  
 if(outputEN==0)   //SD输出延迟
 {
  tCount++;
  if(tCount>1000)
  {
   outputEN=1;
   //MOT_SD_ON;   //motor ON,SD=0
//	RU_SD_ON;
//	RV_SD_ON;
//	RW_SD_ON;
//	LU_SD_ON;
//	LV_SD_ON;
//	LW_SD_ON;
	MaxAnglekProtect=0;	 //开启sd
	TIM_Cmd(TIM1, ENABLE);           //TIM1开启
    TIM_Cmd(TIM8, ENABLE);           //TIM8开启
   //GPIO_SetBits(GPIOE, GPIO_Pin_2);   //Rmotor ON SD=0  
  }
  CPU_LED_ON;   //LED
 }
 
 
 FootSwitchState=FSW_Read_L1;
 FootSwitchState <<=1;
 FootSwitchState |=FSW_Read_L2;
 FootSwitchState <<=1;
 FootSwitchState |=FSW_Read_R1;
 FootSwitchState <<=1;
 FootSwitchState |=FSW_Read_R2;	
 
  if((FootSwitchState & 0x0C)==0x0C)	 //脚踏开关指示
 {
  //OLED_ShowChar(64,0,'A',16,1);
//  OLED_ShowString(64,0,16,"OFF OFF");
  LED6_OFF;
 }
 else
 	LED6_ON;

 if((FootSwitchState & 0x03)==0x03)
 {
//  OLED_ShowString(64,0,16,"OFF ON ");
  LED5_OFF;
 }
 else
 	LED5_ON;



 USART1_Receive();
 Beep();
 key();

// OLED_ShowNum(0,0,testkeydata,3,16);
// if(Redrawi>=0x80)

  if((whileCount&0x0FFF)==0x0000)    //3sec reflash vbus display
  {
    Vbusdisplay = Vbus/20; //
    if(Vbusdisplay<=75)	//低电压告警！
    {
   		//Beep();
		//LED4_OFF;
		//LED3_OFF;
		//LED2_OFF;
		LowBattFlg=1;
	}
	else if(TestKeyNo==0x0)
	{
		LowBattFlg=0;
		if(Vbusdisplay>=79)
		{
		//	LED4_ON;
		//	LED3_ON;
		//	LED2_ON;
			LED1_ON;
		}
		else if(Vbusdisplay>=78)
		{
		//	LED4_OFF;
		//	LED3_ON;
		//	LED2_ON;
			LED1_ON;
		}
		else if(Vbusdisplay>=77)
		{
			//LED4_OFF;
			//LED3_OFF;
			//LED2_ON;
			LED1_ON;
		}
		else if(Vbusdisplay>=76)
		{
			//LED4_OFF;
			//LED3_OFF;
			//LED2_OFF;
			LED1_ON;
		}
	}
  }
//  if(Lrpm>LrpmMax)
//  	LrpmMax=Lrpm;
//  if(Rrpm>RrpmMax)
//  	RrpmMax=Rrpm;
 {    //x,y
//  OLED_ShowNum(0,0,Vbusdisplay,4,16); //32);
//  OLED_ShowNum(0,16,AkCount,4,16); //32);
//  OLED_ShowNum(0,32,GkCount,4,16); //32);
//  OLED_ShowNum(0,48,IkCount,4,16); //32);
//  OLED_ShowNum(0,16,AnglekMax,4,16); //32);	//显示最大倾角
	//s16Anglek =(s16)Anglek;
	//s16Anglek >>=2;		
//  OLED_ShowNum(0,16,pAcceData -> x_Data,4,16); //test:显示加速度原始数据(倾角)
//  OLED_ShowNum(0,16,s16Anglek,4,16); //32);	//显示倾角
//  OLED_ShowNum(0,32,LrpmMax,4,16); //32);//显示最大速度值
//  OLED_ShowNum(0,48,RrpmMax,4,16); //32)
  //OLED_ShowNum(64,0,Joystick_data,4,16);//
  //OLED_ShowNum(64,0,Joystick_Value,4,16); //
//  OLED_ShowNum(64,32,JitterTestCount,4,16); //32);
  //OLED_ShowNum(64,16,ErrorInfo,4,16);
  //OLED_ShowNum(64,16,JitterTestCount,4,16);
  //if(UpAngleOffset>UpAngleOffsetMax)
  //	UpAngleOffsetMax=UpAngleOffset;
//   OLED_ShowNum(64,16,UpAngleOffsetMax>>3,4,16);
//  Redrawi = 0;
 }


//test最大电流显示:
// LI=LIbus_LPF>>3; 
// RI=RIbus_LPF>>3;
// if(LI>LIbusMax)
// 	LIbusMax=LI;
// if(RI>RIbusMax)
// 	RIbusMax=RI;

// OLED_ShowNum(64,32,(LIbusMax),4,16); //左母线电流显示
// OLED_ShowNum(64,48,(RIbusMax),4,16); //右母线电流显示
// OLED_ShowChar(64,32,'A',16,1);
 
// if(Offset_Calibration_Flag)
//   Gyro_Acce_Offset_Calibration();
 
	//test Acc：//0410
	Acc_RDY=Get_Acce_RDY;
	if(Acc_RDY!=Acc_RDY_Last)
	{
		Acc_testCount++;
		if(Acc_testCount>=100)
		{
			Acc_RDY_Flag++;
			Acc_testCount=0;
			if(Acc_RDY_Flag&0x0001)
			{
				//LED4_ON;
				LED8_ON;
			}
			else
			{
				//LED4_OFF;
				LED8_OFF;
			}
		}
	}
	Acc_RDY_Last=Acc_RDY;

 Gyro_Data_ready_check();
 
// Gyro_Data_ready=0; //test2013-04-07  
 if(Gyro_Data_ready)
 {
 	Gyro_Count++;	 //信号检测指示
	if(Gyro_Count==100)
		LED7_ON;
	if(Gyro_Count>=200)
	{
		LED7_OFF;
		Gyro_Count=0;
	}
	   
    
  UpAngleOffsetSpeedcompensation(); //角度补偿计算
  Get_Gyro_Value();
  Get_Acce_Value();
  //LowPassFilterK1_2(pGyroData);
  //LowPassFilterK1_2(pAcceData);
  //Gyro_Acce_process();
  Pitch_Yaw_Calc();    //执行时间1.833us
  //Gyro_Data_ready = 0;
//  pGyroData -> xdatapu875 >>=4;  //2;//5;
//  pAcceData -> xdatapu875 >>=4;   //2;//5; 
  Anglek = kalmanUpdate(pGyroData -> xdatapu875,pAcceData -> xdatapu875);   //执行时间7.278us



  //if(Anglek>AnglekMax)	 //保存最大倾角
  //	AnglekMax = Anglek;

  /*角度保护；暂定30° ,保护执行后，须要关机开机重新启动才能恢复正常使用*/
//  if((Anglek>760)||(Anglek<-760))	//保护角度设置错误
  //if((Anglek>3430)||(Anglek<-3430))	//+/-30°
//  if((Anglek>2285)||(Anglek<-2285))	//+40°，-40° jysPCB //test0422
  if((Anglek>4580)||(Anglek<-4580))	//+40°，-40° jysPCB //0425
  {
  	//PWM_SD
  	//MOT_SD_OFF;  //motor SD off
//	RU_SD_OFF;
//	RV_SD_OFF;
//	RW_SD_OFF;
//	LU_SD_OFF;
//	LV_SD_OFF;
//	LW_SD_OFF;
	MaxAnglekProtect=1;  	
  }
  //Anglek = kalmanUpdate(pGyroData -> xdatapu875,pAcceData -> ydatapu875);  //kalman for ext.board  
  //Anglek = IntegralGyro(pGyroData -> xdatapu875,pAcceData -> xdatapu875);
  //AngleI=(s16)offset_correction_integral(pGyroData -> xdatapu875);
  //AngleI=(s16)offset_correction_integral(Anglek);
//  Anglek >>=2;
//  pitchcmd = pGyroData -> xdatapu875>>2;
  //pitchcmd = 8*Anglek + pGyroData -> xdatapu875; //>>6; //k2*pGyroData -> xdatapu875;
//  Balcommand = 8*Anglek + pGyroData -> xdatapu875>>2;    //卡
//  Balcommand = 8*(Anglek+(s16)offset) + pGyroData -> xdatapu875>>2 + AngleI;    //
//  pitchcmd = 5*Anglek; + pGyroData -> xdatapu875/4;  //太大烧管
//  tmp1=Anglek/Ak;//3; //2;/>>4;///10;    //30°对应3428  
//  tmp2=pGyroData -> xdatapu875/Gk; //256;///192;//128;>>8;
  //UpAngleOffset=0; //test:屏蔽上翘
  //*tmp1=(Anglek+(float)UpAngleOffset)*Ak;//(kalman角度+上翘角度)*角度比例系数
//  tmp1=Anglek*Ak;//屏蔽上翘


  //tmp2=pGyroData -> xdatapu875;  //*Gk; //0.02;
  //tmp2 = offset_correction_integral(pGyroData -> xdatapu875);
//  s16testdata = offset_correction_integral(pGyroData -> xdatapu875);
//  tmp2 = s16testdata;
  //tmp3=tmp2;

  //增加微分项,微分项增加在积分项内或是增加在之外,待实验决定!
  //原陀螺仪微分项计算前已乘Gk系数，会降低微分效果；
  //修改为计算前不乘Gk系数，需修改GAk=0.04为GAk=0.04?
  //*tmp1 += (tmp2-tmp3)*GAk; //微分项增加在积分项内；+微分项*微分系数
//--  tmp1 = (tmp2-tmp3)*0.01;	 //2013-06-13
  //tmp2 += tmp3*3/4;    //GyroData LPF
//  tmp2 += tmp3*3;
//  tmp2 /=4;
//  tmp3 = tmp2;

  //tmp2 *=GAk; //kd; //0.009; //GAk;  //0.0105;  //1.3:1.2*8.75/1000

//  ek_1=ek;
//  ek=Anglek+(float)UpAngleOffset;  //

  //起步前倾补偿方案五：
//  if(LRrpm>800) //>500)	  //前加速补偿； 500转向会启动上翘
//  {
//  	tmp = LPF_AcceLRrpm*GAk;
//	Upcomp =(s16)tmp;
//	if(Upcomp<UpcompMax)
//		Upcomp=UpcompMax;  //保持最高补偿值
//	else
//		UpcompMax=Upcomp;  //保存最高补偿值
//
//	if(LPF_AcceLRrpm<-10)  //减速门限
//	{
//		UpcompMax=0;
//		if(Upcomp>10)  //错误!修正
//			Upcomp -=10;
//		else if(Upcomp>0)
//			Upcomp=0;
//	}
//  }
//  else if(LRrpm<-800)
//  {
//  	tmp = LPF_AcceLRrpm*GAk;
//	Upcomp =(s16)tmp;
//	if(Upcomp>UpcompMax)
//		Upcomp=UpcompMax;  //保持最高补偿值
//	else
//		UpcompMax=Upcomp;  //保存最高补偿值
//
//	if(LPF_AcceLRrpm>10)  //减速门限
//	{
//		UpcompMax=0;
//		if(Upcomp<-10)  //错误!修正
//			Upcomp +=10;
//		else if(Upcomp<0)
//			Upcomp=0;
//	}
//  }
//  else if(((LRrpm<400)&&(LRrpm>-400)))
//  {
//  	UpcompMax=0;
//  	if(Upcomp>0)  //错误!修正
//		Upcomp --;
//	else if(Upcomp<0)
//		Upcomp++;
//  }
//
//  if(Upcomp>600)		 //最值限制
//		Upcomp=600;
//	else if(Upcomp<-600)
//		Upcomp=-600;
  //起步前倾补偿方案五end
//
//  if((LRrpm>1300)||(LRrpm<-1300))	//起步前倾补偿方案四，加速度补偿
//  {									//1300,起始补偿速度为2.55km/h,是否偏高?
//  	tmp = LPF_AcceLRrpm*GAk;
//	Upcomp =(s16)tmp;
//	
//	if(Upcomp>600)		 //最值限制
//		Upcomp=600;
//	else if(Upcomp<-600)
//		Upcomp=-600;
//  }
//  else
//  {
//  	//Upcomp=0;
//	//if(tmp>10)   错误!
//	if(Upcomp>10)  //错误!修正
//		Upcomp -=10;
//	else if(Upcomp>0)
//		Upcomp=0;
//	if(Upcomp<-10)
//		Upcomp +=10;
//	else if(Upcomp<0)
//		Upcomp=0;
//  }

  //ek += (float)Upcomp;

//  if(ek>50)					  //起步前倾补偿方案三
//  {
//  	if((upLRCC>120)&&(upLRCC<320))
//	{
//		if(Upcomp<600)
//		Upcomp+=10;
//	}
//	else if(Upcomp>10)
//		Upcomp-=10;
//  }
//  else if(ek<-50)
//  {
//  	if((upLRCC<-120)&&(upLRCC>-320))
//	{
//		if(Upcomp>-600)
//		Upcomp-=10;
//	}
//	else if(Upcomp<-10)
//		Upcomp+=10;
//  }
  
//  ek += (float)Upcomp*GAk;	   //起步前倾补偿方案三end

//  ek += (float)Upcomp*GAk;	 //起步前倾补偿方案二
//  ek +=UpcompOffset;
//  if(ek>UpcompOffset)	  //起步前倾补偿方案一
//  	Upcomp++;
//  else if((ek>0)&&(Upcomp>1))
//  	Upcomp--;
//  if(ek<-UpcompOffset)
//  	Upcomp--;
//  else if((ek<0)&&(Upcomp<-1))
//  	Upcomp++;
//  if(Upcomp>UpcompMax)
//  	Upcomp=UpcompMax;
//  else if(Upcomp<-UpcompMax)
//  	Upcomp=-UpcompMax;
//
//  ek +=Upcomp;	//起步前倾补偿方案一end

  //ek*=0.011375;	 //倾角误差 //1.3:1.2*8.75/1000
  //ek+=tmp2;		 //角速度误差
//  ek+=tmp1;      //角加速度误差//1.3:1.2*8.75/1000
  //Kpa=0.25,Kpg=0.002;
  //temp5=(Anglek+(float)UpAngleOffset)*Kpa + tmp3*Kpg;	//增加Kp项
//   if(tmp3>6000) //抗猛压-角速度>6000*8.75/1000=52.5dps
//  {
//  	Gk=0.004;
//  }
//  else
//  {
//  	Gk=0.002;
//  }
//  tmp2 +=tmp1;
//  tmp3 *=3;
//  tmp3 +=tmp2;
//  tmp3 /=4;
  //*tmp1 +=tmp3*Gk;  //陀螺仪系数移到此处
//  tmp4 += tmp3*Ik;
  //*tmp4 += tmp1*Ik;	 //对(kalman角度*Ak+陀螺仪*Gk+陀螺仪微分*GAk)积分
  //0426，这里需要增加角度、角速度比例项?，待研究PID及实验:
  //或者加到滤波之后?
  //BlanceCalc();
  //tmp4=pidTerm; //速度增加会倒
//  tmp4=Apid();
//  tmp4+=tmp2;
//  tmp4 += GyroData_Self_offset*kd;
  //tmp4 +=pidTerm;	 //难调整，是否收敛待分析
  tmp4=AGpid();
  //tmp4_LPF *= 3;   //2012-12-13
  tmp4_LPF += tmp4;
//  //*tmp4_LPF += tmp1;
  tmp4_LPF /=2;  //4;  //tmp4_LPF = LPF[Anglek*Ak+LPF(pGyroData -> xdatapu875*Gk)+∑(Anglek*Ak+LPF(pGyroData -> xdatapu875*Gk))*Ik]
  	//tmp4_LPF = tmp4;
   //0.015;  //累积为X' //0.0047,0.002,0.006,0.01,0.2
  
  ////区间判断,脚踏开关释放有急停,增加下列软启,无效!!2012-12-10
  //tmp4 *= softFactor;
  tmp4_LPF *= softFactor;
  //tmp3 *= softFactor;
  iState *=	softFactor;

  //temp5 += tmp4_LPF;
  temp5 = tmp4_LPF;
//  temp5 *= softFactor;

  if(temp5>10000) //数据类型转换前保护
  {
     temp5=10000;
  }
  else if(temp5<-10000)
  {
  	 temp5=-10000;
  }
//  pitchcmd = (s16)(tmp1 + tmp2 + tmp4_LPF);
  //pitchcmd = (s16)temp5;  //tmp4_LPF; //(tmp3 + tmp4_LPF);
  for(i=0;i<11;i++)
  {
  	BalCmd[i]=BalCmd[i+1];
  }
  //BalCmd[11]=(s16)temp5;
  //Y=k1*X(k)+k2*Y(k-1)
  
  if(speedkeepi>7) //调试延迟时间
  	{speedkeepi=0;}
  pitchcmd=(BalCmd[speedkeepi]+BalCmd[speedkeepi+1]+BalCmd[speedkeepi+2]+BalCmd[speedkeepi+3]);
  pitchcmd *= k2;   //Y=k2*Y(k-1)

  pitchcmd += temp5*k1;   //Y+=k1*X(k)   
  
  //pitchcmd = temp5 +((BalCmd[speedkeepi]+BalCmd[speedkeepi+1]+BalCmd[speedkeepi+2]+BalCmd[speedkeepi+3])>>2);
//--//2013-06-13  pitchcmd = BalCmd[11] +((BalCmd[speedkeepi]+BalCmd[speedkeepi+1]+BalCmd[speedkeepi+2]+BalCmd[speedkeepi+3])>>2);
  //BalCmd[11]=(s16)pitchcmd; //振荡但可收敛,需进一步研究
  //temp5 = LPF_LRrpm*0.03; //GAk; //0.03;	//2013-06-13
  //pitchcmd =(s16)temp5;		//2013-06-13
  //pitchcmd += BalCmd[11];	//2013-06-13
  //pitchcmd = BalCmd[11];

  pitchcmd +=BalCmd[11];  //Balcommand;2013-06-28测试缓停有起伏问题，29日修改 //*3; //LPF
  pitchcmd /=2;  //4;		   //增加滤波后速度死区引起的抖动减小！2013-06-14

  BalCmd[11]=(s16)pitchcmd;	 //保存Y(k)

  tmp3 +=LPF_LRrpm*k3;		 //增加速度保持并滤波
  tmp3 +=pitchcmd;
  tmp3 /=2;
  Balcommand = tmp3;
  //Balcommand = pitchcmd;//PID_Regulator(0, pitchcmd, pPitchPID);//+pGyroData -> xdatapu875>>2;
  //Balcommand = -pitchcmd;//for ext Gyro board
//  Balcommand = Anglek + pGyroData -> xdatapu875>>5;
  //Balcommand = AngleI>>2; //PID_Regulator(pitchcmd, 0, pPitchPID);
  Yawcommand = Joystick_Value;//JoystickRateSet*3; //PID_Regulator(s16 hReference, s16 hPresentFeedback, pPitchPID);

  Yawcommand *= softFactor;
//  Anglek_FLAG = 0;
eRPM_LPF=0;	  //屏蔽转向补偿测试test2013-05-30
  if(outputEN==1)
  {
   if(FootSwitchState<0x0F)
   {
    if(softFactor<=0.994)
     softFactor += 0.006; //1.5sce//0.005; //1sec
	if(softFactor>=0.994)
     softFactor = 1.0; 

//    while(Anglek_FLAG); //避免中断指令执行时修改电机指令数据
							//这2个等待指令有时造成电机指令不能赋新值
//    if((LIbus_Protect_Flag)||(RIbus_Protect_Flag))
//    {
//     softFactor *=0.95;
//    }
  
    tmp1 = (Balcommand+Yawcommand+eRPM_LPF)*softFactor;
    tmp2 = (Balcommand-Yawcommand-eRPM_LPF)*softFactor;
    //LMotorCmd=Balcommand-Yawcommand; 
    //RMotorCmd=Balcommand+Yawcommand;
	if(Anglek_FLAG==0)
	{
    	LMotorCmd = (s16)tmp1; 
    	RMotorCmd = (s16)tmp2;
	}
    FootSwitchStateCount=0;
   }
   else 
   {
   
    if(softFactor>=0.002)
     softFactor -= 0.002; //1.5sce//0.005; //1sec
	if(softFactor<=0.002)
     softFactor = 0; 

//    while(Anglek_FLAG);	 //这2个等待指令有时造成电机指令不能赋新值
    tmp1 = (Balcommand+Yawcommand+eRPM_LPF)*softFactor;
    tmp2 = (Balcommand-Yawcommand-eRPM_LPF)*softFactor;
	if(Anglek_FLAG==0)
	{
	    LMotorCmd = (s16)tmp1; 
	    RMotorCmd = (s16)tmp2;
	}
         //区间判断,脚踏开关释放有急停!!2012-12-10
    if(softFactor==0)  //((softFactor>-0.003)||(softFactor<0.003))
    {
      tmp4=0;//reset
      tmp4_LPF=0;
//      Anglek=0;
//      softFactor=0;
      //tmp3=0;
	  UpAngleOffset=0;

	  //eRPM_Integ=0; 
	  //eRPM_LPF=0;
      
    } 
    
   }
 
    
 
  }
  else	//Not outputEN==1
  {
    if(Anglek_FLAG==0)
	{  
	   LMotorCmd=0; 
	   RMotorCmd=0;
    }
    tmp4=0;//reset
    tmp4_LPF=0; 
    //tmp3=0;
	UpAngleOffset=0; 
	eRPM_Integ=0; 
	eRPM_LPF=0; 
  }  
  Anglek_FLAG = 1; 
  //debug output:
  //pGyroData -> xdatapu875 = (s16)offset_correction_integral(pGyroData -> xdatapu875);
  //pGyroData -> xdatapu875 = (s16)offset_correction_integral(pGyroData -> ydatapu875);
  //pGyroData -> xdatapu875 = (s16)offset_correction_integral(pGyroData -> zdatapu875);
  //pGyroData -> xdatapu875 = kalmanUpdate(pGyroData -> xdatapu875,0);
  uart_Tx_count++;   //
  if(GyroAcce_DataOutput_Flag)
  {
   if(uart_Tx_count==1) //每10组数据输出1组
   {
    //GyroAcce_DataOutput(); //max output 8 bytes per ms, output completed 1 set data need 6ms
    //GyroAcce_DataOutput875();
    //test_1DataOutput((s16)Balcommand);
	//test_2DataOutput(LIbus, RIbus);
	//test_2DataOutput(LIbus_LPF,RIbus_LPF);
	//test_2DataOutput(LIbus_LPF,Lsector);
	//test_2DataOutput(pGyroData -> xdatapu875,s16testdata);
	test_2DataOutput(pGyroData -> xdatapu875,pGyroData-> x_LPF_Previous);
	
    uart_Tx_count = 0;
    //Gyro对应uart输出的校准值测试:
    //Gyro_Offset_Calibration();
	
   }
   ;
   
  }
  Gyro_Data_ready = 0;
  No_Gyro_Data_Count=0;
  //test_2DataOutput(LiA_sensor,LiB_sensor);
  //test_2DataOutput(LIbus,RIbus);
  //test_2DataOutput(LIbus_LPF,RIbus_LPF);
  //test_3DataOutput(AkCount,GkCount,IkCount);
  //test_1DataOutput(Joystick_data);
//  test_2DataOutput(Joystick_data,Joystick_Value);
//  test_2DataOutput(5,55);
 }
 else  //Not Gyro_Data_ready
 {
//  if(No_Gyro_Data_Count<200)  //40ms count
//   No_Gyro_Data_Count++;
//  if(No_Gyro_Data_Count>=160)
//  {
////  	Get_Gyro_Value();
////  	Get_Acce_Value();
//	L3G4200D_init();
//  }
  if(Get_Acce_RDY ==1)
  {
  	Get_Acce_Value();
  }
  if(No_Gyro_Data_Count>=200)  //
  {
  	if(LMotorCmd>0)	 //Gyro_Data出错情况下关闭电机
	{LMotorCmd--;}
	if(LMotorCmd<0)
	{LMotorCmd++;}
	if(RMotorCmd>0)
	{RMotorCmd--;}
	if(RMotorCmd<0)
	{RMotorCmd++;}

//	   LMotorCmd=0;
//	   RMotorCmd=0;
	   softFactor=0;
	   ErrorInfo |=Gyro_Data_ready_Error;
	   Anglek_FLAG = 1;
	   No_Gyro_Data_Count=120;
  }
 }    
 //delay_us(5000);
// if(GyroAcce_DataOutput_Flag)
// {
//  GyroAcce_DataOutput();
// }
 
// if((TimUpdateCount%40)==0)
// {
//  test_2DataOutput(LMparam.I_bus, RMparam.I_bus);
// }
 //左右电机硬过流test
// LIbusHP2=GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_9);  //左电机硬过流
// RIbusHP2=GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_3);  //右电机硬过流
//
// if(LIbusHP1 !=LIbusHP2)
//// if(LIbusHP2)
// {
//  LIbusHPcount++;
// }
//
// if(RIbusHP1 !=RIbusHP2)
//// if(RIbusHP2)
// {
//  RIbusHPcount++;
// }
//
// LIbusHP1 =LIbusHP2;
// RIbusHP1 =RIbusHP2;
 //test:
// if(printsector==1)
// {
//  //test_1DataOutput(Lsector);
//  //test_1DataOutput(Rsector);
//  test_2DataOutput(Lsector,Lrpm);
//  
//  printsector=0;
// }
// for(z=0;z<128;z++)
 Vdd33 = (u16)(ADC3_Buffer[0]);
 Vdd33 &= 0x0FFF;
 Vbus =  (u16)(ADC3_Buffer[1]);
 Vbus &=  0x0FFF;

 Joystick_data = (s16)(ADC3_Buffer[4])-Joystick_offset;
 Joystick_Handle();
// YawCalc();

// RIbus =  (u16)(ADC1->DR);
// RIbus &= 0x0FFF;
// LIbus = (u16)(ADC1->DR>>16);
// LIbus &= 0x0FFF;
// LIbus_LPF *=15;
// LIbus_LPF +=LIbus;
// LIbus_LPF >>=4;
// if(LIbus_LPF>1920) //24A 1600) //20A保护 >2400)  //30A 
// {
//  LIbus_Protect_Flag=1;
// }
//// else if(LIbus_LPF<800)
//// {
////  LIbus_Protect_Flag=0;  //10A保护解除
//// }
// RIbus_LPF *=15;
// RIbus_LPF +=RIbus;
// RIbus_LPF >>=4;
// if(RIbus_LPF> 1600) //20A保护 2400)  //30A 1920) //24A
// {
//  RIbus_Protect_Flag=1;
// }
//// else if(RIbus_LPF<800)
//// {
////  RIbus_Protect_Flag=0; //10A保护解除
//// }
// 
//    /*电流保护,保护执行后，须要关机开机重新启动才能恢复正常使用*/
//
// if(RIbus_Protect_Flag|LIbus_Protect_Flag)	//电流保护告警！
// {
// 	//PWM_SD
////  	MOT_SD_OFF;  //Lmotor SD off
////  	GPIO_ResetBits(GPIOE, GPIO_Pin_2);  //Rmotor SD off
// 	Beep();
// }

// OLED_Refresh_Gram();
  } //end while

} //end main
 

/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval None
  */
void RCC_Configuration(void)
{
// SetSysClockTo72(); //lihewen 2012-07-31 test, this function can not be called at here?
   /* PCLK1 = HCLK/4 */
//   RCC_PCLK1Config(RCC_HCLK_Div8);
//   RCC_PCLK2Config(RCC_HCLK_Div8);
  /* TIM1, GPIOA, GPIOB, GPIOE and AFIO clocks enable */ //lihewen  | RCC_APB2Periph_GPIOC
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOE|\
                         RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
}

//
//#define STM32F10X_CL
/*
TIM1--GPIOE:  
Pin8  = UL
Pin9  = UH
Pin10 = VL
Pin11 = VH
Pin12 = WL
Pin13 = WH
*/
#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  while (1)
  {}
}
#endif

//s16 Apid(void)
//{
//// float static sttmp1=0,sttmp2=0;
// float ftmp;
// s16 tmp;
//
//  tmp =Anglek+UpAngleOffset;   //Aek
//  if((tmp<10)&&(tmp>-10))
//  	tmp=0;
//  
//  iState += tmp*ik;			   //Aik
//
//  if(iState>1000)
//   iState=1000;
//  else if(iState<-1000)
//   iState=-1000;
//
//
////  sttmp2 *=3;
////  sttmp2 += (Anglek-sttmp1)*kd;   //dTerm
////  sttmp2 /=4;					   //dTerm LPF
////  sttmp1 = Anglek;
//
//  ftmp = tmp*pk;	   //A pTerm
////  tmp += sttmp2;   //A dTerm LPF
//  ftmp += iState;   //A iTerm
//  return (s16)ftmp;
//}

s16 AGpid(void)
{	
//pk=0.33,ik=0.017,kd=0.013,GAk=0.06;
	float ftmp;
 	s16 tmp;
	s32 s32tmp;
	static s16 sstmp,GdLpf;
	//Anglek	//integral 
	tmp =Anglek+UpAngleOffset;   //Aek
	
//  	if((tmp<5)&&(tmp>-5))
//  		tmp=0;
  	if(tmp>5)			//2013-06-28
		tmp -=5;
	else if(tmp<-5)
		tmp +=5;
	else
		tmp -=0;

  	iState += tmp*ik; //Aik

  	if(iState>1000)
   		iState=1000;
  	else if(iState<-1000)
   		iState=-1000;
	//Anglek	//proportion
	ftmp = tmp*pk;
	ftmp += iState;
	
	//pGyroData -> xdatapu875  //differential+proportion
	s32tmp =sstmp*3+pGyroData -> xdatapu875; //LPF
	s32tmp /=4;
	tmp =(s16)s32tmp;
	ftmp +=tmp*kd;

	//pGyroData  //differential
	s32tmp =GdLpf+(tmp-sstmp); //LPF
	s32tmp /=2;				   //2013-06-28改4阶为2阶
	GdLpf =	(s16)s32tmp;
	//ftmp +=(tmp-sstmp)*GAk;
	ftmp +=GdLpf*GAk;
	sstmp=tmp; //save LPF
	
	return ((s16)ftmp);

}

//void BlanceCalc(void)
//{
//	iState +=ek;	//积分
//	if(iState>6000)  //1250)//pwm最大调制脉宽的50%暂定*限定值=95%duty值/ik
//		iState=6000;	//速度增加会倒可能由于该限定值太小
//	else if(iState<-6000)  //-1000)	//6000=1020/0.17
//		iState = -6000;  //-1000;
//	
//	pidTerm =iState * ik;	  //iTerm	  
//	pidTerm += (ek-ek_1)*kd;  //dTerm
//	pidTerm += ek*pk;		  //pTerm
//}

///////////////////////////////////////////
//ek*=0.011375;	tmp2 *=0.009;
//float pk=1.0,ik=0.10,kd=10.5;
//相当于:
//ek=A*0.0113758+G*0.009;
//pTerm=ek*pk=A*0.0113758+G*0.009;
//iTerm +=ek*ik=(A*0.0113758+G*0.009)*0.1;
//      +=A*0.00113758+G*0.0009;
//-->:ikA=pk*0.0047/TiA=0.00113758-->TiA= 4.1316s
//-->:ikG=pk*0.0047/TiG=0.0009 -->TiG=5.2s
//-->:G积分 +0.0047*G ==A;
//-->:G积分 +=G*0.0009=0.19A;
//-->:pTermA==A*0.0113758+0.19A=0.20A;

//pTermG=G*0.009+d(A*0.0113758)*kd=G*0.009+dA*0.1194459;
//-->: pTermG=0.214*G;
//-->: dpTermG=d(G*0.009)*10.5=dG*dG*0.0945;

//iTermA +=ekA*ik=A*0.0113758*0.10;
//-->: iTermA +=A*0.00113758;

//dTerm =dek*kd=d(A*0.0113758+G*0.009)*10.5=dA*0.1194459+dG*0.0945;

//Ak=A*0.0113758*(pk=1.0)=A*0.0113758;
//Gk=G*0.009;+dA*0.0113758*kd=G*0.009+dA*0.1194459;
//GAk= dG*0.009*kd + 


 
