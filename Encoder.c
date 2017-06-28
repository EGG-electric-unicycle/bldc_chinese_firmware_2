/*encoder config*/


/*
main()调用如下2函数，进行encoder相关配置
TIM4_Configuration(); //
//extINT_init();
*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/* Private function prototypes -----------------------------------------------*/

u8 prnt_opt=0x00;

void test_2DataOutput(s16, s16);
void test_1DataOutput(s16);



//Tim4 编码器模式
void Tim4_RCC_Configuration()
{
	//Step1.开启TIM和相应端口时钟
	//启动GPIO
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD  ,ENABLE);

	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	 	//启动AFIO
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_TIM4,ENABLE);	 
}


void Tim4_GPIO_Configuration()
{//配置TIM4、ch1,ch2 GPIO做相应设置，为AF输出
	GPIO_InitTypeDef GPIO_InitStructure;
	//PA.7/口设置为TIM8_CH1N
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

		//CPU_LED==PD4
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;					    //指示更新事件中断
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		 		//推挽输出模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

}

void TIM_4_Configuration()
{ 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure; 
	TIM_ICInitTypeDef TIM_ICInitStructure; 
	TIM_ICStructInit(& TIM_ICInitStructure); 
	//TIM_ICStructInit(&TIM_ICInitStructure); 
	//TIM_ICInitStructure.TIM_ICMode = TIM_ICMode_PWMI;
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1|TIM_Channel_2; 
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; 
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; 
	TIM_ICInitStructure.TIM_ICFilter = 0x0; 
	TIM_ICInit(TIM4, &TIM_ICInitStructure);
	
	
	
	
	
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF; 
	
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
	
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);  // Time base configuration 
	
	
	
	
	 
	//TIM_ETRConfig(TIM3,TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);
	//TIM_SelectInputTrigger(TIM4, TIM_TS_TI2FP2 );
	
	//TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_External1);
	//TIM_ETRClockMode1Config(TIM4, TIM_ExtTRGPSC_OFF,TIM_ExtTRGPolarity_Inverted, 0x0);
	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12,TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_SetCounter(TIM4, 0);
	    
	
	TIM_Cmd(TIM4, ENABLE);
}

/*调用1*/
void TIM4_Configuration(void)
{
  	 Tim4_RCC_Configuration();
	 Tim4_GPIO_Configuration();
	 TIM_4_Configuration();
}


//Tim4 4倍频模式
/*
void Tim4_RCC_Configuration()
{

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD  ,ENABLE);

	 RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	 	//启动AFIO
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_TIM4,ENABLE);	 
}


void Tim4_GPIO_Configuration()
{//配置TIM1、TIM8.GPIO做相应设置，为AF输出
	GPIO_InitTypeDef GPIO_InitStructure;
	//PA.7/口设置为TIM8_CH1N
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		   
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

}

//Step3. TIM1、TIM8模块初始化
void TIM_4_Configuration()
{ 
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure; 
TIM_ICInitTypeDef TIM_ICInitStructure; 
TIM_ICStructInit(& TIM_ICInitStructure); 
//TIM_ICStructInit(&TIM_ICInitStructure); 
//TIM_ICInitStructure.TIM_ICMode = TIM_ICMode_PWMI;
TIM_ICInitStructure.TIM_Channel = TIM_Channel_2; 
TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising; 
TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; 
TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; 
TIM_ICInitStructure.TIM_ICFilter = 0x0; 
TIM_ICInit(TIM4, &TIM_ICInitStructure);



TIM_TimeBaseStructure.TIM_Prescaler = 0;

TIM_TimeBaseStructure.TIM_Period = 0XFFFF; 

TIM_TimeBaseStructure.TIM_ClockDivision = 0; 

TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 

TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);  // Time base configuration 




 
//TIM_ETRConfig(TIM3,TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);
TIM_SelectInputTrigger(TIM4, TIM_TS_TI2FP2 );
TIM_SelectSlaveMode(TIM4, TIM_SlaveMode_External1);
//TIM_ETRClockMode1Config(TIM4, TIM_ExtTRGPSC_OFF,TIM_ExtTRGPolarity_Inverted, 0x0);
TIM_SetCounter(TIM4, 0);
    

TIM_Cmd(TIM4, ENABLE);


}

void TIM4_Configuration()
{
  	 Tim4_RCC_Configuration();
	 Tim4_GPIO_Configuration();
	 TIM_4_Configuration();
}
*/


/*externalINT */

/**********************************************************************
* 名    称：RCC_Configuration()
* 功    能：配置时钟
* 入口参数： 
* 出口参数：
-----------------------------------------------------------------------
* 说明：使用库函数
***********************************************************************/
void extINT_RCC_Configuration(void)
{

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD|
  RCC_APB2Periph_GPIOF|RCC_APB2Periph_GPIOG|RCC_APB2Periph_AFIO , ENABLE); //给GPIOA和重映射提供时钟，注意：一定要设置RCC_APB2Periph_AFIO

}


/**********************************************************************
* 名    称：GPIO_Configuration()
* 功    能：配置输入输出
* 入口参数： 
* 出口参数：
-----------------------------------------------------------------------
* 说明：使用库函数，配置IO口
***********************************************************************/
 void extINT_GPIO_Configuration(void)
 {

  GPIO_InitTypeDef  GPIO_InitStructure;
  /*设置D10为输入*/

///////方向控制 正反转，INDEX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 ;			   //转向计数2倍频引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		   
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 ;			  //转向INDEX信号
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		   
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;				   //转向方向信号
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		   
	GPIO_Init(GPIOF, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;//|GPIO_Pin_12 ;	 //左轮编码器 正反转，INDEX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;		   
	GPIO_Init(GPIOD, &GPIO_InitStructure);


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;				    //右轮编码器 正反转，INDEX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		   
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		   
	GPIO_Init(GPIOB, &GPIO_InitStructure);




 }

/**********************************************************************
* 名    称:EXTI_Configuration()
* 功    能:
* 入口参数：
* 出口参数：
-----------------------------------------------------------------------
* 说明： 
	PD8  -- left index - 连接到外部中断通道8
	PB10 --
	PB11 --
***********************************************************************/
void extINT_EXTI_Configuration(void)
{
  EXTI_InitTypeDef  EXTI_InitStructure;


  /* Configure EXTI Line3 to generate an interrupt on falling edge */  
//  EXTI_InitStructure.EXTI_Line = EXTI_Line2;                   		//外部中断通道2
//  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling ;   //上升下降沿触发
//  EXTI_InitStructure.EXTI_LineCmd = ENABLE;        
//  EXTI_Init(&EXTI_InitStructure);
//  GPIO_EXTILineConfig(GPIO_PortSourceGPIOG, GPIO_PinSource2);  		//将PG2连接到外部中断通道2
//  EXTI_ClearITPendingBit(EXTI_Line2);
//
//  EXTI_InitStructure.EXTI_Line = EXTI_Line3;                   		//外部中断通道3
//  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling ;    		//下降沿触发
//  EXTI_InitStructure.EXTI_LineCmd = ENABLE;        
//  EXTI_Init(&EXTI_InitStructure);
//  GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource3);  		//将PC3连接到外部中断通道3
//  EXTI_ClearITPendingBit(EXTI_Line3);


  EXTI_InitStructure.EXTI_Line = EXTI_Line8;                   		//外部中断通道8
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;    		//下降沿触发
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;        
  EXTI_Init(&EXTI_InitStructure);
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource8);  		//将PD8连接到外部中断通道8
  EXTI_ClearITPendingBit(EXTI_Line8);


//  EXTI_InitStructure.EXTI_Line = EXTI_Line12;                   	//外部中断通道12
//  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling ;    //上升下降沿触发
//  EXTI_InitStructure.EXTI_LineCmd = ENABLE;        
//  EXTI_Init(&EXTI_InitStructure);
//  GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource12);  	//将PD12连接到外部中断通道12
//  EXTI_ClearITPendingBit(EXTI_Line12);


  EXTI_InitStructure.EXTI_Line = EXTI_Line11;                   	//外部中断通道11
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling ;   //上升下降沿触发
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;        
  EXTI_Init(&EXTI_InitStructure);
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource11); 		//将PB11连接到外部中断通道11
  EXTI_ClearITPendingBit(EXTI_Line11);

  EXTI_InitStructure.EXTI_Line = EXTI_Line10;                   	//外部中断通道10
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling ;      	//下降沿触发
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;        
  EXTI_Init(&EXTI_InitStructure);
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource10);  	//将PB10连接到外部中断通道10
  EXTI_ClearITPendingBit(EXTI_Line10);



}

/**********************************************************************
* 名    称:NVIC_Configuration()
* 功    能:
* 入口参数：
* 出口参数：
-----------------------------------------------------------------------
* 说明： 配置编码器index输入中断
***********************************************************************/
 void extINT_NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure; 

//  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);           //抢占式优先级别设置为无抢占优先级
//  NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;    		//指定中断源
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;        //指定响应优先级别2
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;        	//使能外部中断通道2
//  NVIC_Init(&NVIC_InitStructure);
//
//  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);           //抢占式优先级别设置为无抢占优先级
//  NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;    		//指定中断源
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;        //指定响应优先级别1
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;        	 //使能外部中断通道3
//  NVIC_Init(&NVIC_InitStructure);

//  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);           //抢占式优先级别设置为无抢占优先级
//  NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;    	//指定中断源
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;        //指定响应优先级别2
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;        	 //使能外部中断通道15_10
//  NVIC_Init(&NVIC_InitStructure);
//
//  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);           //抢占式优先级别设置为无抢占优先级
//  NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;    	//指定中断源
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;        //指定响应优先级别1
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;        	 //使能外部中断通道9_5
//  NVIC_Init(&NVIC_InitStructure);

// 	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel =EXTI15_10_IRQn;    			//更新事件
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 		//先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  			//从优先级1级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 				//IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure); 

//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;    			//更新事件
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 		//先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  			//从优先级1级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 				//IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);


} 
///**********************************************************************
//* 名    称：extINT_init(void) 
//* 功    能：外部中断初始化调用
//* 入口参数： 
//* 出口参数：
//-----------------------------------------------------------------------
//* 说明：
//***********************************************************************/
void extINT_init(void) 
{
	 extINT_RCC_Configuration();  //配置时钟                      
	 extINT_GPIO_Configuration(); //配置IO口
	 extINT_EXTI_Configuration(); //外部中断配置
	 extINT_NVIC_Configuration(); //中断配置
}




//左电机INDEX中断处理函数

void EXTI9_5_IRQHandler(void)							//左电机INDEX
{
	u8 opt;
	opt=prnt_opt;
	if(EXTI_GetITStatus(EXTI_Line8) != RESET) 			 
    {
		 if(opt==0x05)
		 	test_1DataOutput(TIM4->CNT);
	     TIM_GenerateEvent(TIM4, TIM_EventSource_Update ); 
	     EXTI_ClearFlag(EXTI_Line8);          
	     EXTI_ClearITPendingBit(EXTI_Line8);
    }
}

void EXTI15_10_IRQHandler(void)					
 {
	 if(EXTI_GetITStatus(EXTI_Line11) != RESET) 	  //右电机方向信号
    {
     //添加中断处理程序
     EXTI_ClearFlag(EXTI_Line11);         
     EXTI_ClearITPendingBit(EXTI_Line11);
     }

	 if(EXTI_GetITStatus(EXTI_Line10) != RESET) 	  //右电机INDEX
    {
     //添加中断处理程序
     EXTI_ClearFlag(EXTI_Line10);          //清除中断标志（必须）
     EXTI_ClearITPendingBit(EXTI_Line10);
     }

 }







