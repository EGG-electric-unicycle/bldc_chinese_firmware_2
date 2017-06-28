/*bldcinit.c*/
//xx0410
/*********************************************************************************************************
*
* File                : bldcinit.c
* Hardware Environment: kech ver 1.0
* Build Environment   : RealView MDK-ARM  Version: 4.22
* Version             : V1.0
* By                  : lihewen
*
*                                  (c) Copyright 2012-, lihewen
*                                          All Rights Reserved
*
*********************************************************************************************************/
/******************************************************************************************************** 
STM32_TIM_ADC_Configuration()
描述：TIM1、TIM8,ADC1、ADC2、ADC3初始化函数
用途：产生驱动电机PWM，ADC测量指定端口的输入电压
使用条件： TIM,ADC初始化调用；
TIM1管脚重映射，TIM1_CH1,TIM1_CH1N,TIM1_CH2,TIM1_CH2N,TIM1_CH3,TIM1_CH3N互补输出
    TIM8_CH1,TIM8_CH1N,TIM8_CH2,TIM8_CH2N,TIM8_CH3,TIM8_CH3N互补输出
TIM1--GPIOE(Right Notor): PE8:PE9 = UL:UH, PE10:PE11= VL:VH, PE12:PE13 = WL:WH;   
TIM8--GPIOx(Lift Notor):  UL:VL:WL = PA7:PB0:PB1; UH:VH:WH = PC6:PC7:PC8;
死区时间设置为零，可由TIM_BDTRInitStructure.TIM_DeadTime = 0x00; 自行设置
ADC1，ADC2工作在同步规则和同步注入模式，规则组通道转换由软件触发启动，注入组通道转换由TIM1_TRGO  Update事件触发启动
(1)ADC1 采样1组规则通道  ADC_Channel_14 
(2) ADC2 采样1组规则通道  ADC_Channel_15  
(3)规则组转换由软件触发启动，同步启动ADC1、ADC2
ADC1注入通道采样ADC123_IN0 、ADC2注入通道采样ADC123_IN3  左电机的U、V 相电流同时采样，通道切换在 TIM1更新事件中断里，
               中断事件触发注入组采样启动，等待采样完成，切换采样通道
ADC3 工作在独立状态，软件触发启动。ADC3一次触发转换5个通道ADC3, ADC_Channel_5、ADC3, ADC_Channel_7、
         ADC3, ADC_Channel_10、ADC3, ADC_Channel_11、ADC3, ADC_Channel_13
ADC1、ADC3启动了DMA传输，转换的数据从ADC_DR寄存器传输到用户指定的目的地址。ADC1、ADC2共用DMA 
            ADC2规则通道数据为ADC_DR里高16位，低16为为ADC1数据 
ADC1、ADC2规则组通道转换数据在地址为0X20000100 SRAM里，高16位为ADC2数据，低16位为ADC1数据
ADC1、ADC2注入组通道转换数据在分别在ADC1->JDR1，ADC2->JDR1中。左电机、右电机数据分时复用ADC1->JDR1，ADC2->JDR1
ADC3 规则组通道转换数据在起始地址为0X20000130 SRAM里 数据为32位，低12位有效 一共5组数据（5*4，每一个数据4个字节32位）

说明:
ADC1、ADC2注入组通道转换,注入组通道转换由TIM1_TRGO  Update事件触发ADC1，ADC2 同时启动 一个PWM周期启动一次。
    第一个周期采样左电机，第二个周期采样右电机，第三个又采样左电机，如此往复
ADC1  LEFT_U_I_ADC  ADC123_IN0 PA0
ADC2  LEFT_V_I_ADC  ADC123_IN0 PA3
ADC1  RIGHT_U_I_ADC ADC123_IN0 PA1
ADC2  RIGHT_V_I_ADC ADC123_IN0 PA2
ADC1、ADC2规则组通道转换 启动软件控制，一次启动同时转换ADC1，ADC2 调用函数ADC_SoftwareStartConvCmd(ADC1, ENABLE);//启动ADC1 ADC2规则组转换
ADC1 右母线电流监测   ADC12_IN14  PC4
ADC2 左母线电流监测   ADC12_IN15  PC5
ADC3 规则组通道转换  启动软件控制，一次启动转换5组 调用函数ADC_SoftwareStartConvCmd(ADC3, ENABLE);//启动ADC3规则组转换
左电机温度测量   ADC3_IN5  PF7
右电机温度测量   ADC3_IN7  PF9
3.3V电源电压监测 ADC123_IN10  PC0
母线电压监测     ADC123_IN11  PC1
方向ADC          ADC123_IN13  PC3
*******************************************************************************/
#include "stm32f10x.h"        //这个头文件包括STM32F10x所有外围寄存器、位、内存映射的定义
#include "MC_Globals.h"
//u8 a=0;
u32 ADC12_Buffer; //__attribute__((at(0X2000f10e0)))= {0};//ADC1-2,DMA缓存 ADC12_Buffer[15:0]=ADC1, ADC12_Buffer[31:16]=ADC2
//
u32 ADC3_Buffer[5]; // __attribute__((at(0X2000f130)))= {0};
//ADC12_Buffer[15:0]=ADC1, --PC4右母线电流
//ADC12_Buffer[31:16]=ADC2,--PC5左母线电流
//u32 *pADC12_Buffer = ADC12_Buffer; //= (u16)&ADC12_Buffer;
//u32 x,y; //x = L/R U相Ia. y = L/R V相Ib.
u16 TimUpdateCount;
u8 Gyro_Data_ready;
/*#define Deadtime Value   */ //(u16)((unsigned long long)CKTIM/2 *(unsigned long long)DEADTIME_NS/1000000000uL)
//#define DEADTIME    ((u16) 30) //define deadtime =800ns   //DTG[7:0]x27.778 ns. @TIM_CKD_DIV2,(tDTS = 2 × tCK_INT); DTG[7:5]=0xx => DT=DTG[7:0]x tdtg with tdtg=tDTS.  
         //DEADTIME*(1/(72(MHz)/2))= DEADTIME_NS  => DEADTIME = (DEADTIME_NS(ns)//1000) * (72(MHz)/2)
//#define DEADTIME    ((u16) 36)  //define deadtime =1000ns
//#define DEADTIME    ((u16) 43)  //define deadtime =1200ns
//#define DEADTIME    ((u16) 54)  //define deadtime =1500ns  //初期调试时，选大些
//extern void SVPWM_CalcDutyCycles(void); //xx0410
//extern void uart_RxTx(void);
//extern void uart_Tx(void);
//extern void uart_Rx(void);
/*GPIO管脚的配置*/
//选用ADC的通道0  1  2  3  0  1  3  4  5  7  9，分别对应的管脚为PA0  PA1  PA2  PA3   PC0  PC1  PC3  PC4  PC5  PF7  PF9
//(PA0 : PA1 : PA2 : PA3 = LEFT_U_I_ADC : RIGHT_U_I_ADC : RIGHT_V_I_ADC : LEFT_V_I_ADC) lihewen 2012-09-17
//ADC1注入: 左/右电机Ia检测(L:R = PA0:PA1 = CH0:CH1)   //lihewen 2012-09-20
//ADC2注入: 左/右电机Ib检测(L:R = PA3:PA2 = CH3:CH2)
//ADC1 右母线电流监测   ADC12_IN14  PC4
//ADC2 左母线电流监测   ADC12_IN15  PC5
//左电机温度测量   ADC3_IN5  PF7
//右电机温度测量   ADC3_IN7  PF9
//3.3V电源电压监测 ADC123_IN10  PC0
//母线电压监测     ADC123_IN11  PC1
//方向ADC          ADC123_IN13  PC3
//void Tim_ADC_GPIO_Configuration(void)	 //xx0410
//{
// GPIO_InitTypeDef  GPIO_InitStructure;
////ADC_GPIO_Configuration 
// //PA0/1/2/3 作为模拟通道输入引脚                         
// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0| GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;     //模拟输入引脚
// GPIO_Init(GPIOA, &GPIO_InitStructure);
// //PC0/1/3/4/5 作为模拟通道输入引脚                         
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;
// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;     //模拟输入引脚
// GPIO_Init(GPIOC, &GPIO_InitStructure);
// //joystick: enc转向
//// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_6 ;      //转向2PULSE引脚TURN_ENCODER_2PULSE
//// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;     
//// GPIO_Init(GPIOG, &GPIO_InitStructure);
////
//// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 ;     //转向INDEX信号 TURN_ENCODER_INDEX
//// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;     
//// GPIO_Init(GPIOC, &GPIO_InitStructure);
////
//// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 ;     //转向DIRECT信号 TURN_ENCODER_DIRECT
//// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;     
//// GPIO_Init(GPIOD, &GPIO_InitStructure);
//
// //PF/7/9 作为模拟通道输入引脚                         
// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_9;
// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;     //模拟输入引脚
// GPIO_Init(GPIOF, &GPIO_InitStructure);
// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_10;      //右电机硬过流
// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;     //上拉输入引脚
// GPIO_Init(GPIOF, &GPIO_InitStructure);
// //GPIOF,GPIO_Pin_10--右脚踏开关 
// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;      //右Footswitch
// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;     //上拉输入引脚
// GPIO_Init(GPIOF, &GPIO_InitStructure);
// 
////TIM_GPIO_Configuration
// //配置TIM1、TIM8.GPIO做相应设置，为AF输出
// //(TIM8 = Lift Motor PWM; UL:VL:WL = PA7:PB0:PB1; UH:VH:WH = PC6:PC7:PC8)
// //PA.7/口设置为TIM8_CH1N   
// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;        //推挽复用输出模式
// GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
// GPIO_Init(GPIOA, &GPIO_InitStructure);
// 
// //PB.0/PB.1口设置为TIM8_CH2N和TIM8_CH3N输出口
// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;        //推挽复用输出模式
// GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    
// GPIO_Init(GPIOB, &GPIO_InitStructure);
// 
// //PC.6/7/8口设置为TIM8_CH1、TIM8_CH2和TIM8_CH3输出口
// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7|GPIO_Pin_8;
// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;        //推挽复用输出模式
// GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
// GPIO_Init(GPIOC, &GPIO_InitStructure);
// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;         //左电机硬过流
// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;        //上拉输入模式
// GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
// GPIO_Init(GPIOC, &GPIO_InitStructure);
// 
// //PE.8/9/10/11/12/13口设置为TIM1_CH1、TIM1_CH2和TIM1_CH3//TIM1_CH1N、TIM1_CH2N和TIM1_CH3N输出口
// //(TIM1--GPIOE(Right Notor): PE8:PE9 = UL:UH, PE10:PE11= VL:VH, PE12:PE13 = WL:WH;)
// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 |GPIO_Pin_9 | GPIO_Pin_10|GPIO_Pin_11 |
//               GPIO_Pin_12 | GPIO_Pin_13;
// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;        //推挽复用输出模式
// GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
// GPIO_Init(GPIOE, &GPIO_InitStructure);
// //PE.1=Beep;PE.2=PWM_RIGHT_SD; PE.7=LED IND设置为输出口 
// GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 |GPIO_Pin_2|GPIO_Pin_1; //PWM_SD;//指示更新事件中断
// GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;     //推挽输出模式
// GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
// GPIO_Init(GPIOE, &GPIO_InitStructure);
// 
// BEEP_OFF;
//}
 

/*配置系统时钟,使能各外设时钟*/
//void Tim_ADC_RCC_Configuration(void) //xx0410
//{
//// SystemInit(); 
// RCC_PCLK2Config(RCC_HCLK_Div1);
//     /*使能各个外设时钟*/
// RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA| RCC_APB2Periph_GPIOB| RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOE|
//    RCC_APB2Periph_GPIOF |RCC_APB2Periph_ADC1|RCC_APB2Periph_ADC2|RCC_APB2Periph_ADC3 , ENABLE );   //使能ADC1通道时钟，各个管脚时钟
// /* Configure ADCCLK such as ADCCLK = PCLK2/6 */ 
// RCC_ADCCLKConfig(RCC_PCLK2_Div8);          //72M/6=12,ADC最大时钟不能超过14M
//// RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);    //使能DMA1时钟
// RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);    //使能DMA2时钟
// //启动AFIO
// RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
// GPIO_PinRemapConfig(GPIO_FullRemap_TIM1,ENABLE);     //TIM1管脚重映射
// //启动TIM1、TIM8
// RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
// RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
//  
//}
 
/*配置ADC1*/
void ADC_Configuration(void)
{
  ADC_InitTypeDef  ADC_InitStructure;
 /* ADC1 configuration ------------------------------------------------------*/
 ADC_InitStructure.ADC_Mode = ADC_Mode_RegInjecSimult ;   //ADC工作模式:ADC1和ADC2工作在同步规则和同步注入模式
 ADC_InitStructure.ADC_ScanConvMode =ENABLE;      //模数转换工作在扫描模式
 ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;    //模数转换工作在连续转换模式
 ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None ;//转换由软件而不是外部触发启动
 ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;   //ADC数据右对齐
 ADC_InitStructure.ADC_NbrOfChannel = 1;       //顺序进行规则转换的ADC通道的数目
 ADC_Init(ADC1, &ADC_InitStructure);        //根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器
 //ADC1规则通道配置 
 /* ADC1 regular channel configuration */ 
 ADC_DiscModeChannelCountConfig(ADC1, 1);       //间断模式规则组通道计数器的值为1
 //右母线电流检测 ADC1, ADC_Channel_14
 ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 1, ADC_SampleTime_1Cycles5 );  //_1Cycles5,_7Cycles5,_13Cycles5,_28Cycles5...
 ADC_DiscModeCmd(ADC1, ENABLE);          //使能间断模式
 ADC_ExternalTrigConvCmd(ADC1, ENABLE);       //使能规则组外部触发
 //ADC1注入通道配置 
 ADC_InjectedSequencerLengthConfig(ADC1, 1);       //注入组通道的转换序列长度1
 ADC_InjectedChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_1Cycles5);  
 ADC_InjectedDiscModeCmd(ADC1, ENABLE);         //注入组间断模式
 ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_TRGO); //选择定时器1的触发输出作为注入转换外部触发
 ADC_ExternalTrigInjectedConvCmd(ADC1, ENABLE); //使能外部触发
 
/* ADC2 configuration ------------------------------------------------------*/
 ADC_InitStructure.ADC_Mode = ADC_Mode_RegInjecSimult;   //ADC工作模式:ADC1和ADC2工作在同步规则和同步注入模式
 ADC_InitStructure.ADC_ScanConvMode =ENABLE;      //模数转换工作在扫描模式
 ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;    //模数转换工作在连续转换模式
 ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; //转换由软件而不是外部触发启动
 ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;   //ADC数据右对齐
 ADC_InitStructure.ADC_NbrOfChannel = 1;       //顺序进行规则转换的ADC通道的数目
 ADC_Init(ADC2, &ADC_InitStructure);        //根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器
 //ADC2规则通道配置
 ADC_DiscModeChannelCountConfig(ADC2, 1);       //间断模式规则组通道计数器的值为1
 //左母线电流检测 ADC2, ADC_Channel_15
 ADC_RegularChannelConfig(ADC2, ADC_Channel_15, 1, ADC_SampleTime_1Cycles5 );  //_1Cycles5,_7Cycles5,_13Cycles5,_28Cycles5...
 ADC_ExternalTrigConvCmd(ADC1, ENABLE);
 ADC_DiscModeCmd(ADC2, ENABLE);          //使能间断模式
 //ADC2注入通道配置
 ADC_InjectedSequencerLengthConfig(ADC2, 1);       //注入组通道的转换序列长度4 
 ADC_InjectedChannelConfig(ADC2, ADC_Channel_2, 1, ADC_SampleTime_1Cycles5);  //同一通道采集四次
 ADC_InjectedDiscModeCmd(ADC2, ENABLE);         //注入组间断模式
 ADC_ExternalTrigInjectedConvConfig(ADC2, ADC_ExternalTrigInjecConv_None); //注入转换由软件而不是外部触发启动
 ADC_ExternalTrigInjectedConvCmd(ADC2, ENABLE);      //使能外部触发

 /* ADC3 configuration ------------------------------------------------------*/
 ADC_InitStructure.ADC_Mode = ADC_Mode_Independent ;    //独立模式
 ADC_InitStructure.ADC_ScanConvMode =ENABLE;      //模数转换工作在扫描模式
 ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;    //模数转换工作在连续转换模式
 ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None ;//转换由软件而不是外部触发启动
 ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;   //ADC数据右对齐
 ADC_InitStructure.ADC_NbrOfChannel = 5;       //顺序进行规则转换的ADC通道的数目
 ADC_Init(ADC3, &ADC_InitStructure);        //根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器
 //ADC3规则通道配置 
 ADC_DiscModeChannelCountConfig(ADC3, 5);       //间断模式规则组通道计数器的值为1
 ADC_RegularChannelConfig(ADC3, ADC_Channel_5, 1, ADC_SampleTime_1Cycles5 );   //_1Cycles5,_7Cycles5,_13Cycles5,_28Cycles5...
 ADC_RegularChannelConfig(ADC3, ADC_Channel_7, 2, ADC_SampleTime_1Cycles5 );
 ADC_RegularChannelConfig(ADC3, ADC_Channel_10, 3, ADC_SampleTime_1Cycles5 );
 ADC_RegularChannelConfig(ADC3, ADC_Channel_11, 4, ADC_SampleTime_1Cycles5 );
 ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 5, ADC_SampleTime_1Cycles5 );  //方向ADC
 ADC_DiscModeCmd(ADC3, ENABLE);          //使能间断模式
 ADC_ExternalTrigConvCmd(ADC3, ENABLE);        //使能规则组外部触发
 // 开启ADC的DMA支持（要实现DMA功能，还需独立配置DMA通道等参数）
    ADC_DMACmd(ADC1, ENABLE);
 ADC_DMACmd(ADC3, ENABLE);
 
 /* Enable ADC1/ADC2 */
  ADC_Cmd(ADC1, ENABLE);              //使能指定的ADC1
  ADC_Cmd(ADC2, ENABLE);              //使能指定的ADC2
  ADC_Cmd(ADC3, ENABLE);              //使能指定的ADC3
  /* Enable ADC1 reset calibaration register */   
 ADC_ResetCalibration(ADC1);            //复位指定的ADC1的校准寄存器
 /* Check the end of ADC1 reset calibration register */
 while(ADC_GetResetCalibrationStatus(ADC1));      //获取ADC1复位校准寄存器的状态,设置状态则等待
 
 /* Start ADC1 calibaration */
 ADC_StartCalibration(ADC1);          //开始指定ADC1的校准状态
 /* Check the end of ADC1 calibration */
 while(ADC_GetCalibrationStatus(ADC1));       //获取指定ADC1的校准程序,设置状态则等待
 
 
 /* Enable ADC2 reset calibaration register */   
 ADC_ResetCalibration(ADC2);            //复位指定的ADC2的校准寄存器
 /* Check the end of ADC2 reset calibration register */
 while(ADC_GetResetCalibrationStatus(ADC2));      //获取ADC2复位校准寄存器的状态,设置状态则等待
 
 /* Start ADC2 calibaration */
 ADC_StartCalibration(ADC2);          //开始指定ADC2的校准状态
 /* Check the end of ADC2 calibration */
 while(ADC_GetCalibrationStatus(ADC2));       //获取指定ADC2的校准程序,设置状态则等待

 /* Enable ADC23 reset calibaration register */   
 ADC_ResetCalibration(ADC3);            //复位指定的ADC3的校准寄存器
 /* Check the end of ADC3 reset calibration register */
 while(ADC_GetResetCalibrationStatus(ADC3));      //获取ADC3复位校准寄存器的状态,设置状态则等待
 
 /* Start ADC3 calibaration */
 ADC_StartCalibration(ADC3);          //开始指定ADC3的校准状态
 /* Check the end of ADC3 calibration */
 while(ADC_GetCalibrationStatus(ADC3));       //获取指定ADC3的校准程序,设置状态则等待
}
 
/*配置DMA*/
void DMA_Configuration(void)
{
 
 /* ADC1  DMA1 Channel Config */
 DMA_InitTypeDef DMA_InitStructure;
// DMA_DeInit(DMA1_Channel1);            //将DMA1的通道1寄存器重设为缺省值
// DMA_InitStructure.DMA_PeripheralBaseAddr =  (u32)&ADC1->DR;  //DMA外设ADC基地址
// DMA_InitStructure.DMA_MemoryBaseAddr = (u32)ADC12_Buffer;  //0x20000100;    //DMA内存基地址
// DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;     //内存作为数据传输的目的地
// DMA_InitStructure.DMA_BufferSize = 1;         //DMA通道的DMA缓存的大小
// DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址寄存器不变
// DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;    //内存地址寄存器递增
// DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;  //数据宽度为32位
// DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word; //数据宽度为32位
// DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;      //工作在循环缓存模式
// DMA_InitStructure.DMA_Priority = DMA_Priority_High;    //DMA通道 x拥有高优先级 
// DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;       //DMA通道x没有设置为内存到内存传输
// DMA_Init(DMA1_Channel1, &DMA_InitStructure);       //根据DMA_InitStruct中指定的参数初始化DMA的通道
 /* ADC3  DMA1 Channel Config */
 DMA_DeInit(DMA2_Channel5);            //将DMA2的通道5寄存器重设为缺省值
 DMA_InitStructure.DMA_PeripheralBaseAddr =  (u32)&ADC3->DR;  //DMA外设ADC基地址
 DMA_InitStructure.DMA_MemoryBaseAddr = (u32)ADC3_Buffer; //0x20000130;    //DMA内存基地址
 DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;     //内存作为数据传输的目的地
 DMA_InitStructure.DMA_BufferSize = 5;         //DMA通道的DMA缓存的大小
 DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //外设地址寄存器不变
 DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;    //内存地址寄存器递增
 DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;  //数据宽度为32位
 DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word; //数据宽度为32位
 DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;      //工作在循环缓存模式
 DMA_InitStructure.DMA_Priority = DMA_Priority_High;    //DMA通道 x拥有高优先级 
 DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;      //DMA通道x没有设置为内存到内存传输
 DMA_Init(DMA2_Channel5, &DMA_InitStructure);            //根据DMA_InitStruct中指定的参数初始化DMA的通道
}

//TIM配置
 
void TIM_NVIC_Configuration()
{ 
 NVIC_InitTypeDef NVIC_InitStructure;
 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
 NVIC_InitStructure.NVIC_IRQChannel =TIM1_UP_IRQn ;       //更新事件
 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;   //先占优先级0级
 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;     //从优先级1级
 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;     //IRQ通道被使能
 NVIC_Init(&NVIC_InitStructure);          //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
}
 

//Step3. TIM1、TIM8模块初始化
void TIM_Configuration()
{
    TIM_TimeBaseInitTypeDef TIM_BaseInitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
 TIM_DeInit(TIM1); //lihewen 2012-09-21
 TIM_TimeBaseStructInit(&TIM_BaseInitStructure); //lihewen 2012-09-21
    //TIM1,8时基参数配置/* Time Base configuration */
    //频率=TIM1_CLK/(ARR+1)
    TIM_BaseInitStructure.TIM_Period = PWM_PERIOD;        //自动装载半周期值（2250*2/72000000）=0.000062.5s=16KHZ
    TIM_BaseInitStructure.TIM_Prescaler = 0x0;        //时钟为72MHZ
    TIM_BaseInitStructure.TIM_ClockDivision = 0x0; ////死区时间分频
    TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned3;//TIM 中央对齐模式3计数模式
    TIM_BaseInitStructure.TIM_RepetitionCounter = 0x1;    //重复计数器值为0时产生2次中断
 //TIM1时基设置
    TIM_TimeBaseInit(TIM1, &TIM_BaseInitStructure);     //调用初始化函数初始化TIM1
 TIM_ARRPreloadConfig(TIM1, ENABLE);        //启用ARR的影子寄存器（直到产生更新事件才更改设置）
 //TIM8时基设置 
 TIM_TimeBaseInit(TIM8, &TIM_BaseInitStructure);     //调用初始化函数初始化TIM8
 TIM_ARRPreloadConfig(TIM8, ENABLE);        //启用ARR的影子寄存器（直到产生更新事件才更改设置）

 TIM_OCStructInit(&TIM_OCInitStructure); //lihewen 2012-09-21
 /* Channel 1, 2,3 in PWM mode */
    //TIM1_OC模块参数配置
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;    //TIM 脉冲宽度调制模式 1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //TIM 输出比较使能
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable; //TIM 互补输出比较使能
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High ;  //TIM 输出比较极性高
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High ; //TIM 互补输出比较极性高
    TIM_OCInitStructure.TIM_Pulse = PWM_PERIOD >> 1;   //1125;   //待装入捕获比较器的值（此寄存器设置PWM占空比25%）         
 TIM_OCInitStructure.TIM_OCIdleState  = TIM_OCIdleState_Reset; //lihewen 2012-09-21
 TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;  //lihewen 2012-09-21
 //TIM1_OC模块设置
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
 TIM_OC2Init(TIM1, &TIM_OCInitStructure);
 TIM_OC3Init(TIM1, &TIM_OCInitStructure); 
    
 //TIM8_OC模块设置
    TIM_OC1Init(TIM8, &TIM_OCInitStructure);
 TIM_OC2Init(TIM8, &TIM_OCInitStructure);
 TIM_OC3Init(TIM8, &TIM_OCInitStructure);

 /* Enables the Preload on CCx Register */
    //TIM1_OCx模块设置启用CCRx寄存器的影子寄存器（直到产生更新事件才更改设置）
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
    //TIM8_OCx模块设置启用CCRx寄存器的影子寄存器（直到产生更新事件才更改设置）
    TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
 TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);

    //死区设置
    TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;  //使能TIM1 OSSR状态 
    TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;  //使能TIM1 OSSI状态
    TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_OFF;  //不锁任何位
    TIM_BDTRInitStructure.TIM_DeadTime  = DEADTIME;      //这里调整死区大小0-0x7f,可以实现0——1777ns 或者取0x80-0xc0可以实现1777ns——3555ns
    TIM_BDTRInitStructure.TIM_Break = TIM_Break_Disable;   //失能TIM1刹车输入
    TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;//TIM1刹车输入管脚极性低
    TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;//自动输出功能使能?2012-09-17
    TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);
    TIM_BDTRConfig(TIM8, &TIM_BDTRInitStructure);
 //配置同步启动TIM1、TIM8
 /* Select the Master Slave Mode: TIM1 */
    TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable);  //配置TIM1为主模式
 TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Enable );    //使能被选为触发输出(TRGO) 
 TIM_SelectSlaveMode(TIM1,   TIM_SlaveMode_Trigger  );
 TIM_SelectInputTrigger(TIM1, TIM_TS_TI1F_ED); 
 TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable);
 /* Slave Mode selection: TIM8 */
 TIM_SelectSlaveMode(TIM8, TIM_SlaveMode_Trigger  );    //配置TIM8为从模式                
    TIM_SelectInputTrigger(TIM8, TIM_TS_ITR0);      //选择TIM 内部触发 0
 //软件产生一个更新事件，使预装载寄存器装载到影子寄存器
 TIM_GenerateEvent(TIM1, TIM_EventSource_Update );     
 TIM_GenerateEvent(TIM8, TIM_EventSource_Update ); 
 //中断配置
 TIM_ClearFlag(TIM1, TIM_FLAG_Update);        //清零中断标志
 TIM_ITConfig(TIM1, TIM_IT_Update , ENABLE );     //使能更新中断
// TIM_ClearFlag(TIM8, TIM_FLAG_Update);        //清零中断标志
// TIM_ITConfig(TIM8, TIM_IT_Update , ENABLE );     //使能更新中断
 TIM1->CCR1 = 1125;
 TIM1->CCR2 = 1125;
 TIM1->CCR3 = 1125;
 TIM8->CCR1 = 1125;
 TIM8->CCR2 = 1125;
 TIM8->CCR3 = 1125;
    TIM_Cmd(TIM1, ENABLE);            //TIM1开启
    TIM_Cmd(TIM8, ENABLE);           //TIM8开启
}

void STM32_TIM_ADC_Configuration(void)
{
// Tim_ADC_RCC_Configuration();	 //xx0410
// Tim_ADC_GPIO_Configuration(); //xx0410
 ADC_Configuration();			 
 DMA_Configuration();			 
// TIM_NVIC_Configuration();	 //xx0410
 TIM_Configuration();
    /* Start ADC1 Software Conversion */ 
// DMA_Cmd(DMA1_Channel1, ENABLE);          //启动DMA通道
 DMA_Cmd(DMA2_Channel5, ENABLE);          //启动DMA通道
 ADC_SoftwareStartConvCmd(ADC1, ENABLE);
 ADC_SoftwareStartConvCmd(ADC2, ENABLE);
 ADC_SoftwareStartConvCmd(ADC3, ENABLE);
// ADC_SoftwareStartInjectedConvCmd(ADC2, ENABLE);    //2012-11-27
 TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update ); //?2012-09-17
}




