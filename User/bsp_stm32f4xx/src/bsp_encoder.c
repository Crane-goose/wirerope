#include "bsp.h"

/***************************************************
							定时器4
							位数：32位
							输入时钟频率：84M
***************************************************/
static void Encoder_TIM4_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;	
	TIM_ICInitTypeDef TIM_ICInitStructure;
	 NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  ///使能TIM5时钟
	
    TIM_TimeBaseInitStructure.TIM_Period = 8;//20; 	//自动重装载值
    //TIM_TimeBaseInitStructure.TIM_Period = 2399;//20; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=0;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);//初始化TIM5
	
	TIM_EncoderInterfaceConfig(TIM4,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);
	
	TIM_ICStructInit(&TIM_ICInitStructure);
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1 ;	
	TIM_ICInitStructure.TIM_ICFilter = 0; //最大为15
	TIM_ICInit(TIM4, &TIM_ICInitStructure);
	
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2 ;	
	TIM_ICInitStructure.TIM_ICFilter = 0;
	TIM_ICInit(TIM4, &TIM_ICInitStructure);


	//溢出中断设置
	 TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE); //允许TIM4溢出中断 
	 
	 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	 
	 NVIC_InitStructure.NVIC_IRQChannel=TIM4_IRQn; 
	 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x00; 
	 NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x00; 
	 NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE; 
	 NVIC_Init(&NVIC_InitStructure);

	
	TIM_SetCounter(TIM4, 0);//应该不能为0
	
	TIM_Cmd(TIM4,ENABLE);
}
#define Get_count TIM_GetCounter(TIM4)


static void Encoder_TIM4_IO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 

	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_TIM4);	
														   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;    
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;//GPIO_PuPd_UP;//GPIO_PuPd_NOPULL
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}
void Encoder_Init(void)
{
	Encoder_TIM4_IO_Init();
	Encoder_TIM4_Init();
}
/**************************************************************
								编码器溢出中断
**************************************************************/
int circle_count=0;//全局变量-圈数
int flagEncoder = 0;
void TIM4_IRQHandler(void) 
{ 
    //编码器模式
    flagEncoder = 1;
    
	if(TIM_GetITStatus(TIM4,TIM_IT_Update)==SET) 
    { 
        if((TIM4->CR1>>4 & 0x01)==0) //DIR==0 
            circle_count++; 
        else if((TIM4->CR1>>4 & 0x01)==1)//DIR==1 
            circle_count--; 
    } 
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update); 
}

