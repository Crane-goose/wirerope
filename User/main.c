/*
*********************************************************************************************************
*
*	模块名称 : 主程序模块。
*	文件名称 : main.c
*	版    本 : V1.2
*	说    明 : V5-107c_FatFS文件系统例程（U盘）
*	修改记录 :
*		版本号  日期       作者    说明
*		v1.0    2013-02-01 armfly  首发
*		v1.1    2013-06-20 armfly  升级BSP模块;更换读取串口命令的写法，不采用 getchar() 阻塞方式。
*		V1.2    2014-02-28 armfly  升级固件库到V1.3.0
*
*	Copyright (C), 2013-2014, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#include "bsp.h"				/* 底层硬件驱动 */
#include "demo_fatfs.h"
#include "ff.h"			/* FatFS文件系统模块*/
#include "usbh_bsp_msc.h"
#include "math.h"



/* 定义例程名和例程发布日期 */
#define EXAMPLE_NAME	"V5-120_AD7606（8通道16位同步ADC）例程"
#define EXAMPLE_DATE	"2014-02-28"
#define DEMO_VER		"1.2"


#define N     9
#define M     9
//滤波相关定义1
void AD7606_Array(void);
void AD7606_Filter(void);
void AD7606_Array_1(void);
void AD7606_Filter_1(void);
static int16_t value_buf0[N];     //中值滤波数组0
static int16_t value_buf1[N];    //中值滤波数组1
static int16_t value_buf2[N];     //中值滤波数组2
static int16_t value_buf3[N];    //中值滤波数组3
static int16_t value_buf4[N];     //中值滤波数组4
static int16_t value_buf5[N];    //中值滤波数组5
static int16_t value_buf6[M];     //中值滤波数组6
uint8_t count=0;
uint8_t count_1=0;
int biaozhi_lb=0;
int biaozhi_lb_1=0;
	
	
int16_t sum;
int16_t sum_lb;
int16_t chan[6];

//巴特沃斯滤波
void bartworth (void);

//float bart_a [9] = {1 , -7.6135 , 25.369 , -48.321 , 57.543 , -43.870 , 20.911 , -5.6975 , 0.67936};
//float bart_b [9] = {3.3756e-12 , 2.7005e-11 , 9.4518e-11 ,1.8904e-10 , 2.3629e-10 , 1.8904e-10 , 9.4518e-11 , 2.7005e-11 , 3.3756e-12};
//float bart_x [9];
//float bart_y [9];
double bart_a [9] = {1 , -7.61353123046636 , 25.3690120949237 , -48.3207014407934 , 57.5427221157395 , 
											-43.8704726071429 , 20.9110761574721 , -5.69746964660762 , 0.679364557739175};
double bart_b [9] = {3.37563310637279E-12 , 2.70050648509823E-11 , 9.45177269784381E-11 ,1.89035453956876E-10 , 
										2.36294317446095E-10 , 1.89035453956876E-10 , 9.45177269784381E-11 , 2.70050648509823E-11 , 3.37563310637279E-12};
double bart_x [9];
double bart_y [9];
uint8_t bart_count_x=0;
uint8_t bart_count_y=0;
int32_t bart_count=0;

int biaozhi_bart=0;
int16_t sum_lb_bart;




void delay_us(uint32_t delay_us)
{
  volatile unsigned int num;
  volatile unsigned int t;

  for (num = 0; num < delay_us; num++)
  {
    t = 11;
    while (t != 0)
    {
      t--;
    }
  }
}

/*
*********************************************************************************************************
*	函 数 名: main
*	功能说明: c程序入口
*	形    参：无
*	返 回 值: 错误代码(无需处理)
*********************************************************************************************************
*/
int main(void)
{
	delay_us(100000);//短暂延时，确保外设供电

//	uint8_t ucKeyCode;
	uint8_t ucRefresh = 0;
	uint8_t ucFifoMode =0;//1;均值 2：采集 3：停止
//	uint16_t storage = 0;
	uint16_t reset = 0;
	//int16_t sum;外部定义
	int32_t add[6];
	int16_t ave[7];
	//int16_t chan[6];
	char buffer[48] = "位置,通道1,通道2,通道3,通道4,通道5,通道6,和通道\n" ;
	int16_t middle = 151;
	int16_t end = 302;

	
	uint16_t Mark = 0;
	uint8_t Calibration = 0;
	int16_t channel[6];
	int16_t sumchannel;
	int32_t num;
	int32_t len;


	char BUF0[5];
	char BUF1[5];
	char BUF2[5];
	char BUF3[5];
	char BUF4[5];
	char BUF5[5];
	char BUF6[6];
	char BUF8[10];
	
	FATFS fs;
	FIL file;
  DIR DirInf;
	uint32_t bw;
	

	
	bsp_Init();
	
	//bsp_DelayMS(500);


	
	#ifdef USE_USB_OTG_FS
		USBH_Init(&USB_OTG_Core,
			USB_OTG_FS_CORE_ID,
            &USB_Host,
            &USBH_MSC_cb,
            &USR_cb);
	#else
		USBH_Init(&USB_OTG_Core,
			USB_OTG_HS_CORE_ID,
            &USB_Host,
            &USBH_MSC_cb,
            &USR_cb);
	#endif



	 	/* AD7606进入普通工作模式 */
	

	bsp_InitAD7606();	/* 配置AD7606所用的GPIO */

	AD7606_SetOS(AD_OS_NO);		/* 无过采样 */
	AD7606_SetInputRange(0);	/* 0表示输入量程为正负5V, 1表示正负10V */
	AD7606_StartConvst();		/* 启动1次转换 */
	ucRefresh = 0;
	Encoder_Init();
	
	while (1)
	{
		bsp_Idle();		/* 空闲时执行的函数,比如喂狗. 在bsp.c中 */
    USBH_Process(&USB_OTG_Core, &USB_Host);

		
/*
*********************************************************************************************************
*	按键扫描
*********************************************************************************************************
*/
		

 if(USART_RX_STA&0x8000)
 {
	if(USART_RX_BUF[5] == 0x20 && USART_RX_BUF[7] == 0x03)////开始暂停
		{
			ucFifoMode = 1;
			bsp_StartAutoTimer(0, 2);	

			add[0] = 0;
			add[1] = 0;
			add[2] = 0;
			add[3] = 0;
			add[4] = 0;
			add[5] = 0;
															
			ave[0] = 0;
			ave[1] = 0;
			ave[2] = 0;
			ave[3] = 0;
			ave[4] = 0;
			ave[5] = 0;
			ave[6] = 0;

		}
	
 if(USART_RX_BUF[5] == 0x10 && USART_RX_BUF[7] == 0x01)//停止采集
		{																	
			ucFifoMode = 3;
		}		
	USART_RX_STA=0;
	}
		 
/*
*********************************************************************************************************
*	Cortex-M3 内核异常中断服务程序
*********************************************************************************************************
*/
	

if (ucFifoMode == 3)	
	{
		bsp_StopTimer(1);
		AD7606_StopRecord();
		f_close(&file);
//		ucFifoMode = 0;
	}
		
/*
*********************************************************************************************************
*	计算标定值
*********************************************************************************************************
*/
										
			
			
	if (ucFifoMode == 1)	/* 计算500个数的均值 */
		{


			if (bsp_CheckTimer(0))
			{
				/* 每隔xms 进来一次. 由软件启动转换 */
				AD7606_ReadNowAdc();		/* 读取采样结果 */
				AD7606_StartConvst();		/* 启动下次转换 */
				ucRefresh = 1;	/* 刷新显示 */
			}
			
			if (ucRefresh == 1)
				{	
					ucRefresh = 0;
					reset++;
					if(reset>100&&reset<=600)
					{
					//计算幅值和编码器距离
					chan[0] = g_tAD7606.sNowAdc[0]*5000/32768;
					chan[1] = g_tAD7606.sNowAdc[1]*5000/32768;
					chan[2] = g_tAD7606.sNowAdc[2]*5000/32768;
					chan[3] = g_tAD7606.sNowAdc[3]*5000/32768;
					chan[4] = g_tAD7606.sNowAdc[4]*5000/32768;
					chan[5] = g_tAD7606.sNowAdc[5]*5000/32768;
					
					add[0] = add[0] + chan[0];
					add[1] = add[1] + chan[1];
					add[2] = add[2] + chan[2];
					add[3] = add[3] + chan[3];
					add[4] = add[4] + chan[4];
					add[5] = add[5] + chan[5];
					}									

					if(reset==600)
					{
						reset=0;
						ave[0] = add[0]/500;
						ave[1] = add[1]/500;
						ave[2] = add[2]/500;
						ave[3] = add[3]/500;
						ave[4] = add[4]/500;
						ave[5] = add[5]/500;
						ave[6] = ave[0]+ave[1]+ave[2]+ave[3]+ave[4]+ave[5];

						bsp_StopTimer(0);
						ucFifoMode = 2;
						//显示时间更改
						bsp_StartAutoTimer(1, 20);	
						
						
						
						
						f_mount(FS_USB, &fs);
						f_opendir(&DirInf, "/");
						f_open(&file, "采集数据.csv", FA_OPEN_ALWAYS | FA_WRITE);
						f_lseek(&file,f_size(&file));
						f_write(&file,  buffer, sizeof(buffer), &bw);
						f_sync(&file);
						
						//设置采集频率
						AD7606_StartRecord(2000);

					}
			  }

	}

			
/*
*********************************************************************************************************
*	采集存储
*********************************************************************************************************
*/
		

	if (ucFifoMode == 2)	/* AD7606 普通工作模式 */
	{

		
		if (bsp_CheckTimer(1))
			{
				ucRefresh = 2;	/* 刷新显示 */
			}

		
		if (ucRefresh == 2)
			{	
				ucRefresh = 0;

				//计算幅值和编码器距离
				chan[0] = g_tAD7606.sNowAdc[0]*5000/32768+200-ave[0];
				chan[1] = g_tAD7606.sNowAdc[1]*5000/32768+200-ave[1];
				chan[2] = g_tAD7606.sNowAdc[2]*5000/32768+200-ave[2];
				chan[3] = g_tAD7606.sNowAdc[3]*5000/32768+200-ave[3];
				chan[4] = g_tAD7606.sNowAdc[4]*5000/32768+200-ave[4];
				chan[5] = g_tAD7606.sNowAdc[5]*5000/32768+200-ave[5];
				sum = chan[0]+chan[1]+chan[2]+chan[3]+chan[4]+chan[5]-600;
/*
*********************************************************************************************************
*	无滤波发送
*********************************************************************************************************
*/				
											 //发送至串口屏
//								comSendChar(COM1, 0x5A);
//								comSendChar(COM1, 0xA5);
//								comSendChar(COM1, 0x10);
//								comSendChar(COM1, 0x84);
//								comSendChar(COM1, 0x7F);
//								comSendHalfword(COM1,sum);
//  							comSendHalfword(COM1,chan[0]);
//								comSendHalfword(COM1,chan[1]);
//								comSendHalfword(COM1,chan[2]);
//								comSendHalfword(COM1,chan[3]);
//								comSendHalfword(COM1,chan[4]);
//								comSendHalfword(COM1,chan[5]);
				
/*
*********************************************************************************************************
*	中值滤波发送
*********************************************************************************************************
*/
				
//        AD7606_Array();


//       if(biaozhi_lb==1)
//        {
//				  sum_lb = chan[0]+chan[1]+chan[2]+chan[3]+chan[4]+chan[5]-400;
//						AD7606_Array_1();
//					   if(biaozhi_lb_1==1)
//						 {					 
//							  comSendChar(COM1, 0x5A);
//								comSendChar(COM1, 0xA5);
//								comSendChar(COM1, 0x12);
//								comSendChar(COM1, 0x84);
//								comSendChar(COM1, 0xFF);
//								comSendHalfword(COM1,sum);
//  							comSendHalfword(COM1,chan[0]);
//								comSendHalfword(COM1,chan[1]);
//								comSendHalfword(COM1,chan[2]);
//								comSendHalfword(COM1,chan[3]);
//								comSendHalfword(COM1,chan[4]);
//								comSendHalfword(COM1,chan[5]);
//							 	comSendHalfword(COM1,sum_lb);
//							 
//								comSendChar(COM1, 0x5A);
//								comSendChar(COM1, 0xA5);
//								comSendChar(COM1, 0x07);
//								comSendChar(COM1, 0x82);
//								comSendChar(COM1, 0x00);
//								comSendChar(COM1, 0x00);
//								comSendword(COM1,len);
//							 
//							 biaozhi_lb_1=0;
//						 }
//							 
//					
//					biaozhi_lb=0;
//        }
/*
*********************************************************************************************************
*	巴特沃斯滤波发送
*********************************************************************************************************
*/			
       bartworth ();

//       if(biaozhi_bart == 1)
//        {
//										comSendChar(COM1, 0x5A);
//										comSendChar(COM1, 0xA5);
//										comSendChar(COM1, 0x12);
//										comSendChar(COM1, 0x84);
//										comSendChar(COM1, 0xFF);
//										comSendHalfword(COM1,sum);
//										comSendHalfword(COM1,chan[0]);
//										comSendHalfword(COM1,chan[1]);
//										comSendHalfword(COM1,chan[2]);
//										comSendHalfword(COM1,chan[3]);
//										comSendHalfword(COM1,chan[4]);
//										comSendHalfword(COM1,chan[5]);
//										comSendHalfword(COM1,sum);
//									 
//										comSendChar(COM1, 0x5A);
//										comSendChar(COM1, 0xA5);
//										comSendChar(COM1, 0x07);
//										comSendChar(COM1, 0x82);
//										comSendChar(COM1, 0x00);
//										comSendChar(COM1, 0x00);
//										comSendword(COM1,len);
//									 
//									 biaozhi_bart=0;
//        }	
       if(biaozhi_bart == 2)
        {
							  comSendChar(COM1, 0x5A);
								comSendChar(COM1, 0xA5);
								comSendChar(COM1, 0x12);
								comSendChar(COM1, 0x84);
								comSendChar(COM1, 0xFF);
								comSendHalfword(COM1,sum);
  							comSendHalfword(COM1,chan[0]);
								comSendHalfword(COM1,chan[1]);
								comSendHalfword(COM1,chan[2]);
								comSendHalfword(COM1,chan[3]);
								comSendHalfword(COM1,chan[4]);
								comSendHalfword(COM1,chan[5]);
							 	comSendHalfword(COM1,sum_lb_bart);
							 
								comSendChar(COM1, 0x5A);
								comSendChar(COM1, 0xA5);
								comSendChar(COM1, 0x07);
								comSendChar(COM1, 0x82);
								comSendChar(COM1, 0x00);
								comSendChar(COM1, 0x00);
								comSendword(COM1,len);
							 
							  comSendword(COM6,len);
							 	comSendHalfword(COM6,sum);
								comSendChar(COM6, 0xBB);
								comSendChar(COM6, 0xAA);
							 
							 biaozhi_bart=0;
        }				
				
				
				

			}					
	}

//写字符型
if(biaozhi == 1)
{
	channel[0] = g_tAD7606.sNowAdc[0]*5000/32768;
	channel[1] = g_tAD7606.sNowAdc[1]*5000/32768;
	channel[2] = g_tAD7606.sNowAdc[2]*5000/32768;
	channel[3] = g_tAD7606.sNowAdc[3]*5000/32768;
	channel[4] = g_tAD7606.sNowAdc[4]*5000/32768;
	channel[5] = g_tAD7606.sNowAdc[5]*5000/32768;
	sumchannel = channel[0]+channel[1]+channel[2]+channel[3]+channel[4]+channel[5];
	num=(circle_count-1)*2400+TIM_GetCounter(TIM4);
	len = num*80*3.14/4/600;

	sprintf(BUF8, " %d" , len);
	sprintf(BUF0, " %d" , channel[0]);
	sprintf(BUF1, " %d" , channel[1]);
	sprintf(BUF2, " %d" , channel[2]);
	sprintf(BUF3, " %d" , channel[3]);
	sprintf(BUF4, " %d" , channel[4]);
	sprintf(BUF5, " %d" , channel[5]);
	sprintf(BUF6, " %d" , sumchannel);
	
	if(Mark==end)
	{
		Mark=0;
		
	}
	//一次54-6=48个字符
	if(Mark<middle)
	{
		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = BUF8[0];
		g_tAdcFifo.usWrite1++;
		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = BUF8[1];
		g_tAdcFifo.usWrite1++;
		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = BUF8[2];
		g_tAdcFifo.usWrite1++;
		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = BUF8[3];
		g_tAdcFifo.usWrite1++;
		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = BUF8[4];
		g_tAdcFifo.usWrite1++;
		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = BUF8[5];
		g_tAdcFifo.usWrite1++;
		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = BUF8[6];
		g_tAdcFifo.usWrite1++;
		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = BUF8[7];
		g_tAdcFifo.usWrite1++;
		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = BUF8[8];
		g_tAdcFifo.usWrite1++;
		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = BUF8[9];
		g_tAdcFifo.usWrite1++;
		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = ',';
		g_tAdcFifo.usWrite1++;
//		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = BUF0[0];
//		g_tAdcFifo.usWrite1++;
		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = BUF0[1];
		g_tAdcFifo.usWrite1++;
		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = BUF0[2];
		g_tAdcFifo.usWrite1++;
		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = BUF0[3];
		g_tAdcFifo.usWrite1++;
		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = BUF0[4];
		g_tAdcFifo.usWrite1++;
		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = ',';
		g_tAdcFifo.usWrite1++;
//		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = BUF1[0];
//		g_tAdcFifo.usWrite1++;
		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = BUF1[1];
		g_tAdcFifo.usWrite1++;
		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = BUF1[2];
		g_tAdcFifo.usWrite1++;
		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = BUF1[3];
		g_tAdcFifo.usWrite1++;
		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = BUF1[4];
		g_tAdcFifo.usWrite1++;
		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = ',';
		g_tAdcFifo.usWrite1++;
//		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = BUF2[0];
//		g_tAdcFifo.usWrite1++;
		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = BUF2[1];
		g_tAdcFifo.usWrite1++;
		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = BUF2[2];
		g_tAdcFifo.usWrite1++;
		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = BUF2[3];
		g_tAdcFifo.usWrite1++;
		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = BUF2[4];
		g_tAdcFifo.usWrite1++;
		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = ',';
		g_tAdcFifo.usWrite1++;	
//		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = BUF3[0];
//		g_tAdcFifo.usWrite1++;
		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = BUF3[1];
		g_tAdcFifo.usWrite1++;
		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = BUF3[2];
		g_tAdcFifo.usWrite1++;
		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = BUF3[3];
		g_tAdcFifo.usWrite1++;
		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = BUF3[4];
		g_tAdcFifo.usWrite1++;
		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = ',';
		g_tAdcFifo.usWrite1++;
//		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = BUF4[0];
//		g_tAdcFifo.usWrite1++;
		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = BUF4[1];
		g_tAdcFifo.usWrite1++;
		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = BUF4[2];
		g_tAdcFifo.usWrite1++;
		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = BUF4[3];
		g_tAdcFifo.usWrite1++;
		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = BUF4[4];
		g_tAdcFifo.usWrite1++;
		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = ',';
		g_tAdcFifo.usWrite1++;
//		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = BUF5[0];
//		g_tAdcFifo.usWrite1++;
		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = BUF5[1];
		g_tAdcFifo.usWrite1++;
		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = BUF5[2];
		g_tAdcFifo.usWrite1++;
		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = BUF5[3];
		g_tAdcFifo.usWrite1++;
		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = BUF5[4];
		g_tAdcFifo.usWrite1++;
		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = ',';
		g_tAdcFifo.usWrite1++;
//		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = BUF6[0];
//		g_tAdcFifo.usWrite1++;
		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = BUF6[1];
		g_tAdcFifo.usWrite1++;
		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = BUF6[2];
		g_tAdcFifo.usWrite1++;
		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = BUF6[3];
		g_tAdcFifo.usWrite1++;
		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = BUF6[4];
		g_tAdcFifo.usWrite1++;
		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = BUF6[5];
		g_tAdcFifo.usWrite1++;
		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = '\n';
		g_tAdcFifo.usWrite1++;

	}
	if(Mark >= middle && Mark < end)
	{
		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = BUF8[0];
		g_tAdcFifo.usWrite2++;
		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = BUF8[1];
		g_tAdcFifo.usWrite2++;
		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = BUF8[2];
		g_tAdcFifo.usWrite2++;
		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = BUF8[3];
		g_tAdcFifo.usWrite2++;
		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = BUF8[4];
		g_tAdcFifo.usWrite2++;
		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = BUF8[5];
		g_tAdcFifo.usWrite2++;
		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = BUF8[6];
		g_tAdcFifo.usWrite2++;
		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = BUF8[7];
		g_tAdcFifo.usWrite2++;
		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = BUF8[8];
		g_tAdcFifo.usWrite2++;
		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = BUF8[9];
		g_tAdcFifo.usWrite2++;
		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = ',';
		g_tAdcFifo.usWrite2++;
//		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = BUF0[0];
//		g_tAdcFifo.usWrite2++;
		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = BUF0[1];
		g_tAdcFifo.usWrite2++;
		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = BUF0[2];
		g_tAdcFifo.usWrite2++;
		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = BUF0[3];
		g_tAdcFifo.usWrite2++;
		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = BUF0[4];
		g_tAdcFifo.usWrite2++;
		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = ',';
		g_tAdcFifo.usWrite2++;
//		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = BUF1[0];
//		g_tAdcFifo.usWrite2++;
		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = BUF1[1];
		g_tAdcFifo.usWrite2++;
		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = BUF1[2];
		g_tAdcFifo.usWrite2++;
		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = BUF1[3];
		g_tAdcFifo.usWrite2++;
		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = BUF1[4];
		g_tAdcFifo.usWrite2++;
		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = ',';
		g_tAdcFifo.usWrite2++;
//		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = BUF2[0];
//		g_tAdcFifo.usWrite2++;
		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = BUF2[1];
		g_tAdcFifo.usWrite2++;
		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = BUF2[2];
		g_tAdcFifo.usWrite2++;
		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = BUF2[3];
		g_tAdcFifo.usWrite2++;
		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = BUF2[4];
		g_tAdcFifo.usWrite2++;
		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = ',';
		g_tAdcFifo.usWrite2++;	
//		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = BUF3[0];
//		g_tAdcFifo.usWrite2++;
		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = BUF3[1];
		g_tAdcFifo.usWrite2++;
		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = BUF3[2];
		g_tAdcFifo.usWrite2++;
		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = BUF3[3];
		g_tAdcFifo.usWrite2++;
		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = BUF3[4];
		g_tAdcFifo.usWrite2++;
		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = ',';
		g_tAdcFifo.usWrite2++;
//		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = BUF4[0];
//		g_tAdcFifo.usWrite2++;
		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = BUF4[1];
		g_tAdcFifo.usWrite2++;
		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = BUF4[2];
		g_tAdcFifo.usWrite2++;
		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = BUF4[3];
		g_tAdcFifo.usWrite2++;
		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = BUF4[4];
		g_tAdcFifo.usWrite2++;
		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = ',';
		g_tAdcFifo.usWrite2++;
//		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = BUF5[0];
//		g_tAdcFifo.usWrite2++;
		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = BUF5[1];
		g_tAdcFifo.usWrite2++;
		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = BUF5[2];
		g_tAdcFifo.usWrite2++;
		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = BUF5[3];
		g_tAdcFifo.usWrite2++;
		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = BUF5[4];
		g_tAdcFifo.usWrite2++;
		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = ',';
		g_tAdcFifo.usWrite2++;
//		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = BUF6[0];
//		g_tAdcFifo.usWrite2++;
		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = BUF6[1];
		g_tAdcFifo.usWrite2++;
		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = BUF6[2];
		g_tAdcFifo.usWrite2++;
		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = BUF6[3];
		g_tAdcFifo.usWrite2++;
		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = BUF6[4];
		g_tAdcFifo.usWrite2++;
		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = BUF6[5];
		g_tAdcFifo.usWrite2++;
		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = '\n';
		g_tAdcFifo.usWrite2++;		

	}

	Mark++;
	
	if(Mark == middle)
	{

		Calibration = 1;
 		g_tAdcFifo.usWrite1 = 0;
	
	}
		if(Mark == end)
	{

		Calibration = 2;
 		g_tAdcFifo.usWrite2 = 0;
		
	}
	
	
		biaozhi = 0;


}


////写16进制数

//if(biaozhi == 1)
//{
//	num=(circle_count-1)*2400+TIM_GetCounter(TIM4);
//	len = num*80*3.14/4/600;

//	if(Mark==end)
//	{
//		Mark=0;
//		
//	}
//	//一次15个int16_t
//	if(Mark<middle)
//	{
//		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = (len>>16);					
//		g_tAdcFifo.usWrite1++;
//		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = len;
//		g_tAdcFifo.usWrite1++;
//		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = ',';
//		g_tAdcFifo.usWrite1++;
//		
//		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = g_tAD7606.sNowAdc[0];
//		g_tAdcFifo.usWrite1++;
//		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = ',';
//		g_tAdcFifo.usWrite1++;
//		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = g_tAD7606.sNowAdc[1];
//		g_tAdcFifo.usWrite1++;
//		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = ',';
//		g_tAdcFifo.usWrite1++;
//		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = g_tAD7606.sNowAdc[2];
//		g_tAdcFifo.usWrite1++;
//		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = ',';
//		g_tAdcFifo.usWrite1++;
//		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = g_tAD7606.sNowAdc[3];
//		g_tAdcFifo.usWrite1++;
//		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = ',';
//		g_tAdcFifo.usWrite1++;	
//		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = g_tAD7606.sNowAdc[4];
//		g_tAdcFifo.usWrite1++;
//		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = ',';
//		g_tAdcFifo.usWrite1++;
//		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = g_tAD7606.sNowAdc[5];
//		g_tAdcFifo.usWrite1++;
//		g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1] = '\n';
//		g_tAdcFifo.usWrite1++;

//	}
//	if(Mark >= middle && Mark < end)
//	{
//		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = (len>>16);					
//		g_tAdcFifo.usWrite2++;
//		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = len;
//		g_tAdcFifo.usWrite2++;
//		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = ',';
//		g_tAdcFifo.usWrite2++;
//		
//		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = g_tAD7606.sNowAdc[0];
//		g_tAdcFifo.usWrite2++;
//		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = ',';
//		g_tAdcFifo.usWrite2++;
//		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = g_tAD7606.sNowAdc[1];
//		g_tAdcFifo.usWrite2++;
//		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = ',';
//		g_tAdcFifo.usWrite2++;
//		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = g_tAD7606.sNowAdc[2];
//		g_tAdcFifo.usWrite2++;
//		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = ',';
//		g_tAdcFifo.usWrite2++;		
//		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = g_tAD7606.sNowAdc[3];
//		g_tAdcFifo.usWrite2++;
//		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = ',';
//		g_tAdcFifo.usWrite2++;
//		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = g_tAD7606.sNowAdc[4];
//		g_tAdcFifo.usWrite2++;
//		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = ',';
//		g_tAdcFifo.usWrite2++;
//		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = g_tAD7606.sNowAdc[5];
//		g_tAdcFifo.usWrite2++;
//		g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2] = ',';
//		g_tAdcFifo.usWrite2++;

//	}

//	Mark++;
//	
//	if(Mark == middle)
//	{

//		Calibration = 1;
// 		g_tAdcFifo.usWrite1 = 0;
//	
//	}
//		if(Mark == end)
//	{

//		Calibration = 2;
// 		g_tAdcFifo.usWrite2 = 0;
//		
//	}
//	
//	
//		biaozhi = 0;


//}
	
	
	
	
	
if(Calibration == 1)
{
	f_lseek(&file,f_size(&file));
	f_write(&file, g_tAdcFifo.sBuf1, sizeof(g_tAdcFifo.sBuf1), &bw);
	f_sync(&file);
	memset(g_tAdcFifo.sBuf1,0,sizeof(g_tAdcFifo.sBuf1));
	Calibration = 0;
}
if(Calibration == 2)
{
	f_lseek(&file,f_size(&file));
	f_write(&file, g_tAdcFifo.sBuf2, sizeof(g_tAdcFifo.sBuf2), &bw);
	f_sync(&file);
	memset(g_tAdcFifo.sBuf2,0,sizeof(g_tAdcFifo.sBuf2));
	Calibration = 0;
}

					
			

}//while函数的括号

}//main函数的括号




void AD7606_Array(void)
{
	  int l_6;
		value_buf0[count]=chan[0];
		value_buf1[count]=chan[1];
		value_buf2[count]=chan[2];
		value_buf3[count]=chan[3]; 
		value_buf4[count]=chan[4];
		value_buf5[count]=chan[5]; 
//		value_buf6[count]=sum;
   
	  count++;
	
	if(count%N==0)
	{
		AD7606_Filter();
		biaozhi_lb=1;
		for( l_6=0;l_6<N-1;l_6++)
		{
			value_buf0[l_6]=value_buf0[l_6+1];
			value_buf1[l_6]=value_buf1[l_6+1];
			value_buf2[l_6]=value_buf2[l_6+1];
			value_buf3[l_6]=value_buf3[l_6+1];
			value_buf4[l_6]=value_buf4[l_6+1];
			value_buf5[l_6]=value_buf5[l_6+1];
//			value_buf6[l]=value_buf6[l+1];
		}
		count=N-1;
	}

}

void AD7606_Filter(void)
{ 
	uint8_t p, j;
	int16_t temp_lb;
	int16_t copy_0[N] = {0};
	int16_t copy_1[N] = {0};
	int16_t copy_2[N] = {0};
	int16_t copy_3[N] = {0};
	int16_t copy_4[N] = {0};
	int16_t copy_5[N] = {0};	
//	int16_t copy_6[N] = {0};	
	
	
  memcpy(copy_0, value_buf0, sizeof(int16_t) * N);
  memcpy(copy_1, value_buf1, sizeof(int16_t) * N);
  memcpy(copy_2, value_buf2, sizeof(int16_t) * N);				
  memcpy(copy_3, value_buf3, sizeof(int16_t) * N);
  memcpy(copy_4, value_buf4, sizeof(int16_t) * N);	
  memcpy(copy_5, value_buf5, sizeof(int16_t) * N);	
//  memcpy(copy_6, value_buf6, sizeof(int16_t) * N);	
  
	//通道0*************************************************************//
	for (j=0; j<N-1; j++)		
	{
		for (p=0; p<N-j-1; p++)
		{
			if (copy_0[p] > copy_0[p+1])
			{
				temp_lb = copy_0[p];
				copy_0[p] = copy_0[p+1];
				copy_0[p+1] = temp_lb;
			}
		}
	}
	//通道1*************************************************************//
	for (j=0; j<N-1; j++)		
	{
		for (p=0; p<N-j-1; p++)
		{
			if (copy_1[p] > copy_1[p+1])
			{
				temp_lb = copy_1[p];
				copy_1[p] = copy_1[p+1];
				copy_1[p+1] = temp_lb;
			}
		}
	}
 //通道2*************************************************************//
	for (j=0; j<N-1; j++)		
	{
		for (p=0; p<N-j-1; p++)
		{
			if (copy_2[p] > copy_2[p+1])
			{
				temp_lb = copy_2[p];
				copy_2[p] = copy_2[p+1];
				copy_2[p+1] = temp_lb;
			}
		}
	}
	//通道3*************************************************************//
	for (j=0; j<N-1; j++)		
	{
		for (p=0; p<N-j-1; p++)
		{
			if (copy_3[p] > copy_3[p+1])
			{
				temp_lb = copy_3[p];
				copy_3[p] = copy_3[p+1];
				copy_3[p+1] = temp_lb;
			}
		}
	}
	//通道4*************************************************************//
	for (j=0; j<N-1; j++)		
	{
		for (p=0; p<N-j-1; p++)
		{
			if (copy_4[p] > copy_4[p+1])
			{
				temp_lb = copy_4[p];
				copy_4[p] = copy_4[p+1];
				copy_4[p+1] = temp_lb;
			}
		}
	}
	//通道5*************************************************************//
	for (j=0; j<N-1; j++)		
	{
		for (p=0; p<N-j-1; p++)
		{
			if (copy_5[p] > copy_5[p+1])
			{
				temp_lb = copy_5[p];
				copy_5[p] = copy_5[p+1];
				copy_5[p+1] = temp_lb;
			}
		}
	}
	//通道和*************************************************************//
//	for (j=0; j<N-1; j++)		
//	{
//		for (p=0; p<N-j-1; p++)
//		{
//			if (copy_6[p] > copy_6[p+1])
//			{
//				temp_lb = copy_6[p];
//				copy_6[p] = copy_6[p+1];
//				copy_6[p+1] = temp_lb;
//			}
//		}
//	}
	
	
	chan[0] = copy_0[(N-1)/2];
	chan[1] = copy_1[(N-1)/2];
	chan[2] = copy_2[(N-1)/2];
	chan[3] = copy_3[(N-1)/2];	
	chan[4] = copy_4[(N-1)/2];
	chan[5] = copy_5[(N-1)/2];	
//	sum = copy_6[(N-1)/2];
	
	
	
}


void AD7606_Array_1(void)
{
	  int l_1;
		value_buf6[count_1]=sum_lb;  
	  count_1++;
	
	if(count_1%M==0)
	{
		AD7606_Filter_1();
		biaozhi_lb_1=1;
		for( l_1=0;l_1<M-1;l_1++)
		{
			value_buf6[l_1]=value_buf6[l_1+1];
		}
		count_1=M-1;
	}

}

void AD7606_Filter_1(void)
{ 
	uint8_t p, j;
	int16_t temp_lb;	
	int16_t copy_6[M] = {0};	

  memcpy(copy_6, value_buf6, sizeof(int16_t) * N);	

	//通道和*************************************************************//
	for (j=0; j<M-1; j++)		
	{
		for (p=0; p<M-j-1; p++)
		{
			if (copy_6[p] > copy_6[p+1])
			{
				temp_lb = copy_6[p];
				copy_6[p] = copy_6[p+1];
				copy_6[p+1] = temp_lb;
			}
		}
	}
	
	
	sum_lb = copy_6[(M-1)/2];
	
	
	
}

void bartworth (void)
{
	bart_count++;
	int bart_l;
	bart_x [bart_count_x] = sum;
	bart_count_x++;
	
	if(bart_count_x == 1)
	{
		bart_y[0] = bart_b[0] * bart_x[0];
		
	}
	
	if(bart_count_x == 2)
	{
		bart_y[1] = (bart_b[0] * bart_x[1] + bart_b[1] * bart_x[0]) -    
		            (bart_a[1] * bart_y[0]);
		
	}
	
	if(bart_count_x == 3)
	{
		bart_y[2] = (bart_b[0] * bart_x[2] + bart_b[1] * bart_x[1] + bart_b[2] * bart_x[0]) - 
		            (bart_a[1] * bart_y[1] + bart_a[2] * bart_y[0]);
		
	}
	
	if(bart_count_x == 4)
	{
		bart_y[3] = (bart_b[0] * bart_x[3] + bart_b[1] * bart_x[2] + bart_b[2] * bart_x[1]+ bart_b[3] * bart_x[0]) - 
		            (bart_a[1] * bart_y[2]+bart_a[2] * bart_y[1]+bart_a[3] * bart_y[0]);

	}
	
	if(bart_count_x == 5)
	{
		bart_y[4] = (bart_b[0] * bart_x[4] + bart_b[1] * bart_x[3] + bart_b[2] * bart_x[2]+ bart_b[3] * bart_x[1]+ bart_b[4] * bart_x[0])- 
		            (bart_a[1] * bart_y[3] + bart_a[2] * bart_y[2] + bart_a[3] * bart_y[1]+ bart_a[4] * bart_y[0]);

	}		
	if(bart_count_x == 6)
	{
		bart_y[5] = (bart_b[0] * bart_x[5] + bart_b[1] * bart_x[4] + bart_b[2] * bart_x[3]+ bart_b[3] * bart_x[2]+ bart_b[4] * bart_x[1]+
		             bart_b[5] * bart_x[0])- 
		            (bart_a[1] * bart_y[4] + bart_a[2] * bart_y[3] + bart_a[3] * bart_y[2]+ bart_a[4] * bart_y[1] +
		             bart_a[5] * bart_y[0]);

	}		
	if(bart_count_x == 7)
	{
		bart_y[6] = (bart_b[0] * bart_x[6] + bart_b[1] * bart_x[5] + bart_b[2] * bart_x[4]+ bart_b[3] * bart_x[3]+ bart_b[4] * bart_x[2]+
		             bart_b[5] * bart_x[1] + bart_b[6] * bart_x[0] )- 
		            (bart_a[1] * bart_y[5] + bart_a[2] * bart_y[4] + bart_a[3] * bart_y[3]+ bart_a[4] * bart_y[2] +
		             bart_a[5] * bart_y[1] + bart_a[6] * bart_y[0] );

	}		
	if(bart_count_x == 8)
	{
		bart_y[7] = (bart_b[0] * bart_x[7] + bart_b[1] * bart_x[6] + bart_b[2] * bart_x[5]+ bart_b[3] * bart_x[4]+ bart_b[4] * bart_x[3]+
		             bart_b[5] * bart_x[2] + bart_b[6] * bart_x[1] + bart_b[7] * bart_x[0])- 
		            (bart_a[1] * bart_y[6] + bart_a[2] * bart_y[5] + bart_a[3] * bart_y[4]+ bart_a[4] * bart_y[3] +
		             bart_a[5] * bart_y[2] + bart_a[6] * bart_y[1] + bart_a[7] * bart_y[0]);

	}	
	
	
	
	if(bart_count_x%9 == 0)
	{
		bart_y[8] = (bart_b[0] * bart_x[8] + bart_b[1] * bart_x[7] + bart_b[2] * bart_x[6]+ bart_b[3] * bart_x[5]+ bart_b[4] * bart_x[4]+
		             bart_b[5] * bart_x[3] + bart_b[6] * bart_x[2] + bart_b[7] * bart_x[1]+ bart_b[8] * bart_x[0])- 
		            (bart_a[1] * bart_y[7] + bart_a[2] * bart_y[6] + bart_a[3] * bart_y[5]+ bart_a[4] * bart_y[4] +
		             bart_a[5] * bart_y[3] + bart_a[6] * bart_y[2] + bart_a[7] * bart_y[1]+ bart_a[8] * bart_y[0]);

		if(bart_count < 500)
		{
			biaozhi_bart = 1;
			
		}	
		
		if(bart_count > 500)
		{
			biaozhi_bart = 2;
			sum_lb_bart = (int16_t)(bart_y[8]+200);
			
		}
		for( bart_l=0;bart_l<8;bart_l++)
		{
			bart_x[bart_l]=bart_x[bart_l+1];
			bart_y[bart_l]=bart_y[bart_l+1];
		}
		bart_count_x = 8;
		
	}	
	
	

	
	
	 

}


