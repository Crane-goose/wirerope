#include "bsp.h"				/* 底层硬件驱动 */
#include "demo_fatfs.h"
#include "ff.h"			        /* FatFS文件系统模块*/
#include "usbh_bsp_msc.h"
#include "math.h"


void DewenSet(void);
void SaveDate(void);
void HandleDate(void);
void EncoderMode(void);
void FreqMode(void);
void HandleDateEncoder(void);
void SaveFile(void);
void SaveBuffer(void);


uint8_t count=0;
uint8_t count_1=0;
	
	
int16_t sum;
int16_t chan[6];




uint8_t ucRefresh = 0;
uint8_t ucFifoMode =0;//1;均值 2：采集 3：停止
uint16_t reset = 0;
int32_t add[6];
int16_t ave[7];
char buffer[48] = "位置,通道1,通道2,通道3,通道4,通道5,通道6,和通道\n" ;
int16_t middle = 150;
int16_t end = 300;

uint16_t Mark = 0;
uint8_t Calibration = 0;
int16_t channel[6];
int16_t sumchannel;
int32_t num;
int32_t len;

char bufferSaveChannel[6][5];
char bufferSaveSum[6];
char bufferSaveLen[10];


FATFS fs;
FIL file;
DIR DirInf;
uint32_t bw;

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

int main(void)
{
	delay_us(100000);//短暂延时，确保外设供电	
	bsp_Init();
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
        
        EncoderMode();
        
        //更换需在encoder文件中更改计数
        //FreqMode();
        

    }

}

void EncoderMode(void)
{
    DewenSet();
    HandleDateEncoder();
}

void FreqMode(void)
{
    DewenSet();
    HandleDate();
    SaveDate();   
}

void DewenSet(void)
{
    if(USART_RX_STA&0x8000)
    {
        if(USART_RX_BUF[5] == 0x20 && USART_RX_BUF[7] == 0x03)//开始采集
        {
            len = 0;
            ucFifoMode = 1;
            bsp_StartAutoTimer(0, 2);	
            for (int i = 0; i < 6; i++) 
            {
                add[i] = 0;
                ave[i] = 0;
            }
            ave[6] = 0;
        }

        if(USART_RX_BUF[5] == 0x10 && USART_RX_BUF[7] == 0x01)//停止采集
        {															
            ucFifoMode = 3;
        }		
        USART_RX_STA=0;
    }
}

void HandleDateEncoder(void)
{
    //执行停止按钮
    if (ucFifoMode == 3)	
    {
        bsp_StopTimer(1);
        AD7606_StopRecord();
        f_close(&file);
    }	
    //执行运行按钮
    if (ucFifoMode == 1)/* 计算500个数的均值 */
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
                for(int i = 0; i < 6; i++)
                {
                    chan[i] = g_tAD7606.sNowAdc[i]*5000/32768;
                }
                
                for (int i = 0; i < 6; i++) 
                {
                    add[i] += chan[i];
                }
            }									
            if(reset==600)
            {
                reset=0;
                for(int i = 0; i < 6; i++) 
                {
                    ave[i] = add[i]/500;
                }
                ave[6] = ave[0]+ave[1]+ave[2]+ave[3]+ave[4]+ave[5];

                bsp_StopTimer(0);
                ucFifoMode = 2;	                                
                f_mount(FS_USB, &fs);
                f_opendir(&DirInf, "/");
                f_open(&file, "采集数据.csv", FA_OPEN_ALWAYS | FA_WRITE);
                f_lseek(&file,f_size(&file));
                f_write(&file,  buffer, sizeof(buffer), &bw);
                f_sync(&file);                
            }
        }

	}
	if (ucFifoMode == 2)	/* AD7606 普通工作模式 */
	{
        if (flagEncoder == 1) 
        {
            flagEncoder = 0;
            AD7606_ReadNowAdc();		/* 读取采样结果 */
            AD7606_StartConvst();		/* 启动下次转换 */
            //计算幅值和编码器距离
            for (int i = 0; i < 6; i++) 
            {
                chan[i] = g_tAD7606.sNowAdc[i]*5000/32768 + 200 - ave[i];
                channel[i] = g_tAD7606.sNowAdc[i]*5000/32768;
            }
            sum = chan[0]+chan[1]+chan[2]+chan[3]+chan[4]+chan[5]-600;
            sumchannel = channel[0]+channel[1]+channel[2]+channel[3]+channel[4]+channel[5];
            len++;
            //发送曲线
            comSendChar(COM1, 0x5A);
            comSendChar(COM1, 0xA5);
            comSendChar(COM1, 0x10);
            comSendChar(COM1, 0x84);
            comSendChar(COM1, 0x7F);
            comSendHalfword(COM1,sum);
            for (int i = 0; i < 6; i++)
            {
                comSendHalfword(COM1,chan[i]);
            }            
             //发送位置
            comSendChar(COM1, 0x5A);
            comSendChar(COM1, 0xA5);
            comSendChar(COM1, 0x07);
            comSendChar(COM1, 0x82);
            comSendChar(COM1, 0x00);
            comSendChar(COM1, 0x00);
            comSendword(COM1,len);
            
            //保存数据
            //打印成字符串类型
            sprintf(bufferSaveLen, " %d" , len);
            for(int i = 0; i < 6; i++)
            {
               sprintf(bufferSaveChannel[i], " %d" , channel[i]); 
            }
            sprintf(bufferSaveSum, " %d" , sumchannel);  
            
            SaveBuffer();
            SaveFile();
            

        }
     					
	}
}


void HandleDate(void)
{
    //执行停止按钮
    if (ucFifoMode == 3)	
    {
        bsp_StopTimer(1);
        AD7606_StopRecord();
        f_close(&file);
    }	
    //执行运行按钮
    if (ucFifoMode == 1)/* 计算500个数的均值 */
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
                for(int i = 0; i < 6; i++)
                {
                    chan[i] = g_tAD7606.sNowAdc[i]*5000/32768;
                }
                
                for (int i = 0; i < 6; i++) 
                {
                    add[i] += chan[i];
                }
            }									
            if(reset==600)
            {
                reset=0;
                for(int i = 0; i < 6; i++) 
                {
                    ave[i] = add[i]/500;
                }
                ave[6] = ave[0]+ave[1]+ave[2]+ave[3]+ave[4]+ave[5];

                bsp_StopTimer(0);
                ucFifoMode = 2;
                //显示时间更改
                bsp_StartAutoTimer(1, 2);	
                                
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
            for (int i = 0; i < 6; i++) 
            {
                chan[i] = g_tAD7606.sNowAdc[i]*5000/32768+200-ave[i];
            }
            sum = chan[0]+chan[1]+chan[2]+chan[3]+chan[4]+chan[5]-600;
            //发送曲线
            comSendChar(COM1, 0x5A);
            comSendChar(COM1, 0xA5);
            comSendChar(COM1, 0x10);
            comSendChar(COM1, 0x84);
            comSendChar(COM1, 0x7F);
            comSendHalfword(COM1,sum);
            for (int i = 0; i < 6; i++)
            {
                comSendHalfword(COM1,chan[i]);
            }	
            //发送位置
            comSendChar(COM1, 0x5A);
            comSendChar(COM1, 0xA5);
            comSendChar(COM1, 0x07);
            comSendChar(COM1, 0x82);
            comSendChar(COM1, 0x00);
            comSendChar(COM1, 0x00);
            comSendword(COM1,len);
        }					
	}
}    

void SaveDate(void)
{
    //写字符型
    if(biaozhi == 1)
    {
        for (int i = 0; i < 6; i++)
        {
            channel[i] = g_tAD7606.sNowAdc[i]*5000/32768;
        }
        sumchannel = channel[0]+channel[1]+channel[2]+channel[3]+channel[4]+channel[5];
        num=(circle_count-1)*2400+TIM_GetCounter(TIM4);
        len = num*80*3.14/4/600;

        sprintf(bufferSaveLen, " %d" , len);
        for(int i = 0; i < 6; i++)
        {
           sprintf(bufferSaveChannel[i], " %d" , channel[i]); 
        }
        sprintf(bufferSaveSum, " %d" , sumchannel);
        
        SaveBuffer();     
        biaozhi = 0;

    }
	SaveFile();

}


void SaveBuffer(void)
{
    if(Mark==end)
    {
        Mark=0;
        
    }
    //一次54-6=48个字符
    if(Mark<middle)
    {           
        for(int i = 0; i <10; i++)
        {
            g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1++] = bufferSaveLen[i];
            
        }
        g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1++] = ',';
        for(int j = 0; j < 6; j++) 
        {
            for(int i = 1; i < 5; i++)
            {
                g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1++] = bufferSaveChannel[j][i];
            }
            g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1++] = ',';
        }
        for(int i = 1; i < 6; i++)
        {
            g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1++] = bufferSaveSum[i];
        }               
        g_tAdcFifo.sBuf1[g_tAdcFifo.usWrite1++] = '\n';
    }
    if(Mark >= middle && Mark < end)
    {
        for(int i = 0; i <10; i++)
        {
            g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2++] = bufferSaveLen[i];
            
        }
        g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2++] = ',';
        for(int j = 0; j < 6; j++) 
        {
            for(int i = 1; i < 5; i++)
            {
                g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2++] = bufferSaveChannel[j][i];
            }
            g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2++] = ',';
        }
        for(int i = 1; i < 6; i++)
        {
            g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2++] = bufferSaveSum[i];
        }               
        g_tAdcFifo.sBuf2[g_tAdcFifo.usWrite2++] = '\n';
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
}

void SaveFile(void)
{
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
}

