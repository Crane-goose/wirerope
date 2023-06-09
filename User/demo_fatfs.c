/*
*********************************************************************************************************
*
*	模块名称 : SD卡Fat文件系统演示模块。
*	文件名称 : demo_sdio_fatfs.c
*	版    本 : V1.0
*	说    明 : 该例程移植FatFS文件系统（版本 R0.09b），演示如何创建文件、读取文件、创建目录和删除文件
*			并测试了文件读写速度。
*
*	修改记录 :
*		版本号  日期        作者     说明
*		V1.0    2013-02-01 armfly  正式发布
*
*	Copyright (C), 2013-2014, 安富莱电子 www.armfly.com
*
*********************************************************************************************************
*/

#include "bsp.h"
#include "ff.h"			/* FatFS文件系统模块*/
#include "demo_fatfs.h"
#include "usbh_bsp_msc.h"

/*
*********************************************************************************************************
*	函 数 名: DemoFatFS
*	功能说明: FatFS文件系统演示主程序
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void DemoFatFS(void)
{

	  /* Init Host Library */
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

	
		bsp_Idle();		/* 这个函数在bsp.c文件。用户可以修改这个函数实现CPU休眠和喂狗 */

		USBH_Process(&USB_OTG_Core, &USB_Host);
	
}

void CreateNewFile(void)
{
	/* 本函数使用的局部变量占用较多，请修改启动文件，保证堆栈空间够用 */
//	FRESULT result;
	FATFS fs;
	FIL file;
	DIR DirInf;
	uint32_t bw;
	
	
	f_mount(FS_USB, &fs);
	f_opendir(&DirInf, "/");
	f_open(&file, "armfly.txt", FA_CREATE_ALWAYS | FA_WRITE);
	f_write(&file, "FatFS Write Demo \r\n www.armfly.com \r\n", 34, &bw);
	f_write(&file, "FatFS Write Demo \r\n www.armfly.com \r\n", 34, &bw);
	f_write(&file, "FatFS Write Demo \r\n www.armfly.com \r\n", 34, &bw);
	f_close(&file);
	f_mount(FS_USB, NULL);
	
	

// 	/* 挂载文件系统 */
//	result = f_mount(FS_USB, &fs);			/* Mount a logical drive */
////	if (result != FR_OK)
////	{
////		printf("挂载文件系统失败 (%d)\r\n", result);
////	}

//	/* 打开根文件夹 */
//	result = f_opendir(&DirInf, "/"); /* 如果不带参数，则从当前目录开始 */
////	if (result != FR_OK)
////	{
////		printf("打开根目录失败 (%d)\r\n", result);
////		return;
////	}

//	/* 打开文件 */
//	result = f_open(&file, "armfly.txt", FA_CREATE_ALWAYS | FA_WRITE);

//	/* 写一串数据 */
//	result = f_write(&file, "FatFS Write Demo \r\n www.armfly.com \r\n", 34, &bw);
////	if (result == FR_OK)
////	{
////		printf("armfly.txt 文件写入成功\r\n");
////	}
////	else
////	{
////		printf("armfly.txt 文件写入失败\r\n");
////	}

//	/* 关闭文件*/
//	f_close(&file);

//	/* 卸载文件系统 */
//	f_mount(FS_USB, NULL);
}



///*
//*********************************************************************************************************
//*	函 数 名: ViewRootDir
//*	功能说明: 显示SD卡根目录下的文件名
//*	形    参：无
//*	返 回 值: 无
//*********************************************************************************************************
//*/
//static void ViewRootDir(void)
//{
//	/* 本函数使用的局部变量占用较多，请修改启动文件，保证堆栈空间够用 */
//	FRESULT result;
//	FATFS fs;
//	DIR DirInf;
//	FILINFO FileInf;
//	uint32_t cnt = 0;
//	char lfname[256];

// 	/* 挂载文件系统 */
//	result = f_mount(FS_USB, &fs);	/* Mount a logical drive */
//	if (result != FR_OK)
//	{
//		printf("挂载文件系统失败 (%d)\r\n", result);
//	}

//	/* 打开根文件夹 */
//	result = f_opendir(&DirInf, "/"); /* 如果不带参数，则从当前目录开始 */
//	if (result != FR_OK)
//	{
//		printf("打开根目录失败 (%d)\r\n", result);
//		return;
//	}

//	/* 读取当前文件夹下的文件和目录 */
//	FileInf.lfname = lfname;
//	FileInf.lfsize = 256;

//	printf("属性        |  文件大小 | 短文件名 | 长文件名\r\n");
//	for (cnt = 0; ;cnt++)
//	{
//		result = f_readdir(&DirInf,&FileInf); 		/* 读取目录项，索引会自动下移 */
//		if (result != FR_OK || FileInf.fname[0] == 0)
//		{
//			break;
//		}

//		if (FileInf.fname[0] == '.')
//		{
//			continue;
//		}

//		/* 判断是文件还是子目录 */
//		if (FileInf.fattrib & AM_DIR)
//		{
//			printf("(0x%02d)目录  ", FileInf.fattrib);
//		}
//		else
//		{
//			printf("(0x%02d)文件  ", FileInf.fattrib);
//		}

//		/* 打印文件大小, 最大4G */
//		printf(" %10d", FileInf.fsize);

//		printf("  %s |", FileInf.fname);	/* 短文件名 */

//		printf("  %s\r\n", (char *)FileInf.lfname);	/* 长文件名 */
//	}

//	/* 卸载文件系统 */
//	f_mount(FS_USB, NULL);
//}

///*
//*********************************************************************************************************
//*	函 数 名: CreateNewFile
//*	功能说明: 在SD卡创建一个新文件，文件内容填写“www.armfly.com”
//*	形    参：无
//*	返 回 值: 无
//*********************************************************************************************************
//*/


///*
//*********************************************************************************************************
//*	函 数 名: ReadFileData
//*	功能说明: 读取文件armfly.txt前128个字符，并打印到串口
//*	形    参：无
//*	返 回 值: 无
//*********************************************************************************************************
//*/
//static void ReadFileData(void)
//{
//	/* 本函数使用的局部变量占用较多，请修改启动文件，保证堆栈空间够用 */
//	FRESULT result;
//	FATFS fs;
//	FIL file;
//	DIR DirInf;
//	uint32_t bw;
//	char buf[128];

// 	/* 挂载文件系统 */
//	result = f_mount(FS_USB, &fs);			/* Mount a logical drive */
//	if (result != FR_OK)
//	{
//		printf("挂载文件系统失败(%d)\r\n", result);
//	}

//	/* 打开根文件夹 */
//	result = f_opendir(&DirInf, "/"); /* 如果不带参数，则从当前目录开始 */
//	if (result != FR_OK)
//	{
//		printf("打开根目录失败(%d)\r\n", result);
//		return;
//	}

//	/* 打开文件 */
//	result = f_open(&file, "armfly.txt", FA_OPEN_EXISTING | FA_READ);
//	if (result !=  FR_OK)
//	{
//		printf("Don't Find File : armfly.txt\r\n");
//		return;
//	}

//	/* 读取文件 */
//	result = f_read(&file, &buf, sizeof(buf) - 1, &bw);
//	if (bw > 0)
//	{
//		buf[bw] = 0;
//		printf("\r\narmfly.txt 文件内容 : \r\n%s\r\n", buf);
//	}
//	else
//	{
//		printf("\r\narmfly.txt 文件内容 : \r\n");
//	}

//	/* 关闭文件*/
//	f_close(&file);

//	/* 卸载文件系统 */
//	f_mount(FS_USB, NULL);
//}

///*
//*********************************************************************************************************
//*	函 数 名: CreateDir
//*	功能说明: 在SD卡根目录创建Dir1和Dir2目录，在Dir1目录下创建子目录Dir1_1
//*	形    参：无
//*	返 回 值: 无
//*********************************************************************************************************
//*/
//static void CreateDir(void)
//{
//	/* 本函数使用的局部变量占用较多，请修改启动文件，保证堆栈空间够用 */
//	FRESULT result;
//	FATFS fs;

// 	/* 挂载文件系统 */
//	result = f_mount(FS_USB, &fs);			/* Mount a logical drive */
//	if (result != FR_OK)
//	{
//		printf("挂载文件系统失败 (%d)\r\n", result);
//	}

//	/* 创建目录/Dir1 */
//	result = f_mkdir("/Dir1");
//	if (result == FR_OK)
//	{
//		printf("f_mkdir Dir1 Ok\r\n");
//	}
//	else if (result == FR_EXIST)
//	{
//		printf("Dir1 目录已经存在(%d)\r\n", result);
//	}
//	else
//	{
//		printf("f_mkdir Dir1 失败 (%d)\r\n", result);
//		return;
//	}

//	/* 创建目录/Dir2 */
//	result = f_mkdir("/Dir2");
//	if (result == FR_OK)
//	{
//		printf("f_mkdir Dir2 Ok\r\n");
//	}
//	else if (result == FR_EXIST)
//	{
//		printf("Dir2 目录已经存在(%d)\r\n", result);
//	}
//	else
//	{
//		printf("f_mkdir Dir2 失败 (%d)\r\n", result);
//		return;
//	}

//	/* 创建子目录 /Dir1/Dir1_1	   注意：创建子目录Dir1_1时，必须先创建好Dir1 */
//	result = f_mkdir("/Dir1/Dir1_1"); /* */
//	if (result == FR_OK)
//	{
//		printf("f_mkdir Dir1_1 成功\r\n");
//	}
//	else if (result == FR_EXIST)
//	{
//		printf("Dir1_1 目录已经存在 (%d)\r\n", result);
//	}
//	else
//	{
//		printf("f_mkdir Dir1_1 失败 (%d)\r\n", result);
//		return;
//	}

//	/* 卸载文件系统 */
//	f_mount(FS_USB, NULL);
//}

///*
//*********************************************************************************************************
//*	函 数 名: DeleteDirFile
//*	功能说明: 删除SD卡根目录下的 armfly.txt 文件和 Dir1，Dir2 目录
//*	形    参：无
//*	返 回 值: 无
//*********************************************************************************************************
//*/
//static void DeleteDirFile(void)
//{
//	/* 本函数使用的局部变量占用较多，请修改启动文件，保证堆栈空间够用 */
//	FRESULT result;
//	FATFS fs;
//	char FileName[13];
//	uint8_t i;

// 	/* 挂载文件系统 */
//	result = f_mount(FS_USB, &fs);			/* Mount a logical drive */
//	if (result != FR_OK)
//	{
//		printf("挂载文件系统失败 (%d)\r\n", result);
//	}

//	#if 0
//	/* 打开根文件夹 */
//	result = f_opendir(&DirInf, "/"); /* 如果不带参数，则从当前目录开始 */
//	if (result != FR_OK)
//	{
//		printf("打开根目录失败(%d)\r\n", result);
//		return;
//	}
//	#endif

//	/* 删除目录/Dir1 【因为还存在目录非空（存在子目录)，所以这次删除会失败】*/
//	result = f_unlink("/Dir1");
//	if (result == FR_OK)
//	{
//		printf("删除目录Dir1成功\r\n");
//	}
//	else if (result == FR_NO_FILE)
//	{
//		printf("没有发现文件或目录 :%s\r\n", "/Dir1");
//	}
//	else
//	{
//		printf("删除Dir1失败(错误代码 = %d) 文件只读或目录非空\r\n", result);
//	}

//	/* 先删除目录/Dir1/Dir1_1 */
//	result = f_unlink("/Dir1/Dir1_1");
//	if (result == FR_OK)
//	{
//		printf("删除子目录/Dir1/Dir1_1成功\r\n");
//	}
//	else if ((result == FR_NO_FILE) || (result == FR_NO_PATH))
//	{
//		printf("没有发现文件或目录 :%s\r\n", "/Dir1/Dir1_1");
//	}
//	else
//	{
//		printf("删除子目录/Dir1/Dir1_1失败(错误代码 = %d) 文件只读或目录非空\r\n", result);
//	}

//	/* 先删除目录/Dir1 */
//	result = f_unlink("/Dir1");
//	if (result == FR_OK)
//	{
//		printf("删除目录Dir1成功\r\n");
//	}
//	else if (result == FR_NO_FILE)
//	{
//		printf("没有发现文件或目录 :%s\r\n", "/Dir1");
//	}
//	else
//	{
//		printf("删除Dir1失败(错误代码 = %d) 文件只读或目录非空\r\n", result);
//	}

//	/* 删除目录/Dir2 */
//	result = f_unlink("/Dir2");
//	if (result == FR_OK)
//	{
//		printf("删除目录 Dir2 成功\r\n");
//	}
//	else if (result == FR_NO_FILE)
//	{
//		printf("没有发现文件或目录 :%s\r\n", "/Dir2");
//	}
//	else
//	{
//		printf("删除Dir2 失败(错误代码 = %d) 文件只读或目录非空\r\n", result);
//	}

//	/* 删除文件 armfly.txt */
//	result = f_unlink("armfly.txt");
//	if (result == FR_OK)
//	{
//		printf("删除文件 armfly.txt 成功\r\n");
//	}
//	else if (result == FR_NO_FILE)
//	{
//		printf("没有发现文件或目录 :%s\r\n", "armfly.txt");
//	}
//	else
//	{
//		printf("删除armfly.txt失败(错误代码 = %d) 文件只读或目录非空\r\n", result);
//	}

//	/* 删除文件 speed1.txt */
//	for (i = 0; i < 20; i++)
//	{
//		sprintf(FileName, "Speed%02d.txt", i);		/* 每写1次，序号递增 */
//		result = f_unlink(FileName);
//		if (result == FR_OK)
//		{
//			printf("删除文件%s成功\r\n", FileName);
//		}
//		else if (result == FR_NO_FILE)
//		{
//			printf("没有发现文件:%s\r\n", FileName);
//		}
//		else
//		{
//			printf("删除%s文件失败(错误代码 = %d) 文件只读或目录非空\r\n", FileName, result);
//		}
//	}

//	/* 卸载文件系统 */
//	f_mount(FS_USB, NULL);
//}

///*
//*********************************************************************************************************
//*	函 数 名: WriteFileTest
//*	功能说明: 测试文件读写速度
//*	形    参：无
//*	返 回 值: 无
//*********************************************************************************************************
//*/
//static void WriteFileTest(void)
//{
//	/* 本函数使用的局部变量占用较多，请修改启动文件，保证堆栈空间够用 */
//	FRESULT result;
//	FATFS fs;
//	FIL file;
//	DIR DirInf;
//	uint32_t bw;
//	uint32_t i,k;
//	uint32_t runtime1,runtime2,timelen;
//	uint8_t err = 0;
//	char TestFileName[13];
//	static uint8_t s_ucTestSn = 0;

//	for (i = 0; i < sizeof(g_TestBuf); i++)
//	{
//		g_TestBuf[i] = (i / 512) + '0';
//	}

//  	/* 挂载文件系统 */
//	result = f_mount(FS_USB, &fs);			/* Mount a logical drive */
//	if (result != FR_OK)
//	{
//		printf("挂载文件系统失败 (%d)\r\n", result);
//	}

//	/* 打开根文件夹 */
//	result = f_opendir(&DirInf, "/"); /* 如果不带参数，则从当前目录开始 */
//	if (result != FR_OK)
//	{
//		printf("打开根目录失败 (%d)\r\n", result);
//		return;
//	}

//	/* 打开文件 */
//	sprintf(TestFileName, "Speed%02d.txt", s_ucTestSn++);		/* 每写1次，序号递增 */
//	result = f_open(&file, TestFileName, FA_CREATE_ALWAYS | FA_WRITE);

//	/* 写一串数据 */
//	printf("开始写文件%s %dKB ...\r\n", TestFileName, TEST_FILE_LEN / 1024);
//	runtime1 = bsp_GetRunTime();	/* 读取系统运行时间 */
//	for (i = 0; i < TEST_FILE_LEN / BUF_SIZE; i++)
//	{
//		result = f_write(&file, g_TestBuf, sizeof(g_TestBuf), &bw);
//		if (result == FR_OK)
//		{
//			if (((i + 1) % 8) == 0)
//			{
//				printf(".");
//			}
//		}
//		else
//		{
//			err = 1;
//			printf("%s文件写失败\r\n", TestFileName);
//			break;
//		}
//	}
//	runtime2 = bsp_GetRunTime();	/* 读取系统运行时间 */

//	if (err == 0)
//	{
//		timelen = (runtime2 - runtime1);
//		printf("\r\n  写耗时 : %dms   平均写速度 : %dB/S (%dKB/S)\r\n",
//			timelen,
//			(TEST_FILE_LEN * 1000) / timelen,
//			((TEST_FILE_LEN / 1024) * 1000) / timelen);
//	}

//	f_close(&file);		/* 关闭文件*/


//	/* 开始读文件测试 */
//	result = f_open(&file, TestFileName, FA_OPEN_EXISTING | FA_READ);
//	if (result !=  FR_OK)
//	{
//		printf("没有找到文件: %s\r\n", TestFileName);
//		return;
//	}

//	printf("开始读文件 %dKB ...\r\n", TEST_FILE_LEN / 1024);
//	runtime1 = bsp_GetRunTime();	/* 读取系统运行时间 */
//	for (i = 0; i < TEST_FILE_LEN / BUF_SIZE; i++)
//	{
//		result = f_read(&file, g_TestBuf, sizeof(g_TestBuf), &bw);
//		if (result == FR_OK)
//		{
//			if (((i + 1) % 8) == 0)
//			{
//				printf(".");
//			}

//			/* 比较写入的数据是否正确，此语句会导致读卡速度结果降低到 3.5MBytes/S */
//			for (k = 0; k < sizeof(g_TestBuf); k++)
//			{
//				if (g_TestBuf[k] != (k / 512) + '0')
//				{
//				  	err = 1;
//					printf("Speed1.txt 文件读成功，但是数据出错\r\n");
//					break;
//				}
//			}
//			if (err == 1)
//			{
//				break;
//			}
//		}
//		else
//		{
//			err = 1;
//			printf("Speed1.txt 文件读失败\r\n");
//			break;
//		}
//	}
//	runtime2 = bsp_GetRunTime();	/* 读取系统运行时间 */

//	if (err == 0)
//	{
//		timelen = (runtime2 - runtime1);
//		printf("\r\n  读耗时 : %dms   平均读速度 : %dB/S (%dKB/S)\r\n", timelen,
//			(TEST_FILE_LEN * 1000) / timelen, ((TEST_FILE_LEN / 1024) * 1000) / timelen);
//	}

//	/* 关闭文件*/
//	f_close(&file);

//	/* 卸载文件系统 */
//	f_mount(FS_USB, NULL);
//}

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/


