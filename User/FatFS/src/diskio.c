/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2013        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control module to the FatFs module with a defined API.        */
/*-----------------------------------------------------------------------*/

#include "diskio.h"			/* FatFs lower layer API */
#include "bsp.h"			/* 底层驱动 */
#include "usbh_bsp_msc.h"			/* 底层驱动 */

#define SECTOR_SIZE		512

/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber (0..) */
)
{
	DSTATUS stat = STA_NOINIT;

	switch (pdrv)
	{
		case FS_USB :		/* STM32 USB Host 口外接U盘 */
			if(HCD_IsDeviceConnected(&USB_OTG_Core))
			{
				stat &= ~STA_NOINIT;
			}
			break;

		default :
			break;
	}
	return stat;
}



/*-----------------------------------------------------------------------*/
/* Get Disk Status                                                       */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber (0..) */
)
{
	DSTATUS stat = STA_NOINIT;

	switch (pdrv)
	{
		case FS_USB :
			stat = 0;
			break;

		default:
			break;
	}
	return stat;
}

/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Sector address (LBA) */
	BYTE count		/* Number of sectors to read (1..128) */
)
{
	DRESULT res;

	switch (pdrv)
	{
		case FS_USB :
			//res = USB_disk_read(buff, sector, count);
			{
				BYTE status = USBH_MSC_OK;

				//if (Stat & STA_NOINIT) 	return RES_NOTRDY;

				if (HCD_IsDeviceConnected(&USB_OTG_Core))
				{
					do
					{
						status = USBH_MSC_Read10(&USB_OTG_Core, buff,sector,512 * count);
						USBH_MSC_HandleBOTXfer(&USB_OTG_Core ,&USB_Host);

						if (!HCD_IsDeviceConnected(&USB_OTG_Core))
						{
							break;
						}
					}
					while (status == USBH_MSC_BUSY );
				}

				if (status == USBH_MSC_OK)
				{
					res = RES_OK;
				}
				else
				{
					res = RES_ERROR;
				}
			}
			break;

		default:
			res = RES_PARERR;
			break;
	}
	return res;
}

/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/
#if _USE_WRITE
DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber (0..) */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Sector address (LBA) */
	BYTE count			/* Number of sectors to write (1..128) */
)
{
	DRESULT res;

	switch (pdrv)
	{
		case FS_USB :
			//res = USB_disk_write(buff, sector, count);
			{
				BYTE status = USBH_MSC_OK;

				//if (drv || !count) return RES_PARERR;

				//if (Stat & STA_NOINIT) return RES_NOTRDY;
				//if (Stat & STA_PROTECT) return RES_WRPRT;

				if (HCD_IsDeviceConnected(&USB_OTG_Core))
				{
					do
					{
						status = USBH_MSC_Write10(&USB_OTG_Core,(BYTE*)buff,sector, 512 * count);
						USBH_MSC_HandleBOTXfer(&USB_OTG_Core, &USB_Host);

						if(!HCD_IsDeviceConnected(&USB_OTG_Core))
						{
							break;
						}
					}
					while(status == USBH_MSC_BUSY );

				}

				if (status == USBH_MSC_OK)
				{
					res = RES_OK;
				}
				else
				{
					res = RES_ERROR;
				}
			}
			break;

		default:
			res = RES_PARERR;
			break;
	}
	return res;
}
#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

#if _USE_IOCTL
DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	DRESULT res;

	switch (pdrv) {
	case FS_USB :
		{
			//if (drv) return RES_PARERR;
			res = RES_ERROR;

			//if (Stat & STA_NOINIT) return RES_NOTRDY;
			switch (cmd)
			{
				case CTRL_SYNC :		/* Make sure that no pending write process */
					res = RES_OK;
					break;

				case GET_SECTOR_COUNT :	/* Get number of sectors on the disk (DWORD) */
					*(DWORD*)buff = (DWORD) USBH_MSC_Param.MSCapacity;
					res = RES_OK;
					break;

				case GET_SECTOR_SIZE :	/* Get R/W sector size (WORD) */
					*(WORD*)buff = 512;
					res = RES_OK;
					break;

				case GET_BLOCK_SIZE :	/* Get erase block size in unit of sector (DWORD) */\
					*(DWORD*)buff = 512;
					res = RES_OK;
					break;

				default:
					res = RES_PARERR;
					break;
			}
			return res;
		}

	}
	return RES_PARERR;
}
#endif

/*
*********************************************************************************************************
*	函 数 名: get_fattime
*	功能说明: 获得系统时间，用于改写文件的创建和修改时间。
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
DWORD get_fattime (void)
{
	/* 如果有全局时钟，可按下面的格式进行时钟转换. 这个例子是2013-01-01 00:00:00 */

	return	  ((DWORD)(2013 - 1980) << 25)	/* Year = 2013 */
			| ((DWORD)1 << 21)				/* Month = 1 */
			| ((DWORD)1 << 16)				/* Day_m = 1*/
			| ((DWORD)0 << 11)				/* Hour = 0 */
			| ((DWORD)0 << 5)				/* Min = 0 */
			| ((DWORD)0 >> 1);				/* Sec = 0 */
}
