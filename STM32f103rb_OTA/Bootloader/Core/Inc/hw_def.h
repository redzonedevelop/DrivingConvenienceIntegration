/*
 * hw_def.h
 *
 *  Created on: Jun 20, 2025
 *      Author: zvxc3
 */

#ifndef INC_HW_DEF_H_
#define INC_HW_DEF_H_


#include "main.h"
#include "def.h"


#define FLASH_SIZE_META             0x400
#define FLASH_SIZE_FIRM             (47*1024)

#define FLASH_ADDR_BOOT             0x08000000
#define FLASH_ADDR_FIRM							0x8008000 /*Partition A address */
#define FLASH_ADDR_UPDATE						0x8014000


#define BOOT_FLAG_BOOT 		0x01
#define BOOT_FLAG_FW	 		0x00 /* Partition A activate */
#define BOOT_FLAG_UPDATE 	0x02 /* Partition B activate */




#endif /* INC_HW_DEF_H_ */
