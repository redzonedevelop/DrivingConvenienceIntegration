/*
 * flash.h
 *
 *  Created on: Jun 20, 2025
 *      Author: zvxc3
 */

#ifndef INC_FLASH_H_
#define INC_FLASH_H_

#include "main.h"
#include <stdbool.h>


bool flashInit(void);
bool flashErase(uint32_t addr, uint32_t length);
bool flashRead(uint32_t addr, uint8_t *p_data, uint32_t length);
bool flashWrite(uint32_t addr, uint8_t *p_data, uint32_t length);



#endif /* INC_FLASH_H_ */
