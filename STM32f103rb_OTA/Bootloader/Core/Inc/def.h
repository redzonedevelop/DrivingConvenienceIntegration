/*
 * def.h
 *
 *  Created on: Jun 20, 2025
 *      Author: zvxc3
 */

#ifndef INC_DEF_H_
#define INC_DEF_H_



#include <stdint.h>
#include <stdbool.h>
#include <string.h>


#include "err_code.h"


#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))


typedef struct
{
	uint8_t  ecu;
	uint8_t  version;
	uint16_t size;
	uint32_t  crc;
} fw_meta_t;



#endif /* INC_DEF_H_ */
