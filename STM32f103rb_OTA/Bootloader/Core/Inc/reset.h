/*
 * reset.h
 *
 *  Created on: Jun 18, 2025
 *      Author: USER
 */

#ifndef INC_RESET_H_
#define INC_RESET_H_

#include "hw_def.h"

enum
{
  MODE_BIT_BOOT = 0,
  MODE_BIT_UPDATE,
};


bool resetInit(void);
void resetToBoot(void);
void resetToReset(void);
void resetSetBootMode(uint32_t mode);
uint32_t resetGetBootMode(void);



#endif /* INC_RESET_H_ */
