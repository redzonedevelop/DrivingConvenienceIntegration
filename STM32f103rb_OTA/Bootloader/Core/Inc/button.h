/*
 * button.h
 *
 *  Created on: May 10, 2025
 *      Author: HONG
 */

#ifndef SRC_COMMON_HW_INCLUDE_BUTTON_H_
#define SRC_COMMON_HW_INCLUDE_BUTTON_H_




#include "hw_def.h"



typedef struct
{
  GPIO_TypeDef *port;
  uint16_t      pin;
  GPIO_PinState on_state;
  GPIO_PinState off_state;
  GPIO_PinState current_state;
} button_tbl_t;

bool button_init(void);
GPIO_PinState button_get_state(uint8_t ch);
bool buttonGetPressed(uint8_t ch);



#endif /* SRC_COMMON_HW_INCLUDE_BUTTON_H_ */
