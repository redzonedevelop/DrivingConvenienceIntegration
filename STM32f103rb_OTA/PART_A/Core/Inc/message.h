/*
 * message.h
 *
 *  Created on: Jun 21, 2025
 *      Author: USER
 */

#ifndef INC_MESSAGE_H_
#define INC_MESSAGE_H_


#include "main.h"

#define MESSAGE_BUFFER_SIZE 513




typedef struct {
  uint8_t len;
  uint8_t rxData[8];
} CAN_Message_t;






/* 메시지 버퍼 관리 함수 */
uint8_t MessageBufferIsFull();
uint8_t MessageBufferIsEmpty(void);
void MessageBufferFlush(void);
void MessageBufferPut(CAN_RxHeaderTypeDef *rxHeader, uint8_t *rxData);
void MessageBufferGet(CAN_RxHeaderTypeDef *rxHeader, uint8_t *rxData);

#endif /* INC_MESSAGE_H_ */
