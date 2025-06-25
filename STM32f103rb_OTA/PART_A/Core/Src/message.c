/*
 * message.c
 *
 *  Created on: Jun 21, 2025
 *      Author: USER
 */


#include "message.h"


CAN_Message_t messageBuffer[MESSAGE_BUFFER_SIZE];
volatile uint16_t messageBufferHead = 0;
volatile uint16_t messageBufferTail = 0;

uint8_t MessageBufferIsFull()
{
    return (((messageBufferHead + 1) % MESSAGE_BUFFER_SIZE) == messageBufferTail);
}


void MessageBufferFlush(void)
{
	messageBufferTail = 0;
	messageBufferHead = 0;
}


uint8_t MessageBufferIsEmpty(void)
{
	return messageBufferHead == messageBufferTail;
}

void MessageBufferPut(CAN_RxHeaderTypeDef *rxHeader, uint8_t *rxData)
{
    if (!MessageBufferIsFull())
    {
        messageBuffer[messageBufferHead].len = rxHeader->DLC;
        memcpy(messageBuffer[messageBufferHead].rxData, rxData, 8);
        messageBufferHead = (messageBufferHead + 1) % MESSAGE_BUFFER_SIZE;
    }
}

void MessageBufferGet(CAN_RxHeaderTypeDef *rxHeader, uint8_t *rxData)
{
    if (!MessageBufferIsEmpty())
    {
        rxHeader->DLC = messageBuffer[messageBufferTail].len;
        memcpy(rxData, messageBuffer[messageBufferTail].rxData, 8);
        messageBufferTail = (messageBufferTail + 1) % MESSAGE_BUFFER_SIZE;
    }
}
