/*
 * reset.c
 *
 *  Created on: Jun 18, 2025
 *      Author: USER
 */


#include "reset.h"


static bool is_init = false;
static uint32_t boot_mode = 0;
#define BOOT_FLAG_ADDR BKP->DR1 /*Boot flag address*/


bool resetInit(void)
{
  // 1) 백업 도메인 접근 허용
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_BKP_CLK_ENABLE();

  bool ret;
  //boot_mode = BOOT_FLAG_UPDATE;
  boot_mode = BOOT_FLAG_ADDR;
  BOOT_FLAG_ADDR = BOOT_FLAG_BOOT;

  is_init = true;

  ret = is_init;
  return ret;
}

void resetToBoot(void)
{
  resetSetBootMode(1<<MODE_BIT_BOOT);
  resetToReset();
}

void resetToReset(void)
{
  HAL_NVIC_SystemReset();
}

void resetSetBootMode(uint32_t mode)
{
  boot_mode = mode;
  BOOT_FLAG_ADDR = boot_mode;
}

uint32_t resetGetBootMode(void)
{
  return boot_mode;
}
