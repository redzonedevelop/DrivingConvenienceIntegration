/*
 * boot.c
 *
 *  Created on: Jun 18, 2025
 *      Author: USER
 */


#include "hw.h"


uint16_t bootInit(void)
{
	uint16_t err_code = CMD_OK;
	fw_meta_t *meta  = (fw_meta_t *)(FLASH_ADDR_FIRM);
	fw_meta_t fw_meta;
	if(meta->ecu == 0xFF)
	{
	  	fw_meta.ecu 		= 14;
	  	fw_meta.version     = 0;
	  	fw_meta.size		= 47 * 1024;

	    if (flashWrite(FLASH_ADDR_FIRM, (uint8_t *)&fw_meta, sizeof(fw_meta_t)) != true)
	    {
	      err_code = ERR_BOOT_FLASH_WRITE;
	    }
	    resetToBoot();
	}
	return err_code;

}

uint16_t bootVerifyUpdate(void)
{
  fw_meta_t update_meta;
  fw_meta_t *fw_meta  = (fw_meta_t *)(FLASH_ADDR_FIRM);

  if (flashRead(FLASH_ADDR_UPDATE, (uint8_t*)&update_meta, sizeof(update_meta)) != true)
	  Error_Handler();
  // 2) 태그 무결성 검사
  if (update_meta.version > fw_meta->version)
  {
	  Error_Handler();
  }
  if (update_meta.size    >= FLASH_SIZE_FIRM)
  {
	  Error_Handler();
  }
  uint32_t crc = 0xFFFFFFFF;
  uint32_t addr = FLASH_ADDR_UPDATE + FLASH_SIZE_META;
  uint32_t remaining = update_meta.size - sizeof(uint32_t);
  uint8_t  buf[128];


  while (remaining)
  {
      uint32_t chunk = remaining > sizeof(buf) ? sizeof(buf) : remaining;
      if (!flashRead(addr, buf, chunk))
          Error_Handler();

      for (uint32_t i = 0; i < chunk; i++)
          crc = (crc >> 8) ^ crc_table[(crc ^ buf[i]) & 0xFF];

      addr      += chunk;
      remaining -= chunk;
  }
  crc ^= 0xFFFFFFFF;

  // 4) 비교
  return (update_meta.crc == crc) ? CMD_OK : ERR_BOOT_FW_CRC;
}

uint16_t bootVerifyFirm(void)
{
  fw_meta_t fw_meta;

  if (flashRead(FLASH_ADDR_UPDATE, (uint8_t*)&fw_meta, sizeof(fw_meta)) != true)
	  Error_Handler();

  uint32_t crc = 0xFFFFFFFF;
  uint32_t addr = FLASH_ADDR_FIRM + FLASH_SIZE_META;
  uint32_t remaining = fw_meta.size - sizeof(uint32_t);
  uint8_t  buf[128];

  while (remaining)
  {
      uint32_t chunk = remaining > sizeof(buf) ? sizeof(buf) : remaining;
      if (!flashRead(addr, buf, chunk))
          return ERR_BOOT_FLASH_READ;

      for (uint32_t i = 0; i < chunk; i++)
          crc = (crc >> 8) ^ crc_table[(crc ^ buf[i]) & 0xFF];

      addr      += chunk;
      remaining -= chunk;
  }
  crc ^= 0xFFFFFFFF;

  // 4) 비교
  return (fw_meta.crc == crc) ? CMD_OK : ERR_BOOT_FW_CRC;
}


uint16_t bootUpdateFirm(void)
{
  uint8_t err_code = CMD_OK;
  fw_meta_t tag;

  fw_meta_t *p_tag = (fw_meta_t *)&tag;
  // Read Tag
  //
  if(flashRead(FLASH_ADDR_UPDATE, (uint8_t *)p_tag, sizeof(fw_meta_t)) != true)
  {
	  Error_Handler();
  }


  // Erase F/W
  //
  if (flashErase(FLASH_ADDR_FIRM, FLASH_SIZE_META + p_tag->size) != true)
  {
	  Error_Handler();
  }
  //ledOff(HW_LED_CH_DOWN);

    // Write F/W
    //
    uint32_t index;
    uint32_t fw_size;

    index = 0;
    fw_size = FLASH_SIZE_META + p_tag->size;

    while(index < fw_size)
    {
      uint8_t buf[512];
      uint32_t wr_size;
      uint32_t wr_addr;


      wr_addr = FLASH_ADDR_UPDATE + index;
      wr_size = constrain(fw_size-index, 0, 512);

      if (flashRead(wr_addr, buf, wr_size) != true)
      {
        err_code = ERR_BOOT_FLASH_READ;
        break;
      }

      wr_addr = FLASH_ADDR_FIRM + index;

      if (flashWrite(wr_addr, buf, wr_size) != true)
      {
        err_code = ERR_BOOT_FLASH_WRITE;
        break;
      }

      index += wr_size;

    }
    //ledOff(HW_LED_CH_UPDATE);

    if (err_code == CMD_OK)
    {
      // Verify F/W
      //
      err_code = bootVerifyFirm();
    }


  return err_code;
}

uint16_t bootJumpFirm(void)
{
  uint16_t err_code = CMD_OK;

  //err_code = bootVerifyFirm();
  if (err_code == CMD_OK)
  {
    void (**jump_func)(void) = (void (**)(void))(FLASH_ADDR_FIRM + FLASH_SIZE_META + 4);
	  //void (**jump_func)(void) = (void (**)(void))(FLASH_ADDR_FIRM + 4);

    if (((uint32_t)*jump_func) >= FLASH_ADDR_FIRM && ((uint32_t)*jump_func) < (FLASH_ADDR_FIRM + FLASH_SIZE_FIRM))
    {
      /*logPrintf("[  ] bootJumpFirm()\n");
      logPrintf("     addr : 0x%lX\n", (uint32_t)*jump_func);
*/
      resetSetBootMode(0);

      // 점프하기전 인터럽트 Disable
      //
      HAL_RCC_DeInit();

      // Disable Interrupts
      //
      for (int i=0; i<8; i++)
      {
        NVIC->ICER[i] = 0xFFFFFFFF;
        __DSB();
        __ISB();
      }
      SysTick->CTRL = 0;

      (*jump_func)();
    }
    else
    {
      err_code = ERR_BOOT_INVALID_FW;
    }
  }

  return err_code;
}
