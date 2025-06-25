/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef void (*pFunction)(void);

typedef enum {
	FW_UPDATE_IDLE,
	FW_UPDATE_REQUESTED,
	FW_UPDATE_IN_PROGRESS,
	FW_UPDATE_COMPLETE
} FirmwareUpdateStat_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define BOOT_FLAG_ADDR BKP->DR1 /* Boot flag address */

#define NUMBER_OF_PAGES_IN_PARTITION 47 /* Number of partition pages */

#define CAN_ID_FILE  0x68c /* receive file data */
#define CAN_ID_META  0x681 /* receive Meta     */
#define CAN_ID_REQ   0x680 /* Request CAN message */
#define CAN_ID_ACK   0x5cb /* ACK */
#define CAN_ID_NACK  0x5cc /* NACK */
#define CAN_ID_LIGHT 0x5c9 /* NACK */

#define CAN_SID_UPDATE    0x680 /* Response CAN message */
#define CAN_SID_VERIFY  	0x15	 /* FW_Verify CAN message */
#define CAN_SID_ROLLBACK	0x16 /* Rollback CAN message */

#define FLASH_SIZE_META             0x400
#define FLASH_SIZE_VEC              0x400
#define FLASH_SIZE_FIRM             (47*1024)

#define FLASH_ADDR_BOOT             0x8000000
#define FLASH_ADDR_FIRM							0x8008000 /*Partition A address */
#define FLASH_ADDR_UPDATE						0x8014000

#define BOOT_FLAG_BOOT 		0x01
#define BOOT_FLAG_FW	 		0x00 /* Partition A activate */
#define BOOT_FLAG_UPDATE 	0x02 /* Partition B activate */

#define BLOCK_SIZE_BYTES 4096    // 4 KB 단위

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef TxHeader;
uint32_t TxMailbox;

CAN_TxHeaderTypeDef txHeader;
uint32_t txMailbox;

FirmwareUpdateStat_t fwUpdateState = FW_UPDATE_IDLE;
volatile uint8_t fwUpdateRequested = 0;
volatile uint8_t fwUpdateComplete = 0;
uint32_t fwUpdateReceivedBytes = 0;

typedef struct {
	uint8_t   ecu;
	uint8_t   version;
	uint16_t  size;
    uint32_t  crc;
} fw_meta_t;

uint8_t TxData[8] = { 0x00, };
uint8_t fw_version;
static uint32_t blockReceivedBytes = 0;  // 현재 블록에 기록된 바이트 수

uint8_t lightMode[8];
uint8_t updateMode = 0;
fw_meta_t fw_meta;
fw_meta_t update_meta;

uint32_t currentAddress = FLASH_ADDR_UPDATE + FLASH_SIZE_META;



static const unsigned int crc_table[256] = {
	0x00000000L, 0x77073096L, 0xee0e612cL, 0x990951baL, 0x076dc419L,
	0x706af48fL, 0xe963a535L, 0x9e6495a3L, 0x0edb8832L, 0x79dcb8a4L,
	0xe0d5e91eL, 0x97d2d988L, 0x09b64c2bL, 0x7eb17cbdL, 0xe7b82d07L,
	0x90bf1d91L, 0x1db71064L, 0x6ab020f2L, 0xf3b97148L, 0x84be41deL,
	0x1adad47dL, 0x6ddde4ebL, 0xf4d4b551L, 0x83d385c7L, 0x136c9856L,
	0x646ba8c0L, 0xfd62f97aL, 0x8a65c9ecL, 0x14015c4fL, 0x63066cd9L,
	0xfa0f3d63L, 0x8d080df5L, 0x3b6e20c8L, 0x4c69105eL, 0xd56041e4L,
	0xa2677172L, 0x3c03e4d1L, 0x4b04d447L, 0xd20d85fdL, 0xa50ab56bL,
	0x35b5a8faL, 0x42b2986cL, 0xdbbbc9d6L, 0xacbcf940L, 0x32d86ce3L,
	0x45df5c75L, 0xdcd60dcfL, 0xabd13d59L, 0x26d930acL, 0x51de003aL,
	0xc8d75180L, 0xbfd06116L, 0x21b4f4b5L, 0x56b3c423L, 0xcfba9599L,
	0xb8bda50fL, 0x2802b89eL, 0x5f058808L, 0xc60cd9b2L, 0xb10be924L,
	0x2f6f7c87L, 0x58684c11L, 0xc1611dabL, 0xb6662d3dL, 0x76dc4190L,
	0x01db7106L, 0x98d220bcL, 0xefd5102aL, 0x71b18589L, 0x06b6b51fL,
	0x9fbfe4a5L, 0xe8b8d433L, 0x7807c9a2L, 0x0f00f934L, 0x9609a88eL,
	0xe10e9818L, 0x7f6a0dbbL, 0x086d3d2dL, 0x91646c97L, 0xe6635c01L,
	0x6b6b51f4L, 0x1c6c6162L, 0x856530d8L, 0xf262004eL, 0x6c0695edL,
	0x1b01a57bL, 0x8208f4c1L, 0xf50fc457L, 0x65b0d9c6L, 0x12b7e950L,
	0x8bbeb8eaL, 0xfcb9887cL, 0x62dd1ddfL, 0x15da2d49L, 0x8cd37cf3L,
	0xfbd44c65L, 0x4db26158L, 0x3ab551ceL, 0xa3bc0074L, 0xd4bb30e2L,
	0x4adfa541L, 0x3dd895d7L, 0xa4d1c46dL, 0xd3d6f4fbL, 0x4369e96aL,
	0x346ed9fcL, 0xad678846L, 0xda60b8d0L, 0x44042d73L, 0x33031de5L,
	0xaa0a4c5fL, 0xdd0d7cc9L, 0x5005713cL, 0x270241aaL, 0xbe0b1010L,
	0xc90c2086L, 0x5768b525L, 0x206f85b3L, 0xb966d409L, 0xce61e49fL,
	0x5edef90eL, 0x29d9c998L, 0xb0d09822L, 0xc7d7a8b4L, 0x59b33d17L,
	0x2eb40d81L, 0xb7bd5c3bL, 0xc0ba6cadL, 0xedb88320L, 0x9abfb3b6L,
	0x03b6e20cL, 0x74b1d29aL, 0xead54739L, 0x9dd277afL, 0x04db2615L,
	0x73dc1683L, 0xe3630b12L, 0x94643b84L, 0x0d6d6a3eL, 0x7a6a5aa8L,
	0xe40ecf0bL, 0x9309ff9dL, 0x0a00ae27L, 0x7d079eb1L, 0xf00f9344L,
	0x8708a3d2L, 0x1e01f268L, 0x6906c2feL, 0xf762575dL, 0x806567cbL,
	0x196c3671L, 0x6e6b06e7L, 0xfed41b76L, 0x89d32be0L, 0x10da7a5aL,
	0x67dd4accL, 0xf9b9df6fL, 0x8ebeeff9L, 0x17b7be43L, 0x60b08ed5L,
	0xd6d6a3e8L, 0xa1d1937eL, 0x38d8c2c4L, 0x4fdff252L, 0xd1bb67f1L,
	0xa6bc5767L, 0x3fb506ddL, 0x48b2364bL, 0xd80d2bdaL, 0xaf0a1b4cL,
	0x36034af6L, 0x41047a60L, 0xdf60efc3L, 0xa867df55L, 0x316e8eefL,
	0x4669be79L, 0xcb61b38cL, 0xbc66831aL, 0x256fd2a0L, 0x5268e236L,
	0xcc0c7795L, 0xbb0b4703L, 0x220216b9L, 0x5505262fL, 0xc5ba3bbeL,
	0xb2bd0b28L, 0x2bb45a92L, 0x5cb36a04L, 0xc2d7ffa7L, 0xb5d0cf31L,
	0x2cd99e8bL, 0x5bdeae1dL, 0x9b64c2b0L, 0xec63f226L, 0x756aa39cL,
	0x026d930aL, 0x9c0906a9L, 0xeb0e363fL, 0x72076785L, 0x05005713L,
	0x95bf4a82L, 0xe2b87a14L, 0x7bb12baeL, 0x0cb61b38L, 0x92d28e9bL,
	0xe5d5be0dL, 0x7cdcefb7L, 0x0bdbdf21L, 0x86d3d2d4L, 0xf1d4e242L,
	0x68ddb3f8L, 0x1fda836eL, 0x81be16cdL, 0xf6b9265bL, 0x6fb077e1L,
	0x18b74777L, 0x88085ae6L, 0xff0f6a70L, 0x66063bcaL, 0x11010b5cL,
	0x8f659effL, 0xf862ae69L, 0x616bffd3L, 0x166ccf45L, 0xa00ae278L,
	0xd70dd2eeL, 0x4e048354L, 0x3903b3c2L, 0xa7672661L, 0xd06016f7L,
	0x4969474dL, 0x3e6e77dbL, 0xaed16a4aL, 0xd9d65adcL, 0x40df0b66L,
	0x37d83bf0L, 0xa9bcae53L, 0xdebb9ec5L, 0x47b2cf7fL, 0x30b5ffe9L,
	0xbdbdf21cL, 0xcabac28aL, 0x53b39330L, 0x24b4a3a6L, 0xbad03605L,
	0xcdd70693L, 0x54de5729L, 0x23d967bfL, 0xb3667a2eL, 0xc4614ab8L,
	0x5d681b02L, 0x2a6f2b94L, 0xb40bbe37L, 0xc30c8ea1L, 0x5a05df1bL,
	0x2d02ef8dL
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void NVIC_Config(void);
/* 펌웨어 업데이트 관련 함수 */
void FirmwareUpdateStateMachine(void);
void StartFirmwareUpdate(void);
void EraseFlashMemory(void);
void SendInactivePartitionAddress(void);

/* 추가: 펌웨어 시작 알림 함수 */
void SendFirmwareStartedMessage(void);

/* 함수 프로토타입 추가 */
void ProcessFirmwareSizeMessage(CAN_RxHeaderTypeDef *rxHeader, uint8_t *rxData);
void LD2Flip(void);
void Error_Handler(void);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	NVIC_Config();

	HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_BKP_CLK_ENABLE();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

	if (HAL_CAN_Start(&hcan) != HAL_OK) {
		Error_Handler();
	}
	  CAN_FilterTypeDef canFilterConfig;
	  canFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
	  canFilterConfig.FilterBank = 0;
	  canFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	  canFilterConfig.FilterIdHigh = 0x0000;
	  canFilterConfig.FilterIdLow = 0x0000;
	  canFilterConfig.FilterMaskIdHigh = 0x0000;
	  canFilterConfig.FilterMaskIdLow = 0x0000;
	  canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	  canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
/*
	// Common parts
	CAN_FilterTypeDef filter;
	filter.FilterMode            = CAN_FILTERMODE_IDLIST;     // 리스트 모드
	filter.FilterScale           = CAN_FILTERSCALE_16BIT;     // 16비트 스케일
	filter.FilterFIFOAssignment  = CAN_FILTER_FIFO0;          // FIFO0 할당
	filter.FilterActivation      = CAN_FILTER_ENABLE;         // 활성화

	// Bank0: CAN_ID_FILE, CAN_ID_META
	filter.FilterBank    = 0;
	filter.FilterIdHigh  = (CAN_ID_FILE << 5) & 0xFFE0;  // StdId << 5 (정렬)
	filter.FilterIdLow   = (CAN_ID_META << 5) & 0xFFE0;
	HAL_CAN_ConfigFilter(&hcan, &filter);

	// Bank1: CAN_ID_REQ, CAN_ID_ACK
	filter.FilterBank    = 1;
	filter.FilterIdHigh  = (CAN_ID_REQ << 5)  & 0xFFE0;
	filter.FilterIdLow   = (CAN_ID_ACK << 5)  & 0xFFE0;
	HAL_CAN_ConfigFilter(&hcan, &filter);

	// Bank2: CAN_ID_NACK, CAN_ID_LIGHT
	filter.FilterBank    = 2;
	filter.FilterIdHigh  = (CAN_ID_NACK << 5) & 0xFFE0;
	filter.FilterIdLow   = (CAN_ID_LIGHT << 5)& 0xFFE0;
	HAL_CAN_ConfigFilter(&hcan, &filter);*/

	__HAL_CAN_CLEAR_FLAG(&hcan, CAN_FLAG_FOV1);
	if (HAL_CAN_ConfigFilter(&hcan, &canFilterConfig) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING)
			!= HAL_OK) {
		Error_Handler();
	}

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	flashInit();

    light_init();
    LCORNER_ON();
    set_lowbeam(120);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(change_flag == 1 && !fwUpdateRequested){
		  change_light(lightMode);
		  HAL_Delay(100);
		  change_flag = 0;
	  }
	  else if (fwUpdateRequested)
	  {
		  FirmwareUpdateStateMachine();
	  }
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void NVIC_Config(void) {
	HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
}

void canMsgWrite(uint32_t id, uint8_t dlc, uint8_t *txData) {

	txHeader.StdId = id;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.IDE = CAN_ID_STD;
	txHeader.DLC = dlc;

	if (HAL_CAN_AddTxMessage(&hcan, &txHeader, txData, &txMailbox) != HAL_OK) {
		Error_Handler();
	}

	while (HAL_CAN_IsTxMessagePending(&hcan, txMailbox)) {

	}
}
void ProcessFirmwareMeta(CAN_RxHeaderTypeDef *rxHeader, uint8_t *rxData) {
	update_meta.ecu = rxData[1];
	update_meta.version = rxData[2];
	update_meta.size = (rxData[4] << 8) | rxData[3];
	fwUpdateState = FW_UPDATE_IN_PROGRESS;

	fw_meta_t *p_meta = (fw_meta_t*) &fw_meta;
	flashRead(FLASH_ADDR_FIRM, (uint8_t*) p_meta, sizeof(fw_meta_t));

	if (update_meta.ecu != p_meta->ecu)						// 내 ecu가 맞는지
	{
		Error_Handler();
	}
	if (update_meta.version <= p_meta->version)		// download할 version이 맞는지
	{
		Error_Handler();
	}

	if (update_meta.size > FLASH_SIZE_FIRM) 	// firmware size over 오류
	{
		Error_Handler();
	}

	uint8_t data = 0x53;
	canMsgWrite(CAN_ID_ACK, 1, &data);

}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	CAN_RxHeaderTypeDef rxHeader;
	uint8_t rxData[8];

	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK) {
		Error_Handler();
		return;
	}
	if (rxHeader.StdId == CAN_ID_LIGHT && fwUpdateRequested == 0)
	{
	    change_flag = 1;
	    memcpy(lightMode, rxData, sizeof(uint8_t) * 8);
	}
	else if (rxHeader.StdId == CAN_SID_UPDATE && fwUpdateRequested == 0)
	{
		fwUpdateRequested = 1;
	}
	else if (fwUpdateRequested == 1 && rxHeader.StdId == CAN_ID_META)	// meta 수신한다.
	{
		ProcessFirmwareMeta(&rxHeader, rxData);
	}
	else if (fwUpdateRequested == 1 && rxHeader.StdId == CAN_ID_FILE) {
		MessageBufferPut(&rxHeader, rxData);
	}
}

void EraseFlashMemory(void) {
	HAL_FLASH_Unlock();

	FLASH_EraseInitTypeDef eraseInitStruct;
	uint32_t pageError = 0;

	eraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	eraseInitStruct.PageAddress = FLASH_ADDR_UPDATE + FLASH_SIZE_META; // 업데이트 fw 시작 주소
	eraseInitStruct.NbPages = NUMBER_OF_PAGES_IN_PARTITION;

	if (HAL_FLASHEx_Erase(&eraseInitStruct, &pageError) != HAL_OK) {
		HAL_FLASH_Lock();
		fwUpdateState = FW_UPDATE_IDLE;
		canMsgWrite(CAN_ID_NACK, 1, TxData);	// update mode no 응답
		NVIC_SystemReset(); 				// 이거는 왜 있는건지 모르겠네
	}

	HAL_FLASH_Lock();
}

void StartFirmwareUpdate(void) {
	// 여기서 이제 원래는 받은 tag 정보를 가지고 인증 후 erase 및
	if (updateMode == 0) {
		fwUpdateReceivedBytes = 0;
		blockReceivedBytes = 0;    // 블록 카운터 초기화

		EraseFlashMemory();
		uint8_t data = 0x52;
		canMsgWrite(CAN_ID_ACK, 1, &data);	// 준비 완료 ack
		updateMode++;
	}

}

uint16_t bootVerifyUpdate(void)
{

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
  return (update_meta.crc == crc) ? 0 : 1;
}

void FirmwareUpdateStateMachine(void) {
	switch (fwUpdateState) {
	case FW_UPDATE_IDLE:
		if (fwUpdateRequested) {
			StartFirmwareUpdate();
		}
		break;

	case FW_UPDATE_IN_PROGRESS:
		CAN_RxHeaderTypeDef rxHeader;
		uint16_t totalSize = update_meta.size;

		uint8_t rxData[8];
		uint8_t dataLength = 0;
		// --- 1) 다음 블록 크기 계산 ---
		uint32_t thisBlockSize =
				(totalSize - fwUpdateReceivedBytes >= BLOCK_SIZE_BYTES) ?
						BLOCK_SIZE_BYTES : (totalSize - fwUpdateReceivedBytes);

		if (MessageBufferIsFull()
				|| ((thisBlockSize + fwUpdateReceivedBytes) == totalSize)) {
			while (thisBlockSize > blockReceivedBytes) {

				if (!MessageBufferIsEmpty()) {
					MessageBufferGet(&rxHeader, rxData);

					dataLength = rxHeader.DLC;

					HAL_FLASH_Unlock();

					for (uint8_t i = 0; i < dataLength; i += 2) {
						uint16_t data16 = rxData[i];
						if (i + 1 < dataLength) {
							data16 |= rxData[i + 1] << 8;
						}

						if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
								currentAddress, data16) != HAL_OK) // flash write 오류 rollback 처리 해줘야 한다.
								{
							HAL_FLASH_Lock();
							fwUpdateRequested = 0;
							fwUpdateState = FW_UPDATE_IDLE;

							canMsgWrite(CAN_ID_NACK, 1, TxData); // flash wirte 오류 nack
							return;
						}

						currentAddress += (i + 1 < dataLength) ? 2 : 1;
						fwUpdateReceivedBytes += (i + 1 < dataLength) ? 2 : 1;
						blockReceivedBytes += (i + 1 < dataLength) ? 2 : 1;
					}
					HAL_FLASH_Lock();
				}
			}

			if (fwUpdateReceivedBytes == totalSize) {
				// flash 한 곳의 마지막 4바이트를 읽어 update_meta.crc에 대임
				uint8_t data[4];
				flashRead(currentAddress - sizeof(uint32_t), data, sizeof(uint32_t));
				update_meta.crc = (data[0] << 24) | (data[1] << 16)
								| (data[2] << 8) | data[3];
				fwUpdateState = FW_UPDATE_COMPLETE;
			}
			// 다음 블록을 위해 카운터 리셋

			blockReceivedBytes = 0;

			// --- 3) 블록 완성 시 ACK 전송 ---

			MessageBufferFlush();

			uint8_t data = 0x54;
			canMsgWrite(CAN_ID_ACK, 8, &data);

		}
		break;

	case FW_UPDATE_COMPLETE:
		//여기서 crc 체크 후 meta data를 flash한다.
		if(bootVerifyUpdate())
		{
			Error_Handler();
		}
		if(flashErase(FLASH_ADDR_UPDATE, sizeof(fw_meta_t)) != true)
		{
			Error_Handler();
		}
		if(flashWrite(FLASH_ADDR_UPDATE, (uint8_t *)&update_meta, sizeof(fw_meta_t)) != true)
		{
			Error_Handler();
		}

		uint8_t data = 0x55;
		canMsgWrite(CAN_ID_ACK, 8, &data);
		HAL_PWR_EnableBkUpAccess();
		__HAL_RCC_BKP_CLK_ENABLE();
		BOOT_FLAG_ADDR = BOOT_FLAG_UPDATE;
		HAL_Delay(1000);
		NVIC_SystemReset();
		break;

	default:
		fwUpdateState = FW_UPDATE_IDLE;
		break;
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_SET);
	while (1) {
		// 여기서 rollback 해야한다.
		canMsgWrite(CAN_ID_NACK, 1, TxData);	// update mode no 응답
		HAL_PWR_EnableBkUpAccess();
		__HAL_RCC_BKP_CLK_ENABLE();
		BOOT_FLAG_ADDR = BOOT_FLAG_BOOT;
		NVIC_SystemReset(); 				// 이거는 왜 있는건지 모르겠네
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
