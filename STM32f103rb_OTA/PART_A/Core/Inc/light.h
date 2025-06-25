#ifndef SRC_COMMON_HW_INCLUDE_BUTTON_H_
#define SRC_COMMON_HW_INCLUDE_BUTTON_H_

#include "main.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"


typedef enum {
    DAY = 0,
    NIGHT = 1
} LowBeam;

typedef enum {
    CITY = 0,
    COUNTRY = 1,
    HIGHWAY = 2
}DriveMode;

typedef enum {
    WEAK = 0,
    STRONG = 1
} RainRoad;

typedef enum {
	H_OFF = 0,
	H_ON = 1
}HighBeam;

typedef enum {
    CLEAN = 0,
    FOG = 1
}FogLight;

typedef enum {
    OFF = 0,
	LEFT = 1,
	RIGHT = 2,
	BOTH = 3
}CornerLight;
typedef enum {
	S_OFF = 0,
	S_LEFT = 1,
	S_RIGHT = 2,
	S_BOTH = 3
}SeatMode;

#define LED_NUM 8  // 제어할 WS2812 개수
#define LED_BIT_LEN (24 * LED_NUM)
#define BIT_OFFSET 80

#define WS2812_HIGH 44
#define WS2812_LOW  25

#define THRESHOLD 5

#define LCORNER_ON()   	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET)
#define LCORNER_OFF()  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET)
#define RCORNER_ON()   	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET)
#define RCORNER_OFF()  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET)
#define FOG_ON()		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET)
#define FOG_OFF()		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET)

extern uint16_t ws2812_lbuffer[BIT_OFFSET+LED_BIT_LEN ];
extern uint16_t ws2812_rbuffer[BIT_OFFSET+LED_BIT_LEN ];
extern uint16_t ws2812_hbuffer[BIT_OFFSET+LED_BIT_LEN ];
//uint8_t red = 255, green = 255, blue = 255;    // 빨간색
extern uint8_t lcolors[LED_NUM][3];
extern uint8_t rcolors[LED_NUM][3];
extern uint8_t hcolors[LED_NUM][3];
extern uint8_t change_flag;

//void seat_control(SeatMode mode);
//void can_init(filter_mask,filter_id);
void ws2812_set_lowbeam_colors(uint8_t (*lcolors)[3],uint8_t (*rcolors)[3]);  // colors[i][0]=R, [1]=G, [2]=B
void ws2812_set_highbeam_colors(uint8_t (*hcolors)[3]); // colors[i][0]=R, [1]=G, [2]=B
void ws2812_show(void);
void set_lowbeam(uint8_t brightness);
void set_highbeam(uint8_t brightness);
void light_init(void);
void ws2812_scroll_init(void);// LED 시작시에 애니메이션 기능을 넣으면 좋지않을까? -> 보완해야함
void low_beam_power_control(LowBeam mode); // LED를 0%와 70%로 토글
void driving_mode_change(DriveMode mode); // 스텝모터 드라이버가 없어서 보류
void rainroad_mode_change(RainRoad mode); // LED의 밝기를 70% 와 100%로 토글
void high_beam_power_control(HighBeam mode);
void foglight_power_control(FogLight mode);
void cornerlight_power_control(CornerLight mode);
void lstep_motor(uint8_t step);
void rstep_motor(uint8_t step);;
void rotate_lmotor(uint8_t dir, uint16_t delay_ms, uint16_t steps);
void rotate_rmotor(uint8_t dir, uint16_t delay_ms, uint16_t steps);
void change_light();
void change_state1(uint8_t* data);
void function(uint8_t data);
#endif /* SRC_COMMON_HW_INCLUDE_BUTTON_H_ */
