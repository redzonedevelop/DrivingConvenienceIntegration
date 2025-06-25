#include "light.h"

uint16_t ws2812_lbuffer[BIT_OFFSET+LED_BIT_LEN ];
uint16_t ws2812_rbuffer[BIT_OFFSET+LED_BIT_LEN ];
uint16_t ws2812_hbuffer[BIT_OFFSET+LED_BIT_LEN ];
uint8_t lcolors[LED_NUM][3];
uint8_t rcolors[LED_NUM][3];
uint8_t hcolors[LED_NUM][3];
uint8_t change_flag;


void ws2812_set_lowbeam_colors(uint8_t (*lcolors)[3],uint8_t (*rcolors)[3])  // colors[i][0]=R, [1]=G, [2]=B

{
    uint32_t i = 0;
    for (int led = 0; led < LED_NUM; led++)
    {
        uint8_t g = lcolors[led][1];
        uint8_t r = lcolors[led][0];
        uint8_t b = lcolors[led][2];

        for (; i <  80; i++) ws2812_lbuffer[i] = 0;
        for (int8_t bit = 7; bit >= 0; bit--) ws2812_lbuffer[i++] = (g >> bit) & 1 ? WS2812_HIGH : WS2812_LOW;
        for (int8_t bit = 7; bit >= 0; bit--) ws2812_lbuffer[i++] = (r >> bit) & 1 ? WS2812_HIGH : WS2812_LOW;
        for (int8_t bit = 7; bit >= 0; bit--) ws2812_lbuffer[i++] = (b >> bit) & 1 ? WS2812_HIGH : WS2812_LOW;
    }

    i = 0;

	for (int led = 0; led < LED_NUM; led++)
	{
		uint8_t g = rcolors[led][1];
		uint8_t r = rcolors[led][0];
		uint8_t b = rcolors[led][2];

		for (; i < 80; i++) ws2812_rbuffer[i] = 0;
		for (int8_t bit = 7; bit >= 0; bit--) ws2812_rbuffer[i++] = (g >> bit) & 1 ? WS2812_HIGH : WS2812_LOW;
		for (int8_t bit = 7; bit >= 0; bit--) ws2812_rbuffer[i++] = (r >> bit) & 1 ? WS2812_HIGH : WS2812_LOW;
		for (int8_t bit = 7; bit >= 0; bit--) ws2812_rbuffer[i++] = (b >> bit) & 1 ? WS2812_HIGH : WS2812_LOW;

	}
}
void ws2812_set_highbeam_colors(uint8_t (*hcolors)[3])  // colors[i][0]=R, [1]=G, [2]=B

{
    uint32_t i = 0;
    for (int led = 0; led < LED_NUM; led++)
    {
        uint8_t g = hcolors[led][1];
        uint8_t r = hcolors[led][0];
        uint8_t b = hcolors[led][2];

        for (; i <  80; i++) ws2812_hbuffer[i] = 0;
        for (int8_t bit = 7; bit >= 0; bit--) ws2812_hbuffer[i++] = (g >> bit) & 1 ? WS2812_HIGH : WS2812_LOW;
        for (int8_t bit = 7; bit >= 0; bit--) ws2812_hbuffer[i++] = (r >> bit) & 1 ? WS2812_HIGH : WS2812_LOW;
        for (int8_t bit = 7; bit >= 0; bit--) ws2812_hbuffer[i++] = (b >> bit) & 1 ? WS2812_HIGH : WS2812_LOW;
    }


}
void ws2812_show(void)
{
    // DMA로 PWM 전송 시작
    HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_1, (uint32_t*)ws2812_lbuffer, LED_BIT_LEN + BIT_OFFSET);
    HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_2, (uint32_t*)ws2812_rbuffer, LED_BIT_LEN + BIT_OFFSET);
    HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_2, (uint32_t*)ws2812_hbuffer, LED_BIT_LEN + BIT_OFFSET);
    // reset time 확보를 위해 전송 후 잠시 대기
    HAL_Delay(2);  // 최소 50µs 이상 필요 (1ms이면 충분)
}
void set_lowbeam(uint8_t brightness)
{
	for (int i = 0; i < LED_NUM; i++)
	{
		lcolors[i][0] = lcolors[i][1] = lcolors[i][2] = brightness;
		rcolors[i][0] = rcolors[i][1] = rcolors[i][2] = brightness;
	}
	ws2812_set_lowbeam_colors(lcolors,rcolors);
	ws2812_show();
}
void set_highbeam(uint8_t brightness)
{
	for (int i = 0; i < LED_NUM; i++)
	{
		hcolors[i][0] = hcolors[i][1] = hcolors[i][2] = brightness;
	}
	ws2812_set_highbeam_colors(hcolors);
	ws2812_show();
}
void light_init(void)
{
	ws2812_scroll_init();
	set_lowbeam(255);
	set_highbeam(255);
	HAL_Delay(500);

	set_lowbeam(125);
	set_highbeam(125);
	HAL_Delay(500);

	set_lowbeam(0);
	set_highbeam(0);
	HAL_Delay(500);
}
void ws2812_scroll_init(void)// LED 시작시에 애니메이션 기능을 넣으면 좋지않을까? -> 보완해야함
{
//	while (1)
//	{
//	for (int i = 0; i < LED_NUM; i++)
//	{
//	// 모든 LED 끄기
//		for (int j = 0; j < LED_NUM; j++)
//		{
//		  lcolors[j][0] = lcolors[j][1] = lcolors[j][2] = 0;
//		  rcolors[j][0] = rcolors[j][1] = rcolors[j][2] = 0;
//		}
//
//		// 현재 위치만 흰색
//		lcolors[i][0] = rcolors[i][0] = 255;  // R
//		lcolors[i][1] = rcolors[i][0] = 255;  // G
//		lcolors[i][2] = rcolors[i][0] = 255;  // B
//
//		ws2812_set_lowbeam_colors(lcolors,rcolors);
//		ws2812_show();
//
//		HAL_Delay(100);  // 속도 조절
//		}
//	}
	for (int i = 0; i < LED_NUM; i++) {
		  	        lcolors[i][0] = lcolors[i][1] = lcolors[i][2] = 0;
		  	        rcolors[i][0] = rcolors[i][1] = rcolors[i][2] = 0;
		  	    }

		  	    for (int i = 0; i < LED_NUM; i++) {
		  	        // i번째까지 흰색으로 설정
		  	        for (int j = 0; j <= i; j++) {
		  	            lcolors[j][0] = lcolors[j][1] = lcolors[j][2] = 255;  // 왼쪽
		  	            rcolors[j][0] = rcolors[j][1] = rcolors[j][2] = 255;  // 오른쪽
		  	        }

		  	        ws2812_set_lowbeam_colors(lcolors, rcolors);
		  	        ws2812_show();
		  	        HAL_Delay(100);  // 속도 조절
		  	    }
}
void low_beam_power_control(LowBeam mode) // LED를 0%와 70%로 토글
{
	static uint8_t day_count = 0;
	static uint8_t night_count = 0;
	static LowBeam last_input = -1;
	static LowBeam current_mode = -1;
	if (mode != last_input)
	{
		day_count = 0;
		night_count = 0;
	}
	switch(mode)
	{
	case DAY:
		day_count++;
		break;
	case NIGHT:
		night_count++;
		break;
	}


	if (day_count >= THRESHOLD && current_mode !=DAY)
	{
		uint8_t brightness = 0;
		set_lowbeam(brightness);
		current_mode = DAY;
		day_count = 0;
		night_count = 0;
	}

	if (night_count >= THRESHOLD && current_mode != NIGHT)
	{
		uint8_t brightness = 178;
		set_lowbeam(brightness);
		current_mode = NIGHT;
		day_count = 0;
		night_count = 0;
	}

	last_input = mode;
}/*
void driving_mode_change(DriveMode mode) // 스텝모터 드라이버가 없어서 보류
{
	static uint8_t city_count = 0;
	static uint8_t country_count = 0;
	static uint8_t highway_count = 0;

	static DriveMode last_input = -1;
	static DriveMode current_mode = -1;
	if (mode != last_input)
	{
		city_count = 0;
		country_count = 0;
		highway_count = 0;
	}
	switch(mode)
	{
	case CITY:
		city_count++;
		break;
	case COUNTRY:
		country_count++;
		break;
	case HIGHWAY:
		highway_count++;
		break;
	}

	if (city_count >= THRESHOLD && current_mode !=CITY)
	{

		current_mode = CITY;
		city_count = 0;
		country_count = 0;
		highway_count = 0;

	}

	if (country_count >= THRESHOLD && current_mode != COUNTRY)
	{
		if(current_mode == HIGHWAY)
		{
			rotate_rmotor(1, 5, 100); // 반시계 방향 100스텝
			rotate_lmotor(0, 5, 100); // 시계 방향 100스텝
		}

		current_mode = COUNTRY;
		city_count = 0;
		country_count = 0;
		highway_count = 0;
	}
	if (highway_count >= THRESHOLD && current_mode != HIGHWAY)
	{

		rotate_rmotor(0, 5, 100); // 시계 방향 100스텝
		rotate_lmotor(1, 5, 100); // 반시계 방향 100스텝

		current_mode = HIGHWAY;
		city_count = 0;
		country_count = 0;
		highway_count = 0;
	}


	last_input = mode;

}*/
void rainroad_mode_change(RainRoad mode) // LED의 밝기를 70% 와 100%로 토글
{
    static uint8_t weak_count = 0;
    static uint8_t strong_count = 0;
    static RainRoad last_input = -1;
    static RainRoad current_mode = -1;
    if (mode != last_input)
    {
		weak_count = 0;
		strong_count = 0;
    }

    switch(mode)
    {
    case WEAK:
    	weak_count++;
    	break;
    case STRONG:
    	strong_count++;
    	break;
    }

    if (weak_count >= THRESHOLD && current_mode!=WEAK)
    {
		uint8_t brightness = 178;
		set_lowbeam(brightness);
		current_mode = WEAK;
		weak_count = 0;
		strong_count = 0;
	}

	if (strong_count >= THRESHOLD && current_mode!=STRONG)
	{
		uint8_t brightness = 255;
		set_lowbeam(brightness);
		current_mode = STRONG;
		weak_count = 0;
		strong_count = 0;
	}

	last_input = mode;

}
void high_beam_power_control(HighBeam mode)
{
	static uint8_t on_count = 0;
	static uint8_t off_count = 0;
	static HighBeam last_input = -1;
	static HighBeam current_mode = -1;
	if (mode != last_input)
	{
		on_count = 0;
		off_count = 0;
	}
	switch(mode)
	{
	case H_ON:
		on_count++;
		break;
	case H_OFF:
		off_count++;
		break;
	}


	if (on_count >= THRESHOLD && current_mode !=H_ON)
	{
		uint8_t brightness = 255;
		set_highbeam(brightness);
		current_mode = H_ON;
		on_count = 0;
		off_count = 0;
	}

	if (off_count >= THRESHOLD && current_mode != H_OFF)
	{
		uint8_t brightness = 0;
		set_highbeam(brightness);
		current_mode = H_OFF;
		on_count = 0;
		off_count = 0;
	}

	last_input = mode;
}
void foglight_power_control(FogLight mode)
{
	static uint8_t clean_count = 0;
	static uint8_t fog_count = 0;
	static FogLight last_input = -1;
	static FogLight current_mode = -1;
	if (mode != last_input)
	{
		clean_count = 0;
		fog_count = 0;
	}
	switch(mode)
	{
	case CLEAN:
		clean_count++;
		break;
	case FOG:
		fog_count++;
		break;
	}


	if (clean_count >= THRESHOLD && current_mode !=CLEAN)
	{
		FOG_OFF();
		//안개등 작동 off
		current_mode = CLEAN;
		clean_count = 0;
		fog_count = 0;
	}

	if (fog_count >= THRESHOLD && current_mode != FOG)
	{
		FOG_ON();
		//안개등 작동 on
		current_mode = FOG;
		clean_count = 0;
		fog_count = 0;
	}

	last_input = mode;
}
void cornerlight_power_control(CornerLight mode)
{
	static uint8_t off_count = 0;
	static uint8_t both_count = 0;
	static uint8_t right_count = 0;
	static uint8_t left_count = 0;

	static CornerLight last_input = -1;
	static CornerLight current_mode = -1;
	if (mode != last_input)
	{
		off_count = 0;
		both_count = 0;
		right_count = 0;
		left_count = 0;
	}
	switch(mode)
	{
	case OFF:
		off_count++;
		break;
	case BOTH:
		both_count++;
		break;
	case RIGHT:
		right_count++;
		break;
	case LEFT:
		left_count++;
		break;
	}

	if (off_count >= THRESHOLD && current_mode !=OFF)
	{
		LCORNER_OFF();
		RCORNER_OFF();

		current_mode = OFF;
		off_count = 0;
		both_count = 0;
		right_count = 0;
		left_count = 0;
	}

	if (both_count >= THRESHOLD && current_mode != BOTH)
	{
		LCORNER_ON();
		RCORNER_ON();

		current_mode = BOTH;
		off_count = 0;
		both_count = 0;
		right_count = 0;
		left_count = 0;
	}
	if (right_count >= THRESHOLD && current_mode != RIGHT)
	{
		LCORNER_OFF();
		RCORNER_ON();

		current_mode = RIGHT;
		off_count = 0;
		both_count = 0;
		right_count = 0;
		left_count = 0;
	}
	if (left_count >= THRESHOLD && current_mode != LEFT)
	{
		LCORNER_ON();
		RCORNER_OFF();

		current_mode = LEFT;
		off_count = 0;
		both_count = 0;
		right_count = 0;
		left_count = 0;
	}

	last_input = mode;
}
/*void lstep_motor(uint8_t step)
{
    HAL_GPIO_WritePin(LSTEPA_GPIO_Port, LSTEPA_Pin, (step == 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LSTEPB_GPIO_Port, LSTEPB_Pin, (step == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LSTEPC_GPIO_Port, LSTEPC_Pin, (step == 2) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LSTEPD_GPIO_Port, LSTEPD_Pin, (step == 3) ? GPIO_PIN_SET : GPIO_PIN_RESET);

}
void rstep_motor(uint8_t step)
{

    HAL_GPIO_WritePin(RSTEPA_GPIO_Port, RSTEPA_Pin, (step == 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RSTEPB_GPIO_Port, RSTEPB_Pin, (step == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RSTEPC_GPIO_Port, RSTEPC_Pin, (step == 2) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RSTEPD_GPIO_Port, RSTEPD_Pin, (step == 3) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}*/
/*void rotate_lmotor(uint8_t dir, uint16_t delay_ms, uint16_t steps)
{
    for (int i = 0; i < steps; i++) {
        uint8_t step = dir ? (i % 4) : (3 - (i % 4));
        lstep_motor(step);
        HAL_Delay(delay_ms);
    }
}
void rotate_rmotor(uint8_t dir, uint16_t delay_ms, uint16_t steps)
{
    for (int i = 0; i < steps; i++) {
        uint8_t step = dir ? (i % 4) : (3 - (i % 4));
        rstep_motor(step);
        HAL_Delay(delay_ms);
    }
}*/
void change_light(uint8_t *lightMode){
	low_beam_power_control(lightMode[0]);
	/*driving_mode_change(lightMode[1]);*/
	rainroad_mode_change(lightMode[2]);
	high_beam_power_control(lightMode[3]);
	foglight_power_control(lightMode[4]);
	cornerlight_power_control(lightMode[5]);
}


/**
  * @brief  The application entry point.
  * @retval int
  */
