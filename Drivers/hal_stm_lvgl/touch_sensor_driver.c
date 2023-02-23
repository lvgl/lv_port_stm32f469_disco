/*
 * touch_sensor_driver.c
 *
 *  Created on: 19 févr. 2023
 *      Author: morga
 */

#include "main.h"
#include "stm32469i_discovery_lcd.h"
#include "stm32469i_discovery_ts.h"
#include <lvgl.h>
#include <touch_sensor_driver.h>
#include <stdio.h>

static void stm32_touchpad_read_cb(lv_indev_drv_t *indev_drv, lv_indev_data_t *indev_data);

void touch_sensor_driver_init(){

	BSP_TS_Init(NT35510_800X480_WIDTH, NT35510_800X480_HEIGHT);
	static lv_indev_drv_t indev_drv;
	lv_indev_drv_init(&indev_drv);

	indev_drv.type = LV_INDEV_TYPE_POINTER;
	indev_drv.read_cb = stm32_touchpad_read_cb;

	lv_indev_drv_register(&indev_drv);
}

static void stm32_touchpad_read_cb(lv_indev_drv_t *indev_drv, lv_indev_data_t *indev_data){
	static int16_t last_x = 0;
	static int16_t last_y = 0;

	TS_StateTypeDef TS_State;

	BSP_TS_GetState(&TS_State);
	if(TS_State.touchDetected){
		indev_data->point.x = TS_State.touchX[0];
		indev_data->point.y = TS_State.touchY[0];
		indev_data->state = LV_INDEV_STATE_PRESSED;
		last_x = indev_data->point.x;
		last_y = indev_data->point.y;
	}else{
		indev_data->point.x = last_x;
		indev_data->point.y = last_y;
		indev_data->state = LV_INDEV_STATE_RELEASED;
	}
}
