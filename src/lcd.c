/*
 * lcd.c
 *
 *  Created on: 10.03.2023
 *      Author: pvvx
 */
#include "tl_common.h"
#include "app_config.h"
#if (DEV_SERVICES & SERVICE_SCREEN)
#if !((DEVICE_TYPE == DEVICE_MJWSD05MMC) || (DEVICE_TYPE == DEVICE_MJWSD05MMC_EN))
#include "drivers.h"
#include "drivers/8258/gpio_8258.h"
#include "app.h"
#include "i2c.h"
#if (DEV_SERVICES & SERVICE_HARD_CLOCK)
#include "rtc.h"
#endif
#include "lcd.h"
#include "ble.h"
#include "battery.h"

//RAM u8 show_stage; // count/stage update lcd code buffer
//RAM u32 chow_ext_sec; // count show validity time, in sec

//RAM u32 min_step_time_update_lcd; // = cfg.min_step_time_update_lcd * 0.05 sec
//RAM u32 tim_last_chow; // timer show lcd >= 1.5 sec

RAM lcd_flg_t lcd_flg;
#if	(DEVICE_TYPE == DEVICE_ZTH03) || (DEVICE_TYPE == DEVICE_ZYZTH01) || (DEVICE_TYPE == DEVICE_MJWSD06MMC)
RAM u8 display_buff[LCD_BUF_SIZE], display_cmp_buff[LCD_BUF_SIZE+1];
#else
RAM u8 display_buff[LCD_BUF_SIZE], display_cmp_buff[LCD_BUF_SIZE];
#endif

#if (!USE_EPD)
_attribute_ram_code_
void update_lcd(void){
	if(cfg.flg2.screen_off)
		return;
#if	(DEVICE_TYPE == DEVICE_ZTH03) || (DEVICE_TYPE == DEVICE_ZYZTH01) || (DEVICE_TYPE == DEVICE_MJWSD06MMC)
	if (memcmp(&display_cmp_buff[1], &display_buff, sizeof(display_buff))) {
		memcpy(&display_cmp_buff[1], &display_buff, sizeof(display_buff));
		send_to_lcd();
#else
	if (memcmp(&display_cmp_buff, &display_buff, sizeof(display_buff))) {
		send_to_lcd();
		memcpy(&display_cmp_buff, &display_buff, sizeof(display_buff));
#endif
		lcd_flg.b.send_notify = lcd_flg.b.notify_on; // set flag LCD for send notify
	}
}
#endif // !USE_EPD

_attribute_ram_code_
u8 is_comfort(s16 t, u16 h) {
	u8 ret = SMILE_SAD;
	if (t >= cmf.t[0] && t <= cmf.t[1] && h >= cmf.h[0] && h <= cmf.h[1])
		ret = SMILE_HAPPY;
	return ret;
}

_attribute_ram_code_
__attribute__((optimize("-Os")))
void lcd(void) {
	if(cfg.flg2.screen_off)
		return;
	bool set_small_number_and_bat = true;

#if (DEV_SERVICES & SERVICE_KEY) || (DEV_SERVICES & SERVICE_RDS)
	bool _ble_con =	wrk.ble_connected != 0 || (ext_key.rest_adv_int_tad & 2) != 0;
#else
#define _ble_con wrk.ble_connected
#endif
	bool show_ext = lcd_flg.chow_ext_ut >= wrk.utc_time_sec;
	if(cfg.flg.show_time_smile || cfg.flg.show_batt_enabled || show_ext)
		lcd_flg.update_next_measure = 0;
	else
		lcd_flg.update_next_measure = 1;
	if (lcd_flg.chow_ext_ut == 0xffffffff) {
#if	(SHOW_SMILEY)
			show_smiley(*((u8 *) &ext.flg));
#endif
			show_battery_symbol(ext.flg.battery);
#if	(DEVICE_TYPE == DEVICE_CGG1) || (DEVICE_TYPE == DEVICE_CGDK2)
			show_small_number_x10(ext.small_number, ext.flg.percent_on);
#else
			// show_small_number(ext.small_number, ext.flg.percent_on);
#endif
			show_temp_symbol(*((u8 *) &ext.flg));
			show_big_number_x10(ext.big_number);
			show_ble_symbol(_ble_con);
			return;
	}
	if (show_ext && (lcd_flg.show_stage & 2)) { // show ext data
		if (lcd_flg.show_stage & 1) { // stage blinking or show battery or clock
			if (cfg.flg.show_batt_enabled
#if	(DEVICE_TYPE == DEVICE_CGG1) || (DEVICE_TYPE == DEVICE_CGDK2)
#else
				|| measured_data.battery_level <= 5
#endif
				) { // Battery
#if	(SHOW_SMILEY)
				show_smiley(0); // stage show battery
#endif
				show_battery_symbol(1);
#if	(DEVICE_TYPE == DEVICE_CGG1) || (DEVICE_TYPE == DEVICE_CGDK2)
#if DEVICE_TYPE == DEVICE_CGG1
				show_batt_cgg1();
#else
				show_batt_cgdk2();
#endif
#else
				// show_small_number((measured_data.battery_level >= 100) ? 99 : measured_data.battery_level, 1);
#endif // (DEVICE_TYPE == DEVICE_CGG1) || (DEVICE_TYPE == DEVICE_CGDK2)
				set_small_number_and_bat = false;
			} else if (cfg.flg.show_time_smile) { // show clock
#if	USE_DISPLAY_CLOCK
				show_clock(); // stage clock
				show_ble_symbol(_ble_con);
				return;
#else
#if	(SHOW_SMILEY)
				show_smiley(0); // stage clock/blinking and blinking on
#endif
#endif // USE_DISPLAY_CLOCK
			}
#if	(SHOW_SMILEY)
			else
				show_smiley(*((u8 *) &ext.flg));
#endif
		}
#if	(SHOW_SMILEY)
		else
			show_smiley(*((u8 *) &ext.flg));
#endif
		if (set_small_number_and_bat) {
			show_battery_symbol(ext.flg.battery);
#if	(DEVICE_TYPE == DEVICE_CGG1) || (DEVICE_TYPE == DEVICE_CGDK2)
			show_small_number_x10(ext.small_number, ext.flg.percent_on);
#else
			// show_small_number(ext.small_number, ext.flg.percent_on);
#endif
		}
		show_temp_symbol(*((u8 *) &ext.flg));
		show_big_number_x10(ext.big_number);
	} else {
		if (lcd_flg.show_stage & 1) { // stage clock/blinking or show battery
#if	USE_DISPLAY_CLOCK
			if (cfg.flg.show_time_smile && (lcd_flg.show_stage & 2)) {
				show_clock(); // stage clock
				show_ble_symbol(_ble_con);
				return;
			}
#endif
			if (cfg.flg.show_batt_enabled
#if	(DEVICE_TYPE == DEVICE_CGG1) || (DEVICE_TYPE == DEVICE_CGDK2) || (DEVICE_TYPE == DEVICE_LKTMZL02)

#else
				|| measured_data.battery_level <= 5
#endif
				) { // Battery
#if	(SHOW_SMILEY)
				show_smiley(0); // stage show battery
#endif
				show_battery_symbol(1);
#if	(DEVICE_TYPE == DEVICE_CGG1) || (DEVICE_TYPE == DEVICE_CGDK2)
#if DEVICE_TYPE == DEVICE_CGG1
				show_batt_cgg1();
#else
				show_batt_cgdk2();
#endif
#else
				// show_small_number((measured_data.battery_level >= 100) ? 99 : measured_data.battery_level, 1);
#endif // (DEVICE_TYPE == DEVICE_CGG1) || (DEVICE_TYPE == DEVICE_CGDK2)
				set_small_number_and_bat = false;
			} else if (cfg.flg.show_time_smile) { // show clock
#if	USE_DISPLAY_CLOCK
				show_clock(); // stage clock
				show_ble_symbol(_ble_con);
				return;
#else
#if	(SHOW_SMILEY)
				show_smiley(0); // stage blinking and blinking on
#endif
#endif // USE_DISPLAY_CLOCK
			} else {
#if	(SHOW_SMILEY)
				if (cfg.flg.comfort_smiley) { // comfort on
					show_smiley(is_comfort(measured_data.temp, measured_data.humi));
				} else
					show_smiley(cfg.flg2.smiley);
#endif
			}
		} else {
#if	(SHOW_SMILEY)
			if (cfg.flg.comfort_smiley) { // comfort on
				show_smiley(is_comfort(measured_data.temp, measured_data.humi));
			} else
				show_smiley(cfg.flg2.smiley);
#endif
		}
		if (set_small_number_and_bat) {
#if	(DEVICE_TYPE == DEVICE_CGG1) || (DEVICE_TYPE == DEVICE_CGDK2)
			show_battery_symbol(!cfg.flg.show_batt_enabled);
			show_small_number_x10(measured_data.humi_x01, 1);
#else
#if	(DEVICE_TYPE == DEVICE_LKTMZL02)
			show_battery_symbol(!cfg.flg.show_batt_enabled);
#else
			show_battery_symbol(measured_data.battery_level);
#endif
			show_small_number(measured_data.humi_x01, 1);
#endif
		}
		if (cfg.flg.temp_F_or_C) {
			show_temp_symbol(TMP_SYM_F); // "°F"
			show_big_number_x10(((s32)((s32)measured_data.temp * 9)/ 50) + 320); // convert C to F
		} else {
			show_temp_symbol(TMP_SYM_C); // "°C"
			show_big_number_x10(measured_data.temp_x01);
		}
	}
	show_ble_symbol(_ble_con);
}

#endif // (DEVICE_TYPE != DEVICE_MJWSD05MMC)
#endif // (DEV_SERVICES & SERVICE_SCREEN)
