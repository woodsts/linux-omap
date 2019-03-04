/*
 * TI LMU (Lighting Management Unit) Backlight Device Data Definitions
 *
 * Copyright 2015 Texas Instruments
 *
 * Author: Milo Kim <milo.kim@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __TI_LMU_BACKLIGHT_H__
#define __TI_LMU_BACKLIGHT_H__

#include <linux/mfd/ti-lmu.h>
#include <linux/mfd/ti-lmu-register.h>

#define MAX_BRIGHTNESS_8BIT		255
#define MAX_BRIGHTNESS_11BIT		2047

enum ti_lmu_bl_pwm_action {
	/* Update PWM duty, no brightness register update is required */
	UPDATE_PWM_ONLY,
	/* Update not only duty but also brightness register */
	UPDATE_PWM_AND_BRT_REGISTER,
	/* Update max value in brightness registers */
	UPDATE_MAX_BRT,
};

struct lmu_bl_reg_data {
	u8 reg;
	u8 mask;
	u8 val;
};

/**
 * struct ti_lmu_bl_reg
 *
 * @init:		Device initialization registers
 * @num_init:		Numbers of initialization registers
 * @channel:		Backlight channel configuration registers
 * @mode:		Brightness control mode registers
 * @ramp:		Ramp registers for lighting effect
 * @ramp_reg_offset:	Ramp register offset.
 *			Only used for multiple ramp registers.
 * @enable:		Enable control register address
 * @enable_usec:	Delay time for updating enable register.
 *			Unit is microsecond.
 * @brightness_msb:	Brightness MSB(Upper 8 bits) registers.
 *			Concatenated with LSB in 11 bit dimming mode.
 *			In 8 bit dimming, only MSB is used.
 * @brightness_lsb:	Brightness LSB(Lower 3 bits) registers.
 *			Only valid in 11 bit dimming mode.
 */
struct ti_lmu_bl_reg {
	const struct lmu_bl_reg_data *init;
	int num_init;
	const struct lmu_bl_reg_data *channel;
	const struct lmu_bl_reg_data *mode;
	const struct lmu_bl_reg_data *ramp;
	int ramp_reg_offset;
	u8 *enable;
	unsigned long enable_usec;
	u8 *brightness_msb;
	u8 *brightness_lsb;
};

/**
 * struct ti_lmu_bl_cfg
 *
 * @reginfo:		Device register configuration
 * @num_channels:	Number of backlight channels
 * @max_brightness:	Max brightness value of backlight device
 * @pwm_action:		How to control brightness registers in PWM mode
 * @ramp_table:		[Optional] Ramp time table for lighting effect.
 *			It's used for searching approximate register index.
 * @size_ramp:		[Optional] Size of ramp table
 * @fault_monitor_used:	[Optional] Set true if the device needs to handle
 *			LMU fault monitor event.
 *
 * This structure is used for device specific data configuration.
 */
struct ti_lmu_bl_cfg {
	const struct ti_lmu_bl_reg *reginfo;
	int num_channels;
	int max_brightness;
	enum ti_lmu_bl_pwm_action pwm_action;
	int *ramp_table;
	int size_ramp;
	bool fault_monitor_used;
};

extern const struct ti_lmu_bl_cfg lmu_bl_cfg[LMU_MAX_ID];
#endif
