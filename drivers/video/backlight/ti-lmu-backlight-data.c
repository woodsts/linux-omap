/*
 * TI LMU (Lighting Management Unit) Backlight Device Data
 *
 * Copyright 2015 Texas Instruments
 *
 * Author: Milo Kim <milo.kim@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "ti-lmu-backlight-data.h"

/* LM3532 */
static const struct lmu_bl_reg_data lm3532_init_data[] = {
	{ LM3532_REG_ZONE_CFG_A, LM3532_ZONE_MASK, LM3532_ZONE_0 },
	{ LM3532_REG_ZONE_CFG_B, LM3532_ZONE_MASK, LM3532_ZONE_1 },
	{ LM3532_REG_ZONE_CFG_C, LM3532_ZONE_MASK, LM3532_ZONE_2 },
};

static const struct lmu_bl_reg_data lm3532_channel_data[] = {
	{ LM3532_REG_OUTPUT_CFG, LM3532_ILED1_CFG_MASK,
	  LM3532_ILED1_CFG_SHIFT },
	{ LM3532_REG_OUTPUT_CFG, LM3532_ILED2_CFG_MASK,
	  LM3532_ILED2_CFG_SHIFT },
	{ LM3532_REG_OUTPUT_CFG, LM3532_ILED3_CFG_MASK,
	  LM3532_ILED3_CFG_SHIFT },
};

static const struct lmu_bl_reg_data lm3532_mode_data[] = {
	{ LM3532_REG_PWM_A_CFG, LM3532_PWM_A_MASK, LM3532_PWM_ZONE_0 },
	{ LM3532_REG_PWM_B_CFG, LM3532_PWM_B_MASK, LM3532_PWM_ZONE_1 },
	{ LM3532_REG_PWM_C_CFG, LM3532_PWM_C_MASK, LM3532_PWM_ZONE_2 },
};

static const struct lmu_bl_reg_data lm3532_ramp_data[] = {
	{ LM3532_REG_RAMPUP, LM3532_RAMPUP_MASK, LM3532_RAMPUP_SHIFT },
	{ LM3532_REG_RAMPDN, LM3532_RAMPDN_MASK, LM3532_RAMPDN_SHIFT },
};

static u8 lm3532_enable_reg = LM3532_REG_ENABLE;

static u8 lm3532_brightness_regs[] = {
	LM3532_REG_BRT_A,
	LM3532_REG_BRT_B,
	LM3532_REG_BRT_C,
};

static const struct ti_lmu_bl_reg lm3532_reg_info = {
	.init		= lm3532_init_data,
	.num_init	= ARRAY_SIZE(lm3532_init_data),
	.channel	= lm3532_channel_data,
	.mode		= lm3532_mode_data,
	.ramp		= lm3532_ramp_data,
	.enable		= &lm3532_enable_reg,
	.brightness_msb	= lm3532_brightness_regs,
};

/* LM3631 */
static const struct lmu_bl_reg_data lm3631_init_data[] = {
	{ LM3631_REG_BRT_MODE, LM3631_MODE_MASK, LM3631_DEFAULT_MODE },
	{ LM3631_REG_BL_CFG, LM3631_MAP_MASK, LM3631_EXPONENTIAL_MAP },
};

static const struct lmu_bl_reg_data lm3631_channel_data[] = {
	{ LM3631_REG_BL_CFG, LM3631_BL_CHANNEL_MASK, LM3631_BL_SINGLE_CHANNEL },
	{ LM3631_REG_BL_CFG, LM3631_BL_CHANNEL_MASK, LM3631_BL_DUAL_CHANNEL },
};

static const struct lmu_bl_reg_data lm3631_ramp_data[] = {
	{ LM3631_REG_SLOPE, LM3631_SLOPE_MASK, LM3631_SLOPE_SHIFT },
};

static u8 lm3631_enable_reg = LM3631_REG_DEVCTRL;
static u8 lm3631_brightness_msb_reg = LM3631_REG_BRT_MSB;
static u8 lm3631_brightness_lsb_reg = LM3631_REG_BRT_LSB;

static const struct ti_lmu_bl_reg lm3631_reg_info = {
	.init		= lm3631_init_data,
	.num_init	= ARRAY_SIZE(lm3631_init_data),
	.channel	= lm3631_channel_data,
	.ramp		= lm3631_ramp_data,
	.enable		= &lm3631_enable_reg,
	.brightness_msb	= &lm3631_brightness_msb_reg,
	.brightness_lsb	= &lm3631_brightness_lsb_reg,
};

/* LM3632 */
static const struct lmu_bl_reg_data lm3632_init_data[] = {
	{ LM3632_REG_CONFIG1, LM3632_OVP_MASK, LM3632_OVP_25V },
	{ LM3632_REG_CONFIG2, LM3632_SWFREQ_MASK, LM3632_SWFREQ_1MHZ },
};

static const struct lmu_bl_reg_data lm3632_channel_data[] = {
	{ LM3632_REG_ENABLE, LM3632_BL_CHANNEL_MASK, LM3632_BL_SINGLE_CHANNEL },
	{ LM3632_REG_ENABLE, LM3632_BL_CHANNEL_MASK, LM3632_BL_DUAL_CHANNEL },
};

static const struct lmu_bl_reg_data lm3632_mode_data[] = {
	{ LM3632_REG_IO_CTRL, LM3632_PWM_MASK, LM3632_PWM_MODE },
};

static u8 lm3632_enable_reg = LM3632_REG_ENABLE;
static u8 lm3632_brightness_msb_reg = LM3632_REG_BRT_MSB;
static u8 lm3632_brightness_lsb_reg = LM3632_REG_BRT_LSB;

static const struct ti_lmu_bl_reg lm3632_reg_info = {
	.init		= lm3632_init_data,
	.num_init	= ARRAY_SIZE(lm3632_init_data),
	.channel	= lm3632_channel_data,
	.mode		= lm3632_mode_data,
	.enable		= &lm3632_enable_reg,
	.brightness_msb	= &lm3632_brightness_msb_reg,
	.brightness_lsb	= &lm3632_brightness_lsb_reg,
};

/* LM3633 */
static const struct lmu_bl_reg_data lm3633_init_data[] = {
	{ LM3633_REG_BOOST_CFG, LM3633_OVP_MASK, LM3633_OVP_40V },
	{ LM3633_REG_BL_RAMP_CONF, LM3633_BL_RAMP_MASK, LM3633_BL_RAMP_EACH },
};

static const struct lmu_bl_reg_data lm3633_channel_data[] = {
	{ LM3633_REG_HVLED_OUTPUT_CFG, LM3633_HVLED1_CFG_MASK,
	  LM3633_HVLED1_CFG_SHIFT },
	{ LM3633_REG_HVLED_OUTPUT_CFG, LM3633_HVLED2_CFG_MASK,
	  LM3633_HVLED2_CFG_SHIFT },
	{ LM3633_REG_HVLED_OUTPUT_CFG, LM3633_HVLED3_CFG_MASK,
	  LM3633_HVLED3_CFG_SHIFT },
};

static const struct lmu_bl_reg_data lm3633_mode_data[] = {
	{ LM3633_REG_PWM_CFG, LM3633_PWM_A_MASK, LM3633_PWM_A_MASK },
	{ LM3633_REG_PWM_CFG, LM3633_PWM_B_MASK, LM3633_PWM_B_MASK },
};

static const struct lmu_bl_reg_data lm3633_ramp_data[] = {
	{ LM3633_REG_BL0_RAMP, LM3633_BL_RAMPUP_MASK, LM3633_BL_RAMPUP_SHIFT },
	{ LM3633_REG_BL0_RAMP, LM3633_BL_RAMPDN_MASK, LM3633_BL_RAMPDN_SHIFT },
};

static u8 lm3633_enable_reg = LM3633_REG_ENABLE;

static u8 lm3633_brightness_msb_regs[] = {
	LM3633_REG_BRT_HVLED_A_MSB,
	LM3633_REG_BRT_HVLED_B_MSB,
};

static u8 lm3633_brightness_lsb_regs[] = {
	LM3633_REG_BRT_HVLED_A_LSB,
	LM3633_REG_BRT_HVLED_B_LSB,
};

static const struct ti_lmu_bl_reg lm3633_reg_info = {
	.init		 = lm3633_init_data,
	.num_init	 = ARRAY_SIZE(lm3633_init_data),
	.channel	 = lm3633_channel_data,
	.mode		 = lm3633_mode_data,
	.ramp		 = lm3633_ramp_data,
	.ramp_reg_offset = 1, /* For LM3633_REG_BL1_RAMPUP/DN */
	.enable		 = &lm3633_enable_reg,
	.brightness_msb	 = lm3633_brightness_msb_regs,
	.brightness_lsb	 = lm3633_brightness_lsb_regs,
};

/* LM3695 */
static const struct lmu_bl_reg_data lm3695_init_data[] = {
	{ LM3695_REG_GP, LM3695_BRT_RW_MASK, LM3695_BRT_RW_MASK },
};

static const struct lmu_bl_reg_data lm3695_channel_data[] = {
	{ LM3695_REG_GP, LM3695_BL_CHANNEL_MASK, LM3695_BL_SINGLE_CHANNEL },
	{ LM3695_REG_GP, LM3695_BL_CHANNEL_MASK, LM3695_BL_DUAL_CHANNEL },
};

static u8 lm3695_enable_reg = LM3695_REG_GP;
static u8 lm3695_brightness_msb_reg = LM3695_REG_BRT_MSB;
static u8 lm3695_brightness_lsb_reg = LM3695_REG_BRT_LSB;

static const struct ti_lmu_bl_reg lm3695_reg_info = {
	.init		= lm3695_init_data,
	.num_init	= ARRAY_SIZE(lm3695_init_data),
	.channel	= lm3695_channel_data,
	.enable		= &lm3695_enable_reg,
	.enable_usec	= 600,
	.brightness_msb	= &lm3695_brightness_msb_reg,
	.brightness_lsb	= &lm3695_brightness_lsb_reg,
};

/* LM3697 */
static const struct lmu_bl_reg_data lm3697_init_data[] = {
	{ LM3697_REG_RAMP_CONF, LM3697_RAMP_MASK, LM3697_RAMP_EACH },
};

static const struct lmu_bl_reg_data lm3697_channel_data[] = {
	{ LM3697_REG_HVLED_OUTPUT_CFG, LM3697_HVLED1_CFG_MASK,
	  LM3697_HVLED1_CFG_SHIFT },
	{ LM3697_REG_HVLED_OUTPUT_CFG, LM3697_HVLED2_CFG_MASK,
	  LM3697_HVLED2_CFG_SHIFT },
	{ LM3697_REG_HVLED_OUTPUT_CFG, LM3697_HVLED3_CFG_MASK,
	  LM3697_HVLED3_CFG_SHIFT },
};

static const struct lmu_bl_reg_data lm3697_mode_data[] = {
	{ LM3697_REG_PWM_CFG, LM3697_PWM_A_MASK, LM3697_PWM_A_MASK },
	{ LM3697_REG_PWM_CFG, LM3697_PWM_B_MASK, LM3697_PWM_B_MASK },
};

static const struct lmu_bl_reg_data lm3697_ramp_data[] = {
	{ LM3697_REG_BL0_RAMP, LM3697_RAMPUP_MASK, LM3697_RAMPUP_SHIFT },
	{ LM3697_REG_BL0_RAMP, LM3697_RAMPDN_MASK, LM3697_RAMPDN_SHIFT },
};

static u8 lm3697_enable_reg = LM3697_REG_ENABLE;

static u8 lm3697_brightness_msb_regs[] = {
	LM3697_REG_BRT_A_MSB,
	LM3697_REG_BRT_B_MSB,
};

static u8 lm3697_brightness_lsb_regs[] = {
	LM3697_REG_BRT_A_LSB,
	LM3697_REG_BRT_B_LSB,
};

static const struct ti_lmu_bl_reg lm3697_reg_info = {
	.init		 = lm3697_init_data,
	.num_init	 = ARRAY_SIZE(lm3697_init_data),
	.channel	 = lm3697_channel_data,
	.mode		 = lm3697_mode_data,
	.ramp		 = lm3697_ramp_data,
	.ramp_reg_offset = 1, /* For LM3697_REG_BL1_RAMPUP/DN */
	.enable		 = &lm3697_enable_reg,
	.brightness_msb	 = lm3697_brightness_msb_regs,
	.brightness_lsb	 = lm3697_brightness_lsb_regs,
};

static int lm3532_ramp_table[] = { 0, 1, 2, 4, 8, 16, 32, 65 };

static int lm3631_ramp_table[] = {
	   0,   1,   2,    5,   10,   20,   50,  100,
	 250, 500, 750, 1000, 1500, 2000, 3000, 4000,
};

static int common_ramp_table[] = {
	   2, 250, 500, 1000, 2000, 4000, 8000, 16000,
};

#define LM3532_MAX_CHANNELS		3
#define LM3631_MAX_CHANNELS		2
#define LM3632_MAX_CHANNELS		2
#define LM3633_MAX_CHANNELS		3
#define LM3695_MAX_CHANNELS		2
#define LM3697_MAX_CHANNELS		3

const struct ti_lmu_bl_cfg lmu_bl_cfg[LMU_MAX_ID] = {
	{
		.reginfo		= &lm3532_reg_info,
		.num_channels		= LM3532_MAX_CHANNELS,
		.max_brightness		= MAX_BRIGHTNESS_8BIT,
		.pwm_action		= UPDATE_PWM_AND_BRT_REGISTER,
		.ramp_table		= lm3532_ramp_table,
		.size_ramp		= ARRAY_SIZE(lm3532_ramp_table),
	},
	{
		.reginfo		= &lm3631_reg_info,
		.num_channels		= LM3631_MAX_CHANNELS,
		.max_brightness		= MAX_BRIGHTNESS_11BIT,
		.pwm_action		= UPDATE_PWM_ONLY,
		.ramp_table		= lm3631_ramp_table,
		.size_ramp		= ARRAY_SIZE(lm3631_ramp_table),
	},
	{
		.reginfo		= &lm3632_reg_info,
		.num_channels		= LM3632_MAX_CHANNELS,
		.max_brightness		= MAX_BRIGHTNESS_11BIT,
		.pwm_action		= UPDATE_PWM_ONLY,
	},
	{
		.reginfo		= &lm3633_reg_info,
		.num_channels		= LM3633_MAX_CHANNELS,
		.max_brightness		= MAX_BRIGHTNESS_11BIT,
		.pwm_action		= UPDATE_MAX_BRT,
		.ramp_table		= common_ramp_table,
		.size_ramp		= ARRAY_SIZE(common_ramp_table),
		.fault_monitor_used	= true,
	},
	{
		.reginfo		= &lm3695_reg_info,
		.num_channels		= LM3695_MAX_CHANNELS,
		.max_brightness		= MAX_BRIGHTNESS_11BIT,
		.pwm_action		= UPDATE_PWM_AND_BRT_REGISTER,
	},
	{
		.reginfo		= &lm3697_reg_info,
		.num_channels		= LM3697_MAX_CHANNELS,
		.max_brightness		= MAX_BRIGHTNESS_11BIT,
		.pwm_action		= UPDATE_PWM_AND_BRT_REGISTER,
		.ramp_table		= common_ramp_table,
		.size_ramp		= ARRAY_SIZE(common_ramp_table),
		.fault_monitor_used	= true,
	},
};
