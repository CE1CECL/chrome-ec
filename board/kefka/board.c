/* Copyright 2016 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */
/* Kefka board-specific configuration */

#include "adc.h"
#include "als.h"
#include "button.h"
#include "charger.h"
#include "charge_state.h"
#include "driver/accel_kionix.h"
#include "driver/gyro_l3gd20h.h"
#include "driver/temp_sensor/tmp432.h"
#include "extpower.h"
#include "gpio.h"
#include "hooks.h"
#include "host_command.h"
#include "i2c.h"
#include "lid_switch.h"
#include "math_util.h"
#include "motion_lid.h"
#include "motion_sense.h"
#include "power.h"
#include "power_button.h"
#include "pwm.h"
#include "pwm_chip.h"
#include "registers.h"
#include "temp_sensor.h"
#include "temp_sensor_chip.h"
#include "thermal.h"
#include "uart.h"
#include "util.h"
#include "keyboard_scan.h"

#define GPIO_KB_INPUT (GPIO_INPUT | GPIO_PULL_UP)
#define GPIO_KB_OUTPUT (GPIO_ODR_HIGH)
#define GPIO_KB_OUTPUT_COL2 (GPIO_OUT_LOW)

#include "gpio_list.h"

/* PWM channels. Must be in the exactly same order as in enum pwm_channel. */
const struct pwm_t pwm_channels[] = {
	{0, PWM_CONFIG_ACTIVE_LOW},
	{1, PWM_CONFIG_ACTIVE_LOW},
	{3, PWM_CONFIG_ACTIVE_LOW},
};

BUILD_ASSERT(ARRAY_SIZE(pwm_channels) == PWM_CH_COUNT);

/* power signal list.  Must match order of enum power_signal. */
const struct power_signal_info power_signal_list[] = {
	{GPIO_ALL_SYS_PGOOD,     1, "ALL_SYS_PWRGD"},
	{GPIO_RSMRST_L_PGOOD,    1, "RSMRST_N_PWRGD"},
	{GPIO_PCH_SLP_S3_L,      1, "SLP_S3#_DEASSERTED"},
	{GPIO_PCH_SLP_S4_L,      1, "SLP_S4#_DEASSERTED"},
};
BUILD_ASSERT(ARRAY_SIZE(power_signal_list) == POWER_SIGNAL_COUNT);

const struct i2c_port_t i2c_ports[]  = {
	{"batt_chg", MEC1322_I2C0_0, 100,
		GPIO_I2C_PORT0_0_SCL, GPIO_I2C_PORT0_0_SDA},
	{"sensors", MEC1322_I2C2, 100,
		GPIO_I2C_PORT2_SCL, GPIO_I2C_PORT2_SDA},
	{"thermal", MEC1322_I2C3, 100,
		GPIO_I2C_PORT3_SCL, GPIO_I2C_PORT3_SDA}
};
const unsigned int i2c_ports_used = ARRAY_SIZE(i2c_ports);

const enum gpio_signal hibernate_wake_pins[] = {
	GPIO_POWER_BUTTON_L,
	GPIO_AC_PRESENT,
};

const int hibernate_wake_pins_used = ARRAY_SIZE(hibernate_wake_pins);

/*
 * Temperature sensors data; must be in same order as enum temp_sensor_id.
 * Sensor index and name must match those present in coreboot:
 *     src/mainboard/google/${board}/acpi/dptf.asl
 */
const struct temp_sensor_t temp_sensors[] = {
	{"TMP432_Internal", TEMP_SENSOR_TYPE_BOARD, tmp432_get_val,
		TMP432_IDX_LOCAL, 4},
	{"TMP432_Power_top", TEMP_SENSOR_TYPE_BOARD, tmp432_get_val,
		TMP432_IDX_REMOTE1, 4},
	{"TMP432_CPU_bottom", TEMP_SENSOR_TYPE_BOARD, tmp432_get_val,
		TMP432_IDX_REMOTE2, 4},
	{"Battery", TEMP_SENSOR_TYPE_BATTERY, charge_temp_sensor_get_val,
		0, 4},
};
BUILD_ASSERT(ARRAY_SIZE(temp_sensors) == TEMP_SENSOR_COUNT);

/* Thermal limits for each temp sensor. All temps are in degrees K. Must be in
 * same order as enum temp_sensor_id. To always ignore any temp, use 0.
 */
struct ec_thermal_config thermal_params[] = {
	{{0, C_TO_K(60), C_TO_K(127)}, 0, 0}, /* TMP432_Internal */
	{{0, C_TO_K(60), C_TO_K(127)}, 0, 0}, /* TMP432_Power_top */
	{{0, C_TO_K(58), C_TO_K(127)}, 0, 0}, /* TMP432_CPU_bottom */
	{{0, 0, 0}, 0, 0}, /* Battery Sensor */
};
BUILD_ASSERT(ARRAY_SIZE(thermal_params) == TEMP_SENSOR_COUNT);

const struct button_config buttons[] = {
	{"Volume Down", KEYBOARD_BUTTON_VOLUME_DOWN, GPIO_VOLUME_DOWN,
		30 * MSEC, 0},
	{"Volume Up", KEYBOARD_BUTTON_VOLUME_UP, GPIO_VOLUME_UP,
		30 * MSEC, 0},
};
BUILD_ASSERT(ARRAY_SIZE(buttons) == CONFIG_BUTTON_COUNT);

/* Four Motion sensors */
/* kxcj9 mutex and local/private data*/
static struct mutex g_kxcj9_mutex[2];
struct kionix_accel_data g_kxcj9_data[2];

/* Matrix to rotate accelrator into standard reference frame */
const matrix_3x3_t base_standard_ref = {
	{ 0,  FLOAT_TO_FP(1),  0},
	{FLOAT_TO_FP(-1),  0,  0},
	{ 0,  0,  FLOAT_TO_FP(1)}
};

const matrix_3x3_t lid_standard_ref = {
	{FLOAT_TO_FP(-1),  0,  0},
	{ 0, FLOAT_TO_FP(1),  0},
	{ 0,  0, FLOAT_TO_FP(-1)}
};

struct motion_sensor_t motion_sensors[] = {
	[BASE_ACCEL] = {
	 .name = "Base",
	 .active_mask = SENSOR_ACTIVE_S0_S3,
	 .chip = MOTIONSENSE_CHIP_KXCJ9,
	 .type = MOTIONSENSE_TYPE_ACCEL,
	 .location = MOTIONSENSE_LOC_BASE,
	 .drv = &kionix_accel_drv,
	 .mutex = &g_kxcj9_mutex[0],
	 .drv_data = &g_kxcj9_data[0],
	 .port = I2C_PORT_ACCEL,
	 .addr = KXCJ9_ADDR1,
	 .rot_standard_ref = &base_standard_ref,
	 .default_range = 2,  /* g, enough for laptop. */
	 .config = {
		 /* AP: by default shutdown all sensors */
		 [SENSOR_CONFIG_AP] = {
			 .odr = 0,
			 .ec_rate = 0,
		 },
		 /* EC use accel for angle detection */
		 [SENSOR_CONFIG_EC_S0] = {
			 .odr = 10000 | ROUND_UP_FLAG,
			 .ec_rate = 0,
		 },
		 /* EC use accel for angle detection */
		 [SENSOR_CONFIG_EC_S3] = {
			 .odr = 10000 | ROUND_UP_FLAG,
			 .ec_rate = 0
		 },
		 /* Sensor off in S5 */
		 [SENSOR_CONFIG_EC_S5] = {
			 .odr = 0,
			 .ec_rate = 0
		 },
	 }
	},
	[LID_ACCEL] = {
	 .name = "Lid",
	 .active_mask = SENSOR_ACTIVE_S0_S3,
	 .chip = MOTIONSENSE_CHIP_KXCJ9,
	 .type = MOTIONSENSE_TYPE_ACCEL,
	 .location = MOTIONSENSE_LOC_LID,
	 .drv = &kionix_accel_drv,
	 .mutex = &g_kxcj9_mutex[1],
	 .drv_data = &g_kxcj9_data[1],
	 .port = I2C_PORT_ACCEL,
	 .addr = KXCJ9_ADDR0,
	 .rot_standard_ref = &lid_standard_ref,
	 .default_range = 2,  /* g, enough for laptop. */
	 .config = {
		 /* AP: by default shutdown all sensors */
		 [SENSOR_CONFIG_AP] = {
			 .odr = 0,
			 .ec_rate = 0,
		 },
		 /* EC use accel for angle detection */
		 [SENSOR_CONFIG_EC_S0] = {
			 .odr = 10000 | ROUND_UP_FLAG,
			 .ec_rate = 0,
		 },
		 /* EC use accel for angle detection */
		 [SENSOR_CONFIG_EC_S3] = {
			 .odr = 10000 | ROUND_UP_FLAG,
			 .ec_rate = 0
		 },
		 /* Sensor off in S5 */
		 [SENSOR_CONFIG_EC_S5] = {
			 .odr = 0,
			 .ec_rate = 0
		 },
	 },
	},
};
unsigned int motion_sensor_count;

static void track_pad_enable(int enable)
{
	gpio_set_level(GPIO_TRACKPAD_PWREN, enable);
	return;
}

void lid_angle_peripheral_enable(int enable)
{
	if (enable) {
		keyboard_scan_enable(1, KB_SCAN_DISABLE_LID_ANGLE);
		track_pad_enable(1);
	} else {
		/*
		* Ensure chipset is off before disabling keyboard. When chipset
		* is on, EC keeps keyboard enabled and the AP decides when to
		* ignore keys based on its more accurate lid angle calculation.
		*
		* TODO(crosbug.com/p/43695): Remove this check once we have a
		* host command that can inform EC when we are entering or
		* exiting tablet mode in S0. Also, add this check back to the
		* function lid_angle_update in lid_angle.c
		*
		*/
		if (!chipset_in_state(CHIPSET_STATE_ON)) {
			keyboard_scan_enable(0, KB_SCAN_DISABLE_LID_ANGLE);
			track_pad_enable(0);
		}
	}
}

/* init ADC ports to avoid floating state due to thermistors */
static void adc_pre_init(void)
{
       /* Configure GPIOs */
	gpio_config_module(MODULE_ADC, 1);
}
DECLARE_HOOK(HOOK_INIT, adc_pre_init, HOOK_PRIO_INIT_ADC - 1);

int i2c_port_is_smbus(int port)
{
	return (port == MEC1322_I2C0_0 || port == MEC1322_I2C0_1) ? 1 : 0;
}

/* To prevent power leakage, need to set touch screen reset pin */
static void touch_screen_control(void)
{

		if (chipset_will_be_in_s0()) {
				/* when platform is at s0, set pin as high */
				gpio_set_level(GPIO_TS_RST_L, 0);
				msleep(1);
				gpio_set_level(GPIO_TS_RST_L, 1);

		} else {
				usleep(20);
				gpio_set_level(GPIO_TS_RST_L, 0);
		}
}
DECLARE_HOOK(HOOK_INIT, touch_screen_control, HOOK_PRIO_DEFAULT);
DECLARE_HOOK(HOOK_CHIPSET_SUSPEND, touch_screen_control, HOOK_PRIO_DEFAULT);
DECLARE_HOOK(HOOK_CHIPSET_RESUME, touch_screen_control, HOOK_PRIO_DEFAULT);
static void get_motion_sensors_count(void)
{
	/*
	 * Get the sensor count by SKU.
	 *
	 * GPIO_BOARD_VERSION1
	 * Sabin : 1
	 * Kefka : 0
	 *
	 */
	if (gpio_get_level(GPIO_BOARD_VERSION1) == 1)
		motion_sensor_count = ARRAY_SIZE(motion_sensors);
	else
		motion_sensor_count = 0;
}
DECLARE_HOOK(HOOK_INIT, get_motion_sensors_count, HOOK_PRIO_FIRST);

uint8_t board_set_battery_level_shutdown(void)
{
	/* Cut off at 5% */
	return 6;
}
