/* Copyright 2018 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */

#include "adc.h"
#include "adc_chip.h"
#include "backlight.h"
#include "button.h"
#include "charge_manager.h"
#include "charge_ramp.h"
#include "charge_state.h"
#include "charger.h"
#include "chipset.h"
#include "common.h"
#include "console.h"
#include "driver/accelgyro_bmi160.h"
#include "driver/als_tcs3400.h"
#include "driver/battery/max17055.h"
#include "driver/bc12/pi3usb9201.h"
#include "driver/charger/rt946x.h"
#include "driver/sync.h"
#include "driver/tcpm/mt6370.h"
#include "ec_commands.h"
#include "extpower.h"
#include "gpio.h"
#include "hooks.h"
#include "host_command.h"
#include "i2c.h"
#include "lid_switch.h"
#include "power.h"
#include "power_button.h"
#include "pwm.h"
#include "pwm_chip.h"
#include "registers.h"
#include "spi.h"
#include "system.h"
#include "task.h"
#include "tcpm.h"
#include "timer.h"
#include "usb_charge.h"
#include "usb_mux.h"
#include "usb_pd_tcpm.h"
#include "util.h"

#define CPRINTS(format, args...) cprints(CC_USBCHARGE, format, ## args)
#define CPRINTF(format, args...) cprintf(CC_USBCHARGE, format, ## args)

static void tcpc_alert_event(enum gpio_signal signal)
{
	schedule_deferred_pd_interrupt(0 /* port */);
}

static void gauge_interrupt(enum gpio_signal signal)
{
	task_wake(TASK_ID_CHARGER);
}

#include "gpio_list.h"

/******************************************************************************/
/* ADC channels. Must be in the exactly same order as in enum adc_channel. */
const struct adc_t adc_channels[] = {
	[ADC_BOARD_ID] = {"BOARD_ID", 3300, 4096, 0, STM32_AIN(10)},
	[ADC_EC_SKU_ID] = {"EC_SKU_ID", 3300, 4096, 0, STM32_AIN(8)},
	[ADC_BATT_ID] = {"BATT_ID", 3300, 4096, 0, STM32_AIN(7)},
	[ADC_POGO_ADC_INT_L] = {"POGO_ADC_INT_L", 3300, 4096, 0, STM32_AIN(6)},
};
BUILD_ASSERT(ARRAY_SIZE(adc_channels) == ADC_CH_COUNT);

/******************************************************************************/
/* I2C ports */
const struct i2c_port_t i2c_ports[] = {
	{"charger",   I2C_PORT_CHARGER,   400, GPIO_I2C1_SCL, GPIO_I2C1_SDA},
	{"tcpc0",     I2C_PORT_TCPC0,     400, GPIO_I2C1_SCL, GPIO_I2C1_SDA},
	{"battery",   I2C_PORT_BATTERY,   400, GPIO_I2C2_SCL, GPIO_I2C2_SDA},
	{"accelgyro", I2C_PORT_ACCEL,     400, GPIO_I2C2_SCL, GPIO_I2C2_SDA},
	{"bc12",      I2C_PORT_BC12,      400, GPIO_I2C2_SCL, GPIO_I2C2_SDA},
	{"als",       I2C_PORT_ALS,       400, GPIO_I2C2_SCL, GPIO_I2C2_SDA},
};
const unsigned int i2c_ports_used = ARRAY_SIZE(i2c_ports);

#define BC12_I2C_ADDR_FLAGS PI3USB9201_I2C_ADDR_3_FLAGS

/* power signal list.  Must match order of enum power_signal. */
const struct power_signal_info power_signal_list[] = {
	{GPIO_AP_IN_SLEEP_L,   POWER_SIGNAL_ACTIVE_LOW,  "AP_IN_S3_L"},
	{GPIO_PMIC_EC_RESETB,  POWER_SIGNAL_ACTIVE_HIGH, "PMIC_PWR_GOOD"},
};
BUILD_ASSERT(ARRAY_SIZE(power_signal_list) == POWER_SIGNAL_COUNT);

/******************************************************************************/
/* SPI devices */
const struct spi_device_t spi_devices[] = {
};
const unsigned int spi_devices_used = ARRAY_SIZE(spi_devices);

/******************************************************************************/
const struct tcpc_config_t tcpc_config[CONFIG_USB_PD_PORT_COUNT] = {
	{
		.bus_type = EC_BUS_TYPE_I2C,
		.i2c_info = {
			.port = I2C_PORT_TCPC0,
			.addr_flags = MT6370_TCPC_I2C_ADDR_FLAGS,
		},
		.drv = &mt6370_tcpm_drv,
	},
};

struct usb_mux usb_muxes[CONFIG_USB_PD_PORT_COUNT] = {
	{
		.driver = &virtual_usb_mux_driver,
		.hpd_update = &virtual_hpd_update,
	},
};

void board_reset_pd_mcu(void)
{
}

uint16_t tcpc_get_alert_status(void)
{
	uint16_t status = 0;

	if (!gpio_get_level(GPIO_USB_C0_PD_INT_ODL))
		status |= PD_STATUS_TCPC_ALERT_0;

	return status;
}

static void board_pogo_charge_init(void)
{
	int i;

	/* Initialize all charge suppliers to 0 */
	for (i = 0; i < CHARGE_SUPPLIER_COUNT; i++)
		charge_manager_update_charge(i, CHARGE_PORT_POGO, NULL);
}
DECLARE_HOOK(HOOK_INIT, board_pogo_charge_init,
	     HOOK_PRIO_CHARGE_MANAGER_INIT + 1);

static int force_discharge;

int board_set_active_charge_port(int charge_port)
{
	CPRINTS("New chg p%d", charge_port);

	/* ignore all request when discharge mode is on */
	if (force_discharge)
		return EC_SUCCESS;

	switch (charge_port) {
	case CHARGE_PORT_USB_C:
		/* Don't charge from a source port */
		if (board_vbus_source_enabled(charge_port))
			return -1;
		gpio_set_level(GPIO_EN_POGO_CHARGE_L, 1);
		gpio_set_level(GPIO_EN_USBC_CHARGE_L, 0);
		break;
	case CHARGE_PORT_POGO:
		gpio_set_level(GPIO_EN_USBC_CHARGE_L, 1);
		gpio_set_level(GPIO_EN_POGO_CHARGE_L, 0);
		break;
	case CHARGE_PORT_NONE:
		/*
		 * To ensure the fuel gauge (max17055) is always powered
		 * even when battery is disconnected, keep VBAT rail on but
		 * set the charging current to minimum.
		 */
		gpio_set_level(GPIO_EN_POGO_CHARGE_L, 1);
		gpio_set_level(GPIO_EN_USBC_CHARGE_L, 1);
		charger_set_current(0);
		break;
	default:
		panic("Invalid charge port\n");
		break;
	}

	return EC_SUCCESS;
}

void board_set_charge_limit(int port, int supplier, int charge_ma,
			    int max_ma, int charge_mv)
{
	charge_set_input_current_limit(MAX(charge_ma,
			       CONFIG_CHARGER_INPUT_CURRENT), charge_mv);
}

int board_discharge_on_ac(int enable)
{
	int ret, port;

	if (enable) {
		port = CHARGE_PORT_NONE;
	} else {
		/* restore the charge port state */
		port = charge_manager_get_override();
		if (port == OVERRIDE_OFF)
			port = charge_manager_get_active_charge_port();
	}

	ret = board_set_active_charge_port(port);
	if (ret)
		return ret;
	force_discharge = enable;

	return charger_discharge_on_ac(enable);
}

int extpower_is_present(void)
{
	/*
	 * The charger will indicate VBUS presence if we're sourcing 5V,
	 * so exclude such ports.
	 */
	int usb_c_extpower_present;

	if (board_vbus_source_enabled(CHARGE_PORT_USB_C))
		usb_c_extpower_present = 0;
	else
		usb_c_extpower_present = tcpm_get_vbus_level(CHARGE_PORT_USB_C);

	return usb_c_extpower_present || gpio_get_level(GPIO_POGO_VBUS_PRESENT);
}

int pd_snk_is_vbus_provided(int port)
{
	if (port)
		panic("Invalid charge port\n");

	return rt946x_is_vbus_ready();
}

#if defined(BOARD_KUKUI) || defined(BOARD_KODAMA)
/* dummy interrupt function for kukui */
void pogo_adc_interrupt(enum gpio_signal signal)
{
}
#endif

static void board_init(void)
{
	/* If the reset cause is external, pulse PMIC force reset. */
	if (system_get_reset_flags() == RESET_FLAG_RESET_PIN) {
		gpio_set_level(GPIO_PMIC_FORCE_RESET_ODL, 0);
		msleep(100);
		gpio_set_level(GPIO_PMIC_FORCE_RESET_ODL, 1);
	}

	/* Set SPI1 PB13/14/15 pins to high speed */
	STM32_GPIO_OSPEEDR(GPIO_B) |= 0xfc000000;

	/* Enable TCPC alert interrupts */
	gpio_enable_interrupt(GPIO_USB_C0_PD_INT_ODL);

	/* Enable charger interrupts */
	gpio_enable_interrupt(GPIO_CHARGER_INT_ODL);

#ifdef SECTION_IS_RW
	/* Enable interrupts from BMI160 sensor. */
	gpio_enable_interrupt(GPIO_ACCEL_INT_ODL);

	/* Enable interrupt for the camera vsync. */
	gpio_enable_interrupt(GPIO_SYNC_INT);
#endif /* SECTION_IS_RW */

	/* Enable interrupt from PMIC. */
	gpio_enable_interrupt(GPIO_PMIC_EC_RESETB);

	/* Enable gauge interrupt from max17055 */
	gpio_enable_interrupt(GPIO_GAUGE_INT_ODL);

	/* Enable pogo interrupt */
	gpio_enable_interrupt(GPIO_POGO_ADC_INT_L);

	if (IS_ENABLED(BOARD_KRANE))
		/* Display bias settings.  */
		mt6370_db_set_voltages(6000, 5800, 5800);

	/* Enable pogo charging signal */
	gpio_enable_interrupt(GPIO_POGO_VBUS_PRESENT);
}
DECLARE_HOOK(HOOK_INIT, board_init, HOOK_PRIO_DEFAULT);

static void board_rev_init(void)
{
	/* Board revision specific configs. */

	/*
	 * It's a P1 pin BOOTBLOCK_MUX_OE, also a P2 pin BC12_DET_EN.
	 * Keep this pin defaults to P1 setting since that eMMC enabled with
	 * High-Z stat.
	 */
	if (IS_ENABLED(BOARD_KUKUI) && board_get_version() == 1)
		gpio_set_flags(GPIO_BC12_DET_EN, GPIO_ODR_HIGH);

	if (board_get_version() >= 2) {
		/*
		 * Enable MT6370 DB_POSVOUT/DB_NEGVOUT (controlled by _EN pins).
		 */
		mt6370_db_external_control(1);
	}

	if (board_get_version() == 2) {
		/* configure PI3USB9201 to USB Path ON Mode */
		i2c_write8(I2C_PORT_BC12, BC12_I2C_ADDR_FLAGS,
			   PI3USB9201_REG_CTRL_1,
			   (PI3USB9201_USB_PATH_ON <<
			    PI3USB9201_REG_CTRL_1_MODE_SHIFT));
	}
}
DECLARE_HOOK(HOOK_INIT, board_rev_init, HOOK_PRIO_INIT_ADC + 1);

void board_config_pre_init(void)
{
	STM32_RCC_AHBENR |= STM32_RCC_HB_DMA1;
	/*
	 * Remap USART1 and SPI2 DMA:
	 *
	 * Ch4: USART1_TX / Ch5: USART1_RX (1000)
	 * Ch6: SPI2_RX / Ch7: SPI2_TX (0011)
	 */
	STM32_DMA_CSELR(STM32_DMAC_CH4) = (8 << 12) | (8 << 16) |
					  (3 << 20) | (3 << 24);
}

enum kukui_board_version {
	BOARD_VERSION_UNKNOWN = -1,
	BOARD_VERSION_REV0 = 0,
	BOARD_VERSION_REV1 = 1,
	BOARD_VERSION_REV2 = 2,
	BOARD_VERSION_REV3 = 3,
	BOARD_VERSION_REV4 = 4,
	BOARD_VERSION_REV5 = 5,
	BOARD_VERSION_REV6 = 6,
	BOARD_VERSION_REV7 = 7,
	BOARD_VERSION_REV8 = 8,
	BOARD_VERSION_REV9 = 9,
	BOARD_VERSION_REV10 = 10,
	BOARD_VERSION_REV11 = 11,
	BOARD_VERSION_REV12 = 12,
	BOARD_VERSION_REV13 = 13,
	BOARD_VERSION_REV14 = 14,
	BOARD_VERSION_REV15 = 15,
	BOARD_VERSION_COUNT,
};

struct {
	enum kukui_board_version version;
	int expect_mv;
} const kukui_boards[] = {
	{ BOARD_VERSION_REV0, 109 },   /* 51.1K , 2.2K(gru 3.3K) ohm */
	{ BOARD_VERSION_REV1, 211 },   /* 51.1k , 6.8K ohm */
	{ BOARD_VERSION_REV2, 319 },   /* 51.1K , 11K ohm */
	{ BOARD_VERSION_REV3, 427 },   /* 56K   , 17.4K ohm */
	{ BOARD_VERSION_REV4, 542 },   /* 51.1K , 22K ohm */
	{ BOARD_VERSION_REV5, 666 },   /* 51.1K , 30K ohm */
	{ BOARD_VERSION_REV6, 781 },   /* 51.1K , 39.2K ohm */
	{ BOARD_VERSION_REV7, 900 },   /* 56K   , 56K ohm */
	{ BOARD_VERSION_REV8, 1023 },  /* 47K   , 61.9K ohm */
	{ BOARD_VERSION_REV9, 1137 },  /* 47K   , 80.6K ohm */
	{ BOARD_VERSION_REV10, 1240 }, /* 56K   , 124K ohm */
	{ BOARD_VERSION_REV11, 1343 }, /* 51.1K , 150K ohm */
	{ BOARD_VERSION_REV12, 1457 }, /* 47K   , 200K ohm */
	{ BOARD_VERSION_REV13, 1576 }, /* 47K   , 330K ohm */
	{ BOARD_VERSION_REV14, 1684 }, /* 47K   , 680K ohm */
	{ BOARD_VERSION_REV15, 1800 }, /* 56K   , NC */
};
BUILD_ASSERT(ARRAY_SIZE(kukui_boards) == BOARD_VERSION_COUNT);

#define THRESHOLD_MV 56 /* Simply assume 1800/16/2 */

int board_get_version(void)
{
	static int version = BOARD_VERSION_UNKNOWN;
	int mv;
	int i;

	if (version != BOARD_VERSION_UNKNOWN)
		return version;

	gpio_set_level(GPIO_EC_BOARD_ID_EN_L, 0);
	/* Wait to allow cap charge */
	msleep(10);
	mv = adc_read_channel(ADC_BOARD_ID);

	if (mv == ADC_READ_ERROR)
		mv = adc_read_channel(ADC_BOARD_ID);

	gpio_set_level(GPIO_EC_BOARD_ID_EN_L, 1);

	for (i = 0; i < BOARD_VERSION_COUNT; ++i) {
		if (mv < kukui_boards[i].expect_mv + THRESHOLD_MV) {
			version = kukui_boards[i].version;
			break;
		}
	}

	return version;
}

/* Motion sensors */
/* Mutexes */
#ifdef SECTION_IS_RW
static struct mutex g_lid_mutex;

static struct bmi160_drv_data_t g_bmi160_data;

static struct als_drv_data_t g_tcs3400_data = {
	.als_cal.scale = 1,
	.als_cal.uscale = 0,
	.als_cal.offset = 0,
};

static struct tcs3400_rgb_drv_data_t g_tcs3400_rgb_data = {
	.device_scale = 1,
	.device_uscale = 0,
	.rgb_cal[X] = {
		.scale = ALS_CHANNEL_SCALE(1),
		.offset = 0,
	},
	.rgb_cal[Y] = {
		.scale = ALS_CHANNEL_SCALE(1),
		.offset = 0,
	},
	.rgb_cal[Z] = {
		.scale = ALS_CHANNEL_SCALE(1),
		.offset = 0,
	},
};

#ifdef BOARD_KRANE
/* Matrix to rotate accelerometer into standard reference frame */
static const mat33_fp_t lid_standard_ref_rev3 = {
	{0, FLOAT_TO_FP(-1), 0},
	{FLOAT_TO_FP(1), 0, 0},
	{0, 0, FLOAT_TO_FP(1)}
};
#endif /* BOARD_KRANE */

/* Matrix to rotate accelerometer into standard reference frame */
static const mat33_fp_t lid_standard_ref = {
	{FLOAT_TO_FP(1), 0, 0},
	{0, FLOAT_TO_FP(1), 0},
	{0, 0, FLOAT_TO_FP(1)}
};

#ifdef CONFIG_MAG_BMI160_BMM150
/* Matrix to rotate accelrator into standard reference frame */
static const mat33_fp_t mag_standard_ref = {
	{0, FLOAT_TO_FP(-1), 0},
	{FLOAT_TO_FP(-1), 0, 0},
	{0, 0, FLOAT_TO_FP(-1)}
};
#endif /* CONFIG_MAG_BMI160_BMM150 */

struct motion_sensor_t motion_sensors[] = {
	/*
	 * Note: bmi160: supports accelerometer and gyro sensor
	 * Requirement: accelerometer sensor must init before gyro sensor
	 * DO NOT change the order of the following table.
	 */
	[LID_ACCEL] = {
	 .name = "Accel",
	 .active_mask = SENSOR_ACTIVE_S0_S3,
	 .chip = MOTIONSENSE_CHIP_BMI160,
	 .type = MOTIONSENSE_TYPE_ACCEL,
	 .location = MOTIONSENSE_LOC_LID,
	 .drv = &bmi160_drv,
	 .mutex = &g_lid_mutex,
	 .drv_data = &g_bmi160_data,
	 .port = I2C_PORT_ACCEL,
	 .i2c_spi_addr_flags = BMI160_ADDR0_FLAGS,
	 .rot_standard_ref = &lid_standard_ref,
	 .default_range = 4,  /* g */
	 .min_frequency = BMI160_ACCEL_MIN_FREQ,
	 .max_frequency = BMI160_ACCEL_MAX_FREQ,
	 .config = {
		 /* Enable accel in S0 */
		 [SENSOR_CONFIG_EC_S0] = {
			 .odr = 10000 | ROUND_UP_FLAG,
			 .ec_rate = 100 * MSEC,
		 },
	 },
	},
	[LID_GYRO] = {
	 .name = "Gyro",
	 .active_mask = SENSOR_ACTIVE_S0_S3,
	 .chip = MOTIONSENSE_CHIP_BMI160,
	 .type = MOTIONSENSE_TYPE_GYRO,
	 .location = MOTIONSENSE_LOC_LID,
	 .drv = &bmi160_drv,
	 .mutex = &g_lid_mutex,
	 .drv_data = &g_bmi160_data,
	 .port = I2C_PORT_ACCEL,
	 .i2c_spi_addr_flags = BMI160_ADDR0_FLAGS,
	 .default_range = 1000, /* dps */
	 .rot_standard_ref = &lid_standard_ref,
	 .min_frequency = BMI160_GYRO_MIN_FREQ,
	 .max_frequency = BMI160_GYRO_MAX_FREQ,
	},
#ifdef CONFIG_MAG_BMI160_BMM150
	[LID_MAG] = {
	 .name = "Lid Mag",
	 .active_mask = SENSOR_ACTIVE_S0_S3,
	 .chip = MOTIONSENSE_CHIP_BMI160,
	 .type = MOTIONSENSE_TYPE_MAG,
	 .location = MOTIONSENSE_LOC_LID,
	 .drv = &bmi160_drv,
	 .mutex = &g_lid_mutex,
	 .drv_data = &g_bmi160_data,
	 .port = I2C_PORT_ACCEL,
	 .i2c_spi_addr_flags = BMI160_ADDR0_FLAGS,
	 .default_range = BIT(11), /* 16LSB / uT, fixed */
	 .rot_standard_ref = &mag_standard_ref,
	 .min_frequency = BMM150_MAG_MIN_FREQ,
	 .max_frequency = BMM150_MAG_MAX_FREQ(SPECIAL),
	},
#endif /* CONFIG_MAG_BMI160_BMM150 */
	[CLEAR_ALS] = {
	 .name = "Clear Light",
	 .active_mask = SENSOR_ACTIVE_S0_S3,
	 .chip = MOTIONSENSE_CHIP_TCS3400,
	 .type = MOTIONSENSE_TYPE_LIGHT,
	 .location = MOTIONSENSE_LOC_LID,
	 .drv = &tcs3400_drv,
	 .drv_data = &g_tcs3400_data,
	 .port = I2C_PORT_ALS,
	 .i2c_spi_addr_flags = TCS3400_I2C_ADDR_FLAGS,
	 .rot_standard_ref = NULL,
	 .default_range = 0x10000, /* scale = 1x, uscale = 0 */
	 .min_frequency = TCS3400_LIGHT_MIN_FREQ,
	 .max_frequency = TCS3400_LIGHT_MAX_FREQ,
	 .config = {
		 /* Run ALS sensor in S0 */
		[SENSOR_CONFIG_EC_S0] = {
			.odr = 1000,
		},
	 },
	},
	[RGB_ALS] = {
	.name = "RGB Light",
	 .active_mask = SENSOR_ACTIVE_S0_S3,
	 .chip = MOTIONSENSE_CHIP_TCS3400,
	 .type = MOTIONSENSE_TYPE_LIGHT_RGB,
	 .location = MOTIONSENSE_LOC_LID,
	 .drv = &tcs3400_rgb_drv,
	 .drv_data = &g_tcs3400_rgb_data,
	 /*.port = I2C_PORT_ALS,*/ /* Unused. RGB channels read by CLEAR_ALS. */
	 .rot_standard_ref = NULL,
	 .default_range = 0x10000, /* scale = 1x, uscale = 0 */
	 .min_frequency = 0, /* 0 indicates we should not use sensor directly */
	 .max_frequency = 0, /* 0 indicates we should not use sensor directly */
	},
	[VSYNC] = {
	 .name = "Camera vsync",
	 .active_mask = SENSOR_ACTIVE_S0,
	 .chip = MOTIONSENSE_CHIP_GPIO,
	 .type = MOTIONSENSE_TYPE_SYNC,
	 .location = MOTIONSENSE_LOC_CAMERA,
	 .drv = &sync_drv,
	 .default_range = 0,
	 .min_frequency = 0,
	 .max_frequency = 1,
	},
};
const unsigned int motion_sensor_count = ARRAY_SIZE(motion_sensors);
const struct motion_sensor_t *motion_als_sensors[] = {
	&motion_sensors[CLEAR_ALS],
};

#endif /* SECTION_IS_RW */

#ifdef BOARD_KRANE
static void fix_krane(void)
{
	if (board_get_version() != 3)
		return;

	/*
	 * Fix backlight led maximum current: tolerance 120mA * 0.75 = 90mA.
	 * (b/133655155)
	 */
	mt6370_backlight_set_dim(MT6370_BLDIM_DEFAULT * 3 / 4);

#ifdef SECTION_IS_RW
	/* Fix reference point */
	motion_sensors[LID_ACCEL].rot_standard_ref = &lid_standard_ref_rev3;
	motion_sensors[LID_GYRO].rot_standard_ref = &lid_standard_ref_rev3;
#endif /* SECTION_IS_RW */
}
DECLARE_HOOK(HOOK_INIT, fix_krane, HOOK_PRIO_INIT_ADC + 1);
#endif /* BOARD_KRANE */

int board_allow_i2c_passthru(int port)
{
	return (port == I2C_PORT_VIRTUAL_BATTERY);
}

void usb_charger_set_switches(int port, enum usb_switch setting)
{
}

/*
 * Return if VBUS is sagging too low
 */
int board_is_vbus_too_low(int port, enum chg_ramp_vbus_state ramp_state)
{
	/*
	 * Though we have a more tolerant range (3.9V~13.4V), setting 4400 to
	 * prevent from a bad charger crashed.
	 *
	 * TODO(b:131284131): mt6370 VBUS reading is not accurate currently.
	 * Vendor will provide a workaround solution to fix the gap between ADC
	 * reading and actual voltage.  After the workaround applied, we could
	 * try to raise this value to 4600.  (when it says it read 4400, it is
	 * actually close to 4600)
	 */
	return charger_get_vbus_voltage(port) < 4400;
}

int board_charge_port_is_sink(int port)
{
	/* TODO(b:128386458): Check POGO_ADC_INT_L */
	return 1;
}

int board_charge_port_is_connected(int port)
{
	return gpio_get_level(GPIO_POGO_VBUS_PRESENT);
}

void board_fill_source_power_info(int port,
				  struct ec_response_usb_pd_power_info *r)
{
	r->meas.voltage_now = 3300;
	r->meas.voltage_max = 3300;
	r->meas.current_max = 1500;
	r->meas.current_lim = 1500;
	r->max_power = r->meas.voltage_now * r->meas.current_max;
}
