/* Copyright 2016 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */

/* Caroline board configuration */

#ifndef __CROS_EC_BOARD_H
#define __CROS_EC_BOARD_H

/*
 * By default, enable all console messages except HC, ACPI and event:
 * The sensor stack is generating a lot of activity.
 */
#define CC_DEFAULT     (CC_ALL & ~(CC_MASK(CC_EVENTS) | CC_MASK(CC_LPC)))
#undef CONFIG_HOSTCMD_DEBUG_MODE
#define CONFIG_HOSTCMD_DEBUG_MODE HCDEBUG_OFF

/* Reduce shared memory space for FIFO */
#undef CONFIG_SHARED_MEM_MIN_SIZE
#define CONFIG_SHARED_MEM_MIN_SIZE 512

/* Optional features */
#define CONFIG_ADC
#define CONFIG_BATTERY_CRITICAL_SHUTDOWN_CUT_OFF
#define CONFIG_BATTERY_CUT_OFF
#undef CONFIG_BATTERY_LEVEL_NEAR_FULL
#define CONFIG_BATTERY_LEVEL_NEAR_FULL 96
#define CONFIG_BATTERY_PRESENT_GPIO GPIO_BAT_PRESENT_L
#define CONFIG_BATTERY_SMART
#define CONFIG_BATTERY_REVIVE_DISCONNECT
#define CONFIG_BOARD_SPECIFIC_VERSION
#define CONFIG_BOARD_HAS_RTC_RESET
#define CONFIG_BOARD_VERSION
#define CONFIG_BUTTON_COUNT 2
#define CONFIG_CHARGE_MANAGER
#define CONFIG_CHARGE_RAMP_HW

#define CONFIG_CHARGER
#define CONFIG_CHARGER_V2

#define CONFIG_CHARGER_DISCHARGE_ON_AC
#define CONFIG_CHARGER_ISL9237
#define CONFIG_CHARGER_ILIM_PIN_DISABLED
#define CONFIG_CHARGER_INPUT_CURRENT 512
#define CONFIG_CHARGER_LIMIT_POWER_THRESH_BAT_PCT BATTERY_LEVEL_SHUTDOWN
#define CONFIG_CHARGER_LIMIT_POWER_THRESH_CHG_MW 25000
#define CONFIG_CHARGER_LIMIT_POWER_ENFORCE_RO_THRESH
#define CONFIG_CHARGER_MIN_BAT_PCT_FOR_POWER_ON 1
#define CONFIG_CHARGER_NARROW_VDC
#define CONFIG_CHARGER_SENSE_RESISTOR 10
#define CONFIG_CHARGER_SENSE_RESISTOR_AC 20

#define CONFIG_CHIPSET_SKYLAKE
#define CONFIG_CHIPSET_RESET_HOOK
#define CONFIG_CLOCK_CRYSTAL
#define CONFIG_DEBUG_ASSERT_BRIEF
#define CONFIG_DPTF_DEVICE_ORIENTATION
#define CONFIG_DYNAMIC_MOTION_SENSOR_COUNT
#define CONFIG_EXTPOWER_GPIO
#define CONFIG_HOSTCMD_ALIGNED
#define CONFIG_HOSTCMD_PD
#define CONFIG_I2C
#define CONFIG_I2C_MASTER
#define CONFIG_KBLIGHT_MAX14521
#define CONFIG_KEYBOARD_COL2_INVERTED
#define CONFIG_KEYBOARD_PROTOCOL_8042
#define CONFIG_KEYBOARD_VIVALDI
#define CONFIG_LED_COMMON
#define CONFIG_LID_SWITCH
#define CONFIG_LOW_POWER_IDLE
#define CONFIG_LTO
#define CONFIG_POWER_BUTTON
#define CONFIG_POWER_BUTTON_X86
#define CONFIG_POWER_COMMON
#define CONFIG_POWER_S0IX
#define CONFIG_POWER_SIGNAL_INTERRUPT_STORM_DETECT_THRESHOLD 30
#undef CONFIG_PWM
#define CONFIG_PWM_KBLIGHT
#define CONFIG_PWM_KBLIGHT_CHIP
/* All data won't fit in data RAM.  So, moving boundary slightly. */
#undef CONFIG_RO_SIZE
#define CONFIG_RO_SIZE (104 * 1024)
#define CONFIG_SCI_GPIO GPIO_PCH_SCI_L
/* We're space constrained on Caroline, so reduce the UART TX buffer size. */
#undef CONFIG_UART_TX_BUF_SIZE
#define CONFIG_UART_TX_BUF_SIZE 512
#define CONFIG_USB_CHARGER
#define CONFIG_USB_MUX_PI3USB30532
#define CONFIG_USB_MUX_PS8740
#define CONFIG_USB_POWER_DELIVERY
#define CONFIG_USB_PD_ALT_MODE
#define CONFIG_USB_PD_ALT_MODE_DFP
#define CONFIG_USB_PD_COMM_LOCKED
#define CONFIG_USB_PD_CUSTOM_VDM
#define CONFIG_USB_PD_DISCHARGE_GPIO
#define CONFIG_USB_PD_DUAL_ROLE
#define CONFIG_USB_PD_LOGGING
#define CONFIG_USB_PD_LOG_SIZE 128
#define CONFIG_USB_PD_PORT_COUNT 2
#define CONFIG_USB_PD_TCPM_TCPCI
#define CONFIG_USB_PD_TRY_SRC
#define CONFIG_USB_SWITCH_PI3USB9281
#define CONFIG_USB_SWITCH_PI3USB9281_CHIP_COUNT 2
#define CONFIG_USBC_SS_MUX
#define CONFIG_USBC_SS_MUX_DFP_ONLY
#define CONFIG_USBC_VCONN
#define CONFIG_USBC_VCONN_SWAP
#define CONFIG_VBOOT_HASH

#define CONFIG_SPI_FLASH_PORT 1
#define CONFIG_SPI_FLASH
#define CONFIG_FLASH_SIZE 524288
#define CONFIG_SPI_FLASH_GD25Q41B

#define CONFIG_TABLET_MODE

#define CONFIG_TEMP_SENSOR
#define CONFIG_TEMP_SENSOR_BD99992GW
#define CONFIG_THERMISTOR_NCP15WB

/*
 * Enable 1 slot of secure temporary storage to support
 * suspend/resume with read/write memory training.
 */
#define CONFIG_VSTORE
#define CONFIG_VSTORE_SLOT_COUNT 1

#define CONFIG_WATCHDOG_HELP

#define CONFIG_WIRELESS
#define CONFIG_WIRELESS_SUSPEND \
	(EC_WIRELESS_SWITCH_WLAN | EC_WIRELESS_SWITCH_WLAN_POWER)

/* Wireless signals */
#define WIRELESS_GPIO_WLAN GPIO_WLAN_OFF_L
#define WIRELESS_GPIO_WLAN_POWER GPIO_PP3300_WLAN_EN

/* I2C ports */
#define I2C_PORT_PMIC MEC1322_I2C0_0
#define I2C_PORT_KBLIGHT MEC1322_I2C0_0
#define I2C_PORT_USB_CHARGER_1 MEC1322_I2C0_1
#define I2C_PORT_USB_MUX MEC1322_I2C0_1
#define I2C_PORT_USB_CHARGER_2 MEC1322_I2C0_0
#define I2C_PORT_PD_MCU MEC1322_I2C1
#define I2C_PORT_TCPC MEC1322_I2C1
#define I2C_PORT_ACCEL MEC1322_I2C2
#define I2C_PORT_ALS MEC1322_I2C2
#define I2C_PORT_BATTERY MEC1322_I2C3
#define I2C_PORT_CHARGER MEC1322_I2C3

/* Thermal sensors read through PMIC ADC interface */
#define I2C_PORT_THERMAL I2C_PORT_PMIC

/* Sensors */
#define CONFIG_MKBP_EVENT
#define CONFIG_MKBP_USE_HOST_EVENT
#define CONFIG_ACCELGYRO_BMI160
#define CONFIG_ACCELGYRO_BMI160_INT_EVENT TASK_EVENT_CUSTOM(4)
#define CONFIG_ACCELGYRO_BMI160_INT2_OUTPUT
#define CONFIG_ACCEL_BMA255
#define CONFIG_ACCEL_INTERRUPTS
#define CONFIG_ALS_BH1730
#define CONFIG_LID_ANGLE
#define CONFIG_LID_ANGLE_SENSOR_BASE BASE_ACCEL
#define CONFIG_LID_ANGLE_SENSOR_LID LID_ACCEL
#define CONFIG_LID_ANGLE_UPDATE
#define CONFIG_LID_ANGLE_TABLET_MODE
#define CONFIG_LID_ANGLE_INVALID_CHECK
/* FIFO size is in power of 2. */
#define CONFIG_ACCEL_FIFO 128
/* Parameter to calculate LUX on caroline */
#define CONFIG_ALS_BH1730_LUXTH_PARAMS
#define BH1730_LUXTH1_1K                270
#define BH1730_LUXTH1_D0_1K             4000
#define BH1730_LUXTH1_D1_1K             6364
#define BH1730_LUXTH2_1K                740
#define BH1730_LUXTH2_D0_1K             3088
#define BH1730_LUXTH2_D1_1K             3010
#define BH1730_LUXTH3_1K                1030
#define BH1730_LUXTH3_D0_1K             2056
#define BH1730_LUXTH3_D1_1K             1608
#define BH1730_LUXTH4_1K                3670
#define BH1730_LUXTH4_D0_1K             550
#define BH1730_LUXTH4_D1_1K             149
/* Depends on how fast the AP boots and typical ODRs */
#define CONFIG_ACCEL_FIFO_THRES (CONFIG_ACCEL_FIFO / 3)

/* Modules we want to exclude */
#undef CONFIG_CMD_APTHROTTLE
#undef CONFIG_CMD_BATTFAKE
#undef CONFIG_CMD_HASH
#undef CONFIG_CMD_HIBDELAY
#undef CONFIG_CMD_I2C_SCAN
#undef CONFIG_CMD_I2C_XFER
#undef CONFIG_CMD_IDLE_STATS
#undef CONFIG_CMD_POWERINDEBUG
#undef CONFIG_CMD_SLEEPMASK
#undef CONFIG_CMD_SYSJUMP
#undef CONFIG_CMD_TEMP_SENSOR
#undef CONFIG_CMD_TIMERINFO
#undef CONFIG_CMD_TYPEC
#undef CONFIG_CONSOLE_CMDHELP
#undef CONFIG_CONSOLE_HISTORY
#undef CONFIG_PECI
#undef CONFIG_TASK_PROFILING

#undef DEFERRABLE_MAX_COUNT
#define DEFERRABLE_MAX_COUNT 17

#ifndef __ASSEMBLER__

#include "gpio_signal.h"
#include "registers.h"

/* ADC signal */
enum adc_channel {
	ADC_VBUS,
	ADC_AMON_BMON,
	ADC_PSYS,
	/* Number of ADC channels */
	ADC_CH_COUNT
};

/* power signal definitions */
enum power_signal {
	X86_RSMRST_L_PWRGD = 0,
	X86_SLP_S3_DEASSERTED,
	X86_SLP_S4_DEASSERTED,
	X86_SLP_SUS_DEASSERTED,
	X86_PMIC_DPWROK,

	/* Number of X86 signals */
	POWER_SIGNAL_COUNT
};

/*
 * Motion sensors:
 * When reading through IO memory is set up, the first 2 entries must be
 * accelerometers, then gyroscope.
 * For BMI160, accel and gyro sensors must be next to each other.
 */
enum sensor_id {
	LID_ACCEL = 0,
	BASE_ACCEL,
	BASE_GYRO,
        LID_ALS,
};

/*
 * For backward compatibility, to report ALS via ACPI,
 * Define the number of ALS sensors: motion_sensor copy the data to the ALS
 * memmap region.
 */
#define CONFIG_ALS
#define ALS_COUNT 1

enum pwm_channel {
        PWM_CH_KBLIGHT,
        /* Number of PWM channels */
        PWM_CH_COUNT,
};

enum temp_sensor_id {
	TEMP_SENSOR_BATTERY,

	/* These temp sensors are only readable in S0 */
	TEMP_SENSOR_AMBIENT,
	TEMP_SENSOR_CHARGER,
	TEMP_SENSOR_DRAM,
	TEMP_SENSOR_WIFI,

	TEMP_SENSOR_COUNT
};

/* start as a sink in case we have no other power supply/battery */
#define PD_DEFAULT_STATE PD_STATE_SNK_DISCONNECTED

/* TODO: determine the following board specific type-C power constants */
/*
 * delay to turn on the power supply max is ~16ms.
 * delay to turn off the power supply max is about ~180ms.
 */
#define PD_POWER_SUPPLY_TURN_ON_DELAY  30000  /* us */
#define PD_POWER_SUPPLY_TURN_OFF_DELAY 250000 /* us */

/* delay to turn on/off vconn */
#define PD_VCONN_SWAP_DELAY 5000 /* us */

/* Define typical operating power and max power */
#define PD_OPERATING_POWER_MW 15000
#define PD_MAX_POWER_MW       45000
#define PD_MAX_CURRENT_MA     3000
#define PD_MAX_VOLTAGE_MV     20000

/* Reset PD MCU */
void board_reset_pd_mcu(void);

int board_get_version(void);

/* Reset RTC */
void board_rtc_reset(void);

/* Sensors without hardware FIFO are in forced mode */
#define CONFIG_ACCEL_FORCE_MODE_MASK ((1 << LID_ACCEL) || (1 << LID_ALS))

#endif /* !__ASSEMBLER__ */

#endif /* __CROS_EC_BOARD_H */
