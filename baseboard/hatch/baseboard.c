/* Copyright 2019 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */

/* Hatch family-specific configuration */
#include "atomic.h"
#include "battery_fuel_gauge.h"
#include "charge_manager.h"
#include "charge_state_v2.h"
#include "chipset.h"
#include "console.h"
#include "driver/bc12/pi3usb9201.h"
#include "driver/ppc/sn5s330.h"
#include "driver/tcpm/anx7447.h"
#include "driver/tcpm/ps8xxx.h"
#include "driver/tcpm/tcpci.h"
#include "driver/tcpm/tcpm.h"
#include "espi.h"
#include "gpio.h"
#include "hooks.h"
#include "i2c.h"
#include "keyboard_scan.h"
#include "power.h"
#include "tcpci.h"
#include "timer.h"
#include "usbc_ppc.h"
#include "util.h"

#define CPRINTS(format, args...) cprints(CC_SYSTEM, format, ## args)
#define CPRINTF(format, args...) cprintf(CC_SYSTEM, format, ## args)

#define CPRINTSUSB(format, args...) cprints(CC_USBCHARGE, format, ## args)
#define CPRINTFUSB(format, args...) cprintf(CC_USBCHARGE, format, ## args)

#define USB_PD_PORT_TCPC_0	0
#define USB_PD_PORT_TCPC_1	1

/******************************************************************************/
/* Wake up pins */
const enum gpio_signal hibernate_wake_pins[] = {
	GPIO_LID_OPEN,
	GPIO_ACOK_OD,
	GPIO_POWER_BUTTON_L,
	/* EC_RST_ODL needs to wake device while in PSL hibernate. */
	GPIO_SYS_RESET_L,
};
const int hibernate_wake_pins_used = ARRAY_SIZE(hibernate_wake_pins);

/******************************************************************************/
/* Keyboard scan setting */
struct keyboard_scan_config keyscan_config = {
	/*
	 * F3 key scan cycle completed but scan input is not
	 * charging to logic high when EC start scan next
	 * column for "T" key, so we set .output_settle_us
	 * to 80us from 50us.
	 */
	.output_settle_us = 80,
	.debounce_down_us = 9 * MSEC,
	.debounce_up_us = 30 * MSEC,
	.scan_period_us = 3 * MSEC,
	.min_post_scan_delay_us = 1000,
	.poll_timeout_us = 100 * MSEC,
	.actual_key_mask = {
		0x14, 0xff, 0xff, 0xff, 0xff, 0xf5, 0xff,
		0xa4, 0xff, 0xfe, 0x55, 0xfa, 0xca  /* full set */
	},
};

/******************************************************************************/
/* I2C port map configuration */
const struct i2c_port_t i2c_ports[] = {
	{"sensor",  I2C_PORT_SENSOR,  100, GPIO_I2C0_SCL, GPIO_I2C0_SDA},
	{"ppc0",    I2C_PORT_PPC0,    100, GPIO_I2C1_SCL, GPIO_I2C1_SDA},
	{"tcpc1",   I2C_PORT_TCPC1,   100, GPIO_I2C2_SCL, GPIO_I2C2_SDA},
	{"tcpc0",   I2C_PORT_TCPC0,   100, GPIO_I2C3_SCL, GPIO_I2C3_SDA},
	{"power",   I2C_PORT_POWER,   100, GPIO_I2C5_SCL, GPIO_I2C5_SDA},
	{"eeprom",  I2C_PORT_EEPROM,  100, GPIO_I2C7_SCL, GPIO_I2C7_SDA},
};
const unsigned int i2c_ports_used = ARRAY_SIZE(i2c_ports);

/* power signal list. */
const struct power_signal_info power_signal_list[] = {

	[X86_SLP_S0_DEASSERTED] = {GPIO_SLP_S0_L,
		POWER_SIGNAL_ACTIVE_HIGH | POWER_SIGNAL_DISABLE_AT_BOOT,
		"SLP_S0_DEASSERTED"},
#ifdef CONFIG_HOSTCMD_ESPI_VW_SIGNALS
	[X86_SLP_S3_DEASSERTED] = {VW_SLP_S3_L, POWER_SIGNAL_ACTIVE_HIGH,
				   "SLP_S3_DEASSERTED"},
	[X86_SLP_S4_DEASSERTED] = {VW_SLP_S4_L, POWER_SIGNAL_ACTIVE_HIGH,
				   "SLP_S4_DEASSERTED"},
#else
	[X86_SLP_S3_DEASSERTED] = {GPIO_SLP_S3_L, POWER_SIGNAL_ACTIVE_HIGH,
				   "SLP_S3_DEASSERTED"},
	[X86_SLP_S4_DEASSERTED] = {GPIO_SLP_S4_L, POWER_SIGNAL_ACTIVE_HIGH,
				   "SLP_S4_DEASSERTED"},
#endif
	[X86_RSMRST_L_PGOOD] = {GPIO_PG_EC_RSMRST_L, POWER_SIGNAL_ACTIVE_HIGH,
				"RSMRST_L_PGOOD"},
	[PP5000_A_PGOOD] = {GPIO_PP5000_A_PG_OD, POWER_SIGNAL_ACTIVE_HIGH,
			    "PP5000_A_PGOOD"},
	[ALL_SYS_PGOOD] = {GPIO_PG_EC_ALL_SYS_PWRGD, POWER_SIGNAL_ACTIVE_HIGH,
			   "ALL_SYS_PWRGD"}
};
BUILD_ASSERT(ARRAY_SIZE(power_signal_list) == POWER_SIGNAL_COUNT);

/******************************************************************************/
/* Chipset callbacks/hooks */

/* Called on AP S5 -> S3 transition */
static void baseboard_chipset_startup(void)
{
	/* TODD(b/122266850): Need to fill out this hook */
}
DECLARE_HOOK(HOOK_CHIPSET_STARTUP, baseboard_chipset_startup,
	     HOOK_PRIO_DEFAULT);

/* Called on AP S0iX -> S0 transition */
static void baseboard_chipset_resume(void)
{
	/* TODD(b/122266850): Need to fill out this hook */
	/* Enable keyboard backlight */
	gpio_set_level(GPIO_EC_KB_BL_EN, 1);
}
DECLARE_HOOK(HOOK_CHIPSET_RESUME, baseboard_chipset_resume, HOOK_PRIO_DEFAULT);

/* Called on AP S0 -> S0iX transition */
static void baseboard_chipset_suspend(void)
{
	/* TODD(b/122266850): Need to fill out this hook */
	/* Disable keyboard backlight */
	gpio_set_level(GPIO_EC_KB_BL_EN, 0);
}
DECLARE_HOOK(HOOK_CHIPSET_SUSPEND, baseboard_chipset_suspend,
	     HOOK_PRIO_DEFAULT);

/* Called on AP S3 -> S5 transition */
static void baseboard_chipset_shutdown(void)
{
	/* TODD(b/122266850): Need to fill out this hook */
}
DECLARE_HOOK(HOOK_CHIPSET_SHUTDOWN, baseboard_chipset_shutdown,
	     HOOK_PRIO_DEFAULT);

/******************************************************************************/
/* USB-C TPCP Configuration */
const struct tcpc_config_t tcpc_config[CONFIG_USB_PD_PORT_COUNT] = {
	[USB_PD_PORT_TCPC_0] = {
		.i2c_host_port = I2C_PORT_TCPC0,
		.i2c_slave_addr = AN7447_TCPC0_I2C_ADDR,
		.drv = &anx7447_tcpm_drv,
		/* Alert is active-low, push-pull */
		.flags = 0,
	},
	[USB_PD_PORT_TCPC_1] = {
		.i2c_host_port = I2C_PORT_TCPC1,
		.i2c_slave_addr = PS8751_I2C_ADDR1,
		.drv = &ps8xxx_tcpm_drv,
		/* Alert is active-low, push-pull */
		.flags = 0,
	},
};

/******************************************************************************/
/* USB-C PPC Configuration */
struct ppc_config_t ppc_chips[CONFIG_USB_PD_PORT_COUNT] = {
	[USB_PD_PORT_TCPC_0] = {
		.i2c_port = I2C_PORT_PPC0,
		.i2c_addr = SN5S330_ADDR0,
		.drv = &sn5s330_drv
	},

	[USB_PD_PORT_TCPC_1] = {
		.i2c_port = I2C_PORT_TCPC1,
		.i2c_addr = SN5S330_ADDR0,
		.drv = &sn5s330_drv
	},
};
unsigned int ppc_cnt = ARRAY_SIZE(ppc_chips);

struct usb_mux usb_muxes[CONFIG_USB_PD_PORT_COUNT] = {
	[USB_PD_PORT_TCPC_0] = {
		.driver = &anx7447_usb_mux_driver,
		.hpd_update = &anx7447_tcpc_update_hpd_status,
	},
	[USB_PD_PORT_TCPC_1] = {
		.driver = &tcpci_tcpm_usb_mux_driver,
		.hpd_update = &ps8xxx_tcpc_update_hpd_status,
	}
};

const struct pi3usb2901_config_t pi3usb2901_bc12_chips[] = {
	[USB_PD_PORT_TCPC_0] = {
		.i2c_port = I2C_PORT_PPC0,
		.i2c_addr = PI3USB9201_I2C_ADDR_3,
	},

	[USB_PD_PORT_TCPC_1] = {
		.i2c_port = I2C_PORT_TCPC1,
		.i2c_addr = PI3USB9201_I2C_ADDR_3,
	},
};

/* GPIO to enable/disable the USB Type-A port. */
const int usb_port_enable[CONFIG_USB_PORT_POWER_SMART_PORT_COUNT] = {
	GPIO_EN_USB_A_5V,
};

/* Power Delivery and charging functions */

void baseboard_tcpc_init(void)
{
	/* Enable PPC interrupts. */
	gpio_enable_interrupt(GPIO_USB_C0_PPC_INT_ODL);
	gpio_enable_interrupt(GPIO_USB_C1_PPC_INT_ODL);

	/* Enable TCPC interrupts. */
	gpio_enable_interrupt(GPIO_USB_C0_TCPC_INT_ODL);
	gpio_enable_interrupt(GPIO_USB_C1_TCPC_INT_ODL);

	/* Enable HDMI HPD interrupt. */
	gpio_enable_interrupt(GPIO_HDMI_CONN_HPD);

	/* Enable BC 1.2 interrupts */
	gpio_enable_interrupt(GPIO_USB_C0_BC12_INT_ODL);
	gpio_enable_interrupt(GPIO_USB_C1_BC12_INT_ODL);
}
DECLARE_HOOK(HOOK_INIT, baseboard_tcpc_init, HOOK_PRIO_INIT_I2C + 1);

uint16_t tcpc_get_alert_status(void)
{
	uint16_t status = 0;

	/*
	 * Check which port has the ALERT line set and ignore if that TCPC has
	 * its reset line active. Note that port 0 reset is active high and
	 * port 1 reset is active low.
	 */
	if (!gpio_get_level(GPIO_USB_C0_TCPC_INT_ODL)) {
		if (!gpio_get_level(GPIO_USB_C0_TCPC_RST))
			status |= PD_STATUS_TCPC_ALERT_0;
	}

	if (!gpio_get_level(GPIO_USB_C1_TCPC_INT_ODL)) {
		if (gpio_get_level(GPIO_USB_C1_TCPC_RST_ODL))
			status |= PD_STATUS_TCPC_ALERT_1;
	}

	return status;
}

void board_reset_pd_mcu(void)
{
	/*
	 * C0: Assert reset to TCPC0 (ANX7447) for required delay (1ms) only if
	 * we have a battery
	 *
	 */
	if (battery_is_present() == BP_YES) {
		gpio_set_level(GPIO_USB_C0_TCPC_RST, 1);
		msleep(ANX74XX_RESET_HOLD_MS);
		gpio_set_level(GPIO_USB_C0_TCPC_RST, 0);
		msleep(ANX74XX_RESET_FINISH_MS);
	}
	/*
	 * C1: Assert reset to TCPC1 (PS8751) for required delay (1ms) only if
	 * we have a battery, otherwise we may brown out the system.
	 */
	if (battery_is_present() == BP_YES) {
		/*
		 * TODO(crbug:846412): After refactor, ensure that battery has
		 * enough charge to last the reboot as well
		 */
		gpio_set_level(GPIO_USB_C1_TCPC_RST_ODL, 0);
		msleep(PS8XXX_RESET_DELAY_MS);
		gpio_set_level(GPIO_USB_C1_TCPC_RST_ODL, 1);
	} else {
		CPRINTS("Skipping C1 TCPC reset because no battery");
	}
}

void board_pd_vconn_ctrl(int port, int cc_pin, int enabled)
{
	/*
	 * We ignore the cc_pin because the polarity should already be set
	 * correctly in the PPC driver via the pd state machine.
	 */
	if (ppc_set_vconn(port, enabled) != EC_SUCCESS)
		cprints(CC_USBPD, "C%d: Failed %sabling vconn",
			port, enabled ? "en" : "dis");
}

int board_set_active_charge_port(int port)
{
	int is_valid_port = (port >= 0 &&
			    port < CONFIG_USB_PD_PORT_COUNT);
	int i;

	if (!is_valid_port && port != CHARGE_PORT_NONE)
		return EC_ERROR_INVAL;

	if (port == CHARGE_PORT_NONE) {
		CPRINTSUSB("Disabling all charger ports");

		/* Disable all ports. */
		for (i = 0; i < ppc_cnt; i++) {
			/*
			 * Do not return early if one fails otherwise we can
			 * get into a boot loop assertion failure.
			 */
			if (ppc_vbus_sink_enable(i, 0))
				CPRINTSUSB("Disabling C%d as sink failed.", i);
		}

		return EC_SUCCESS;
	}

	/* Check if the port is sourcing VBUS. */
	if (ppc_is_sourcing_vbus(port)) {
		CPRINTFUSB("Skip enable C%d", port);
		return EC_ERROR_INVAL;
	}

	CPRINTSUSB("New charge port: C%d", port);

	/*
	 * Turn off the other ports' sink path FETs, before enabling the
	 * requested charge port.
	 */
	for (i = 0; i < ppc_cnt; i++) {
		if (i == port)
			continue;

		if (ppc_vbus_sink_enable(i, 0))
			CPRINTSUSB("C%d: sink path disable failed.", i);
	}

	/* Enable requested charge port. */
	if (ppc_vbus_sink_enable(port, 1)) {
		CPRINTSUSB("C%d: sink path enable failed.", port);
		return EC_ERROR_UNKNOWN;
	}

	return EC_SUCCESS;
}

void board_set_charge_limit(int port, int supplier, int charge_ma,
			    int max_ma, int charge_mv)
{
	charge_set_input_current_limit(MAX(charge_ma,
					   CONFIG_CHARGER_INPUT_CURRENT),
				       charge_mv);
}

void baseboard_mst_enable_control(enum mst_source src, int level)
{
	static uint32_t mst_input_levels;

	if (level)
		atomic_or(&mst_input_levels, 1 << src);
	else
		atomic_clear(&mst_input_levels, 1 << src);

	gpio_set_level(GPIO_EN_MST, mst_input_levels ? 1 : 0);
}

/* Enable or disable input devices, based on chipset state */
#ifndef TEST_BUILD
void lid_angle_peripheral_enable(int enable)
{
	/* TODO(b/125936966): Need to add SKU dependency for convertibles */
	if (chipset_in_state(CHIPSET_STATE_ANY_OFF))
		enable = 0;
	keyboard_scan_enable(enable, KB_SCAN_DISABLE_LID_ANGLE);
}
#endif
