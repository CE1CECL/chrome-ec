/* Copyright 2018 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */

/* NX20P348x USB-C Power Path Controller */

#include "common.h"
#include "console.h"
#include "driver/ppc/nx20p348x.h"
#include "gpio.h"
#include "hooks.h"
#include "i2c.h"
#include "system.h"
#include "tcpm.h"
#include "usb_charge.h"
#include "usb_pd_tcpm.h"
#include "usbc_ppc.h"
#include "util.h"

#define CPRINTF(format, args...) cprintf(CC_USBPD, format, ## args)
#define CPRINTS(format, args...) cprints(CC_USBPD, format, ## args)

static uint32_t irq_pending; /* Bitmask of ports signaling an interrupt. */

#define NX20P348X_DB_EXIT_FAIL_THRESHOLD 10
static int db_exit_fail_count[CONFIG_USB_PD_PORT_COUNT];

#define NX20P348X_FLAGS_SOURCE_ENABLED (1 << 0)
static uint8_t flags[CONFIG_USB_PD_PORT_COUNT];

static int read_reg(uint8_t port, int reg, int *regval)
{
	return i2c_read8__7bf(ppc_chips[port].i2c_port,
			 ppc_chips[port].i2c_addr__7bf,
			 reg,
			 regval);
}

static int write_reg(uint8_t port, int reg, int regval)
{
	return i2c_write8__7bf(ppc_chips[port].i2c_port,
			  ppc_chips[port].i2c_addr__7bf,
			  reg,
			  regval);
}

static int nx20p348x_set_ovp_limit(int port)
{
	int rv;
	int reg;

	/* Set VBUS over voltage threshold (OVLO) */
	rv = read_reg(port, NX20P348X_OVLO_THRESHOLD_REG, &reg);
	if (rv)
		return rv;
	/* OVLO threshold is 3 bit field */
	reg &= ~NX20P348X_OVLO_THRESHOLD_MASK;
	/* Set SNK OVP to 23.0 V */
	reg |= NX20P348X_OVLO_23_0;
	rv = write_reg(port, NX20P348X_OVLO_THRESHOLD_REG, reg);
	if (rv)
		return rv;

	return EC_SUCCESS;
}

static int nx20p348x_is_sourcing_vbus(int port)
{
	return flags[port] & NX20P348X_FLAGS_SOURCE_ENABLED;
}

static int nx20p348x_set_vbus_source_current_limit(int port,
						 enum tcpc_rp_value rp)
{
	int regval;
	int status;

	status = read_reg(port, NX20P348X_5V_SRC_OCP_THRESHOLD_REG, &regval);
	if (status)
		return status;

	regval &= ~NX20P348X_ILIM_MASK;

	/* We need buffer room for all current values. */
	switch (rp) {
	case TYPEC_RP_3A0:
		regval |= NX20P348X_ILIM_3_200;
		break;

	case TYPEC_RP_1A5:
		regval |= NX20P348X_ILIM_1_600;
		break;

	case TYPEC_RP_USB:
	default:
		regval |= NX20P348X_ILIM_0_600;
		break;
	};


	return write_reg(port, NX20P348X_5V_SRC_OCP_THRESHOLD_REG, regval);
}

static int nx20p348x_discharge_vbus(int port, int enable)
{
	int regval;
	int status;

	status = read_reg(port, NX20P348X_DEVICE_CONTROL_REG, &regval);
	if (status)
		return status;

	if (enable)
		regval |= NX20P348X_CTRL_VBUSDIS_EN;
	else
		regval &= ~NX20P348X_CTRL_VBUSDIS_EN;

	status = write_reg(port, NX20P348X_DEVICE_CONTROL_REG, regval);
	if (status) {
		CPRINTS("Failed to %s vbus discharge",
			enable ? "enable" : "disable");
		return status;
	}

	return EC_SUCCESS;
}

static int nx20p348x_vbus_sink_enable(int port, int enable)
{
	int status;
	int rv;
	int desired_mode = enable ? NX20P348X_MODE_HV_SNK :
		NX20P348X_MODE_STANDBY;

	enable = !!enable;

	/*
	 * We cannot use an EC GPIO for EN_SNK since an EC reset will float the
	 * GPIO thus browning out the board (without a battery).
	 */
	rv = tcpm_set_snk_ctrl(port, enable);
	if (rv)
		return rv;

	/* Read device status register to get mode */
	rv = read_reg(port, NX20P348X_DEVICE_STATUS_REG, &status);
	if (rv)
		return rv;

	return ((status & NX20P348X_DEVICE_MODE_MASK) == desired_mode) ?
		EC_SUCCESS : EC_ERROR_UNKNOWN;
}

static int nx20p348x_vbus_source_enable(int port, int enable)
{
	int status;
	int rv;
	int desired_mode = enable ? NX20P348X_MODE_5V_SRC :
		NX20P348X_MODE_STANDBY;

	enable = !!enable;

	/*
	 * For parity's sake, we should not use an EC GPIO for EN_SRC since we
	 * cannot use it for EN_SNK (for brown out reason listed above).
	 */
	rv = tcpm_set_src_ctrl(port, enable);
	if (rv)
		return rv;

	/* Read device status register to get mode */
	rv = read_reg(port, NX20P348X_DEVICE_STATUS_REG, &status);
	if (rv)
		return rv;

	if ((status & NX20P348X_DEVICE_MODE_MASK) != desired_mode)
		return EC_ERROR_UNKNOWN;

	/* Cache the Vbus state */
	if (enable)
		flags[port] |= NX20P348X_FLAGS_SOURCE_ENABLED;
	else
		flags[port] &= ~NX20P348X_FLAGS_SOURCE_ENABLED;

	return EC_SUCCESS;
}

static int nx20p348x_init(int port)
{
	int reg;
	int mask;
	int mode;
	int rv;

	/* Mask interrupts for interrupt 2 register */
	mask = ~NX20P348X_INT2_EN_ERR;
	rv = write_reg(port, NX20P348X_INTERRUPT2_MASK_REG, mask);
	if (rv)
		return rv;

	/* Mask interrupts for interrupt 1 register */
	mask = ~(NX20P348X_INT1_OC_5VSRC | NX20P348X_INT1_DBEXIT_ERR);
	rv = write_reg(port, NX20P348X_INTERRUPT1_MASK_REG, mask);
	if (rv)
		return rv;

	/* Clear any pending interrupts by reading interrupt registers */
	read_reg(port, NX20P348X_INTERRUPT1_REG, &reg);
	read_reg(port, NX20P348X_INTERRUPT2_REG, &reg);

	/* Get device  mode */
	rv = read_reg(port, NX20P348X_DEVICE_STATUS_REG, &mode);
	if (rv)
		return rv;
	mode &= NX20P348X_DEVICE_MODE_MASK;

	/* Check if dead battery mode is active. */
	if (mode == NX20P348X_MODE_DEAD_BATTERY) {
		/*
		 * If in dead battery mode, must enable HV SNK mode prior to
		 * exiting dead battery mode or VBUS path will get cut off and
		 * system will lose power. Before DB mode is exited, the device
		 * mode will not reflect the correct value and therefore the
		 * return value isn't useful here.
		 */
		nx20p348x_vbus_sink_enable(port, 1);

		/* Exit dead battery mode. */
		rv = read_reg(port, NX20P348X_DEVICE_CONTROL_REG, &reg);
		if (rv)
			return rv;
		reg |= NX20P348X_CTRL_DB_EXIT;
		rv = write_reg(port, NX20P348X_DEVICE_CONTROL_REG, reg);
		if (rv)
			return rv;
	}

	/*
	 * Set VBUS over voltage threshold (OVLO). Note that while the PPC is in
	 * dead battery mode, OVLO is forced to 6.8V, so this setting must be
	 * done after dead battery mode is exited.
	 */
	nx20p348x_set_ovp_limit(port);

	return EC_SUCCESS;
}

static void nx20p348x_handle_interrupt(int port)
{
	int reg;
	int control_reg;

	/*
	 * Read interrupt 1 status register. Note, interrupt register is
	 * automatically cleared by reading.
	 */
	read_reg(port, NX20P348X_INTERRUPT1_REG, &reg);

	/* Check for DBEXIT error */
	if (reg & NX20P348X_INT1_DBEXIT_ERR) {
		int mask_reg;

		/*
		 * This failure is not expected. If for some reason, this
		 * keeps happening, then log an error and mask the interrupt to
		 * prevent interrupt floods.
		 */
		if (++db_exit_fail_count[port] >=
		    NX20P348X_DB_EXIT_FAIL_THRESHOLD) {
			CPRINTS("Port %d PPC failed to exit DB mode", port);
			if (read_reg(port, NX20P348X_INTERRUPT1_MASK_REG,
				    &mask_reg)) {
				mask_reg |= NX20P348X_INT1_DBEXIT_ERR;
				write_reg(port, NX20P348X_INTERRUPT1_MASK_REG,
					  mask_reg);
			}
		}
		read_reg(port, NX20P348X_DEVICE_CONTROL_REG, &control_reg);
		reg |= NX20P348X_CTRL_DB_EXIT;
		write_reg(port, NX20P348X_DEVICE_CONTROL_REG, control_reg);
		/*
		 * If DB exit mode failed, then the OVP limit setting done in
		 * the init routine will not be successful. Set the OVP limit
		 * again here.
		 */
		nx20p348x_set_ovp_limit(port);
	}

	/* Check for 5V OC interrupt */
	if (reg & NX20P348X_INT1_OC_5VSRC) {
		CPRINTS("C%d: PPC detected overcurrent!", port);
		/*
		 * TODO (b/69935262): The overcurrent action hasn't
		 * been completed yet, but is required for TI PPC. When that
		 * work is complete, tie it in here.
		 */
	}

	/*
	 * Read interrupt 2 status register. Note, interrupt register is
	 * automatically cleared by reading.
	 */
	/*
	 * TODO (b/75272421): Not sure if any of these interrupts
	 * will be used. Might want to use EN_ERR which tracks when both
	 * SNK_EN and SRC_EN are set. However, since for the Analogix TCPC
	 * these values aren't controlled by the EC directly, not sure what
	 * action if any can be taken.
	 */
	read_reg(port, NX20P348X_INTERRUPT2_REG, &reg);
}

static void nx20p348x_irq_deferred(void)
{
	int i;
	uint32_t pending = atomic_read_clear(&irq_pending);

	for (i = 0; i < CONFIG_USB_PD_PORT_COUNT; i++)
		if ((1 << i) & pending)
			nx20p348x_handle_interrupt(i);
}
DECLARE_DEFERRED(nx20p348x_irq_deferred);

void nx20p348x_interrupt(int port)
{
	atomic_or(&irq_pending, (1 << port));
	hook_call_deferred(&nx20p348x_irq_deferred_data, 0);
}

#ifdef CONFIG_CMD_PPC_DUMP
static int nx20p348x_dump(int port)
{
	int reg_addr;
	int reg;
	int rv;

	ccprintf("Port %d NX20P348X registers\n", port);
	for (reg_addr = NX20P348X_DEVICE_ID_REG; reg_addr <=
		     NX20P348X_DEVICE_CONTROL_REG; reg_addr++) {
		rv = read_reg(port, reg_addr, &reg);
		if (rv) {
			ccprintf("nx20p: Failed to read register 0x%x\n",
				 reg_addr);
			return rv;
		}
		ccprintf("[0x%02x]: 0x%02x\n", reg_addr, reg);

		/* Flush every call otherwise buffer may get full */
		cflush();
	}

	return EC_SUCCESS;
}
#endif /* defined(CONFIG_CMD_PPC_DUMP) */

const struct ppc_drv nx20p348x_drv = {
	.init = &nx20p348x_init,
	.is_sourcing_vbus = &nx20p348x_is_sourcing_vbus,
	.vbus_sink_enable = &nx20p348x_vbus_sink_enable,
	.vbus_source_enable = &nx20p348x_vbus_source_enable,
#ifdef CONFIG_CMD_PPC_DUMP
	.reg_dump = &nx20p348x_dump,
#endif /* defined(CONFIG_CMD_PPC_DUMP) */
	.set_vbus_source_current_limit =
		&nx20p348x_set_vbus_source_current_limit,
	.discharge_vbus = &nx20p348x_discharge_vbus,
};
