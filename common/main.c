/* Copyright (c) 2013 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 *
 * Main routine for Chrome EC
 */

#include "board_config.h"
#include "button.h"
#include "clock.h"
#include "common.h"
#include "console.h"
#include "cpu.h"
#include "dma.h"
#include "eeprom.h"
#include "flash.h"
#include "gpio.h"
#include "hooks.h"
#include "lpc.h"
#include "keyboard_scan.h"
#ifdef CONFIG_MPU
#include "mpu.h"
#endif
#include "rwsig.h"
#include "system.h"
#include "task.h"
#include "timer.h"
#include "uart.h"
#include "util.h"
#include "watchdog.h"

/* Console output macros */
#define CPUTS(outstr) cputs(CC_SYSTEM, outstr)
#define CPRINTF(format, args...) cprintf(CC_SYSTEM, format, ## args)
#define CPRINTS(format, args...) cprints(CC_SYSTEM, format, ## args)

test_mockable __keep int main(void)
{
#ifdef CONFIG_REPLACE_LOADER_WITH_BSS_SLOW
	/*
	 * Now that we have started execution, we no longer need the loader.
	 * Instead, variables placed in the .bss.slow section will use this
	 * space.  Therefore, clear out this region now.
	 */
	memset((void *)(CONFIG_PROGRAM_MEMORY_BASE + CONFIG_LOADER_MEM_OFF), 0,
	       CONFIG_LOADER_SIZE);
#endif /* defined(CONFIG_REPLACE_LOADER_WITH_BSS_SLOW) */
	/*
	 * Pre-initialization (pre-verified boot) stage.  Initialization at
	 * this level should do as little as possible, because verified boot
	 * may need to jump to another image, which will repeat this
	 * initialization.  In particular, modules should NOT enable
	 * interrupts.
	 */
#ifdef CONFIG_BOARD_PRE_INIT
	board_config_pre_init();
#endif

#ifdef CONFIG_CHIP_PRE_INIT
	chip_pre_init();
#endif

#ifdef CONFIG_MPU
	mpu_pre_init();
#endif

	gpio_pre_init();

#ifdef CONFIG_BOARD_POST_GPIO_INIT
	board_config_post_gpio_init();
#endif
	/*
	 * Initialize interrupts, but don't enable any of them.  Note that
	 * task scheduling is not enabled until task_start() below.
	 */
	task_pre_init();

	/*
	 * Initialize the system module.  This enables the hibernate clock
	 * source we need to calibrate the internal oscillator.
	 */
	system_pre_init();
	system_common_pre_init();

#if defined(CONFIG_FLASH_PHYSICAL)
	/*
	 * Initialize flash and apply write protect if necessary.  Requires
	 * the reset flags calculated by system initialization.
	 */
	flash_pre_init();
#endif

	/* Set the CPU clocks / PLLs.  System is now running at full speed. */
	clock_init();

	/*
	 * Initialize timer.  Everything after this can be benchmarked.
	 * get_time() and udelay() may now be used.  usleep() requires task
	 * scheduling, so cannot be used yet.  Note that interrupts declared
	 * via DECLARE_IRQ() call timer routines when profiling is enabled, so
	 * timer init() must be before uart_init().
	 */
	timer_init();

	/* Main initialization stage.  Modules may enable interrupts here. */
	cpu_init();

#ifdef CONFIG_DMA
	/* Initialize DMA.  Must be before UART. */
	dma_init();
#endif

	/* Initialize UART.  Console output functions may now be used. */
	uart_init();

	/* be less verbose if we boot for USB resume to meet spec timings */
	if (!(system_get_reset_flags() & RESET_FLAG_USB_RESUME)) {
		if (system_jumped_to_this_image()) {
			CPRINTS("UART initialized after sysjump");
		} else {
			CPUTS("\n\n--- UART initialized after reboot ---\n");
			CPUTS("[Reset cause: ");
			system_print_reset_flags();
			CPUTS("]\n");
		}
		CPRINTF("[Image: %s, %s]\n",
			 system_get_image_copy_string(),
			 system_get_build_info());
	}

#ifdef CONFIG_BRINGUP
	ccprintf("\n\nWARNING: BRINGUP BUILD\n\n\n");
#endif

#ifdef CONFIG_WATCHDOG
	/*
	 * Initialize watchdog timer.  All lengthy operations between now and
	 * task_start() must periodically call watchdog_reload() to avoid
	 * triggering a watchdog reboot.  (This pretty much applies only to
	 * verified boot, because all *other* lengthy operations should be done
	 * by tasks.)
	 */
	watchdog_init();
#endif

	/*
	 * Verified boot needs to read the initial keyboard state and EEPROM
	 * contents.  EEPROM must be up first, so keyboard_scan can toggle
	 * debugging settings via keys held at boot.
	 */
#ifdef CONFIG_EEPROM
	eeprom_init();
#endif

	/*
	 * Keyboard scan init/Button init can set recovery events to
	 * indicate to host entry into recovery mode. Before this is
	 * done, lpc always report mask needs to be initialized
	 * correctly.
	 */
#ifdef CONFIG_LPC
	lpc_init_mask();
#endif
#ifdef HAS_TASK_KEYSCAN
	keyboard_scan_init();
#endif
#ifdef CONFIG_BUTTON_COUNT
	button_init();
#endif

#if !defined(CONFIG_VBOOT_EFS) && \
	defined(CONFIG_RWSIG) && !defined(HAS_TASK_RWSIG)
	/*
	 * Check the RW firmware signature and jump to it if it is good.
	 *
	 * Only the Read-Only firmware needs to do the signature check.
	 */
	if (system_get_image_copy() == SYSTEM_IMAGE_RO) {
#if defined(CONFIG_RWSIG_DONT_CHECK_ON_PIN_RESET)
		/*
		 * If system was reset by reset-pin, do not jump and wait for
		 * command from host
		 */
		if (system_get_reset_flags() == RESET_FLAG_RESET_PIN) {
			CPRINTS("Hard pin-reset detected, disable RW jump");
		} else
#endif
		{
			if (rwsig_check_signature())
				rwsig_jump_now();
		}
	}
#endif  /* !CONFIG_VBOOT_EFS && CONFIG_RWSIG && !HAS_TASK_RWSIG */

	/*
	 * Print the init time.  Not completely accurate because it can't take
	 * into account the time before timer_init(), but it'll at least catch
	 * the majority of the time.
	 */
	CPRINTS("Inits done");

	/* Launch task scheduling (never returns) */
	return task_start();
}
