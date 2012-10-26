/*
 *  Copyright (C) 2011 Xilinx
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/of_platform.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/hardware.h>
#include "common.h"

#include <linux/i2c/adv7511.h>


#if defined(CONFIG_ADV7511)

/* Initial ADV7511 DVI output format is set to RGB */
static struct adv7511_platform_data adv7511_0 = {
	.format = "RGB",
};

static struct i2c_board_info __initdata adv7511_board_info[] = {
	{
		I2C_BOARD_INFO("adv7511", 0x39),
		.platform_data = &adv7511_0,
	}
};

#endif /* CONFIG_ADV7511 */

extern struct sys_timer xttcpss_sys_timer;

static void __init board_zc770_init(void)
{

	/* initialize the xilinx common code before the board
	 * specific
	 */
	xilinx_init_machine();

#if defined(CONFIG_ADV7511)
	i2c_register_board_info(0, adv7511_board_info,
				ARRAY_SIZE(adv7511_board_info));
#endif
}

static const char *xilinx_dt_match[] = {
	"xlnx,zynq-zc770",
	NULL
};

MACHINE_START(XILINX_EP107, "Xilinx Zynq Platform")
	.map_io		= xilinx_map_io,
	.init_irq	= xilinx_irq_init,
	.init_machine	= board_zc770_init,
	.timer		= &xttcpss_sys_timer,
	.dt_compat	= xilinx_dt_match,
	.reserve	= xilinx_memory_init,
MACHINE_END
