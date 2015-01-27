/*
 * arch/arm/mach-sun7i/board-mod-duo.c
 *
 * (c)2015 Musical Operating Devices <http://www.portalmod.com/>
 *
 * Authors:
 * 	Felipe Correa da Silva Sanches <juca@members.fsf.org>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2 (or later), as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/platform_device.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>

static struct platform_device mod_duo_audio_device = {
	.name	= "mod-duo-audio",
	.id	= -1,
};

static struct platform_device *mod_duo_devices[] __initdata = {
	&mod_duo_audio_device,
};

static void __init mod_duo_init(void)
{
	platform_add_devices(mod_duo_devices, ARRAY_SIZE(mod_duo_devices));
}

MACHINE_START(MODDUO, "MOD Duo")
	.init_machine   = &mod_duo_init,
MACHINE_END
