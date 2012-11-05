/*
 * adv7511.h - Configuration for adv7511 misc driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation (version 2 of the License only).
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef __LINUX_ADV7511_H
#define __LINUX_ADV7511_H

#include <linux/types.h>
#include <linux/device.h>
#include <linux/i2c.h>

#define ADV7511_UNDEFINED 0
#define ADV7511_RGB       1
#define ADV7511_YCbCr     2

#define FORMAT_NAME_SZ (5+1)

struct adv7511_platform_data {
	char format[FORMAT_NAME_SZ];
};

struct i2c_client *adv7511_get_client(void);
int adv7511_get_format(struct device *dev);
int adv7511_set_format(struct device *dev, int type);

#endif /* __LINUX_ADV7511_H */
