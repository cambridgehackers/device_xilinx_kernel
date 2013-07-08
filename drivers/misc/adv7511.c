/*
 * Simple I2C driver for initalizing ADV7511 HDMI transmitter
 *
 * Author: Xylon d.o.o.
 * e-mail: davor.joja@logicbricks.com
 *
 * 2012 (c) Xylon d.o.o.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */


/*
 * This file implements I2C functionality for controlling ADV7511 confguration
 * used with logiCVC.
 */


#include <linux/module.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/of_i2c.h>
#include <linux/i2c/adv7511.h>


#define FORMAT_RGB "RGB"
#define FORMAT_YCbCr "YCbCr"
#define FORMAT_DISCONNECTED "disconnected"

struct adv7511_config_data {
	u8 address;
	u8 value;
};

struct adv7511_data {
	struct attribute_group attrs;
	struct mutex lock;
	const struct adv7511_config_data *adv7511_cfg_data;
	size_t adv7511_cfg_size;
	char format[FORMAT_NAME_SZ];
};


static const struct adv7511_config_data adv7511_rgb_config[] = {
	{0x41, 0x10},
	{0xD6, 0xC0},
	{0x15, 0x01},
	{0x16, 0x38},
	{0x18, 0xAB},
	{0x19, 0x37},
	{0x1A, 0x08},
	{0x1B, 0x00},
	{0x1C, 0x00},
	{0x1D, 0x00},
	{0x1E, 0x1A},
	{0x1F, 0x86},
	{0x20, 0x1A},
	{0x21, 0x49},
	{0x22, 0x08},
	{0x23, 0x00},
	{0x24, 0x1D},
	{0x25, 0x3F},
	{0x26, 0x04},
	{0x27, 0x22},
	{0x28, 0x00},
	{0x29, 0x00},
	{0x2A, 0x08},
	{0x2B, 0x00},
	{0x2C, 0x0E},
	{0x2D, 0x2D},
	{0x2E, 0x19},
	{0x2F, 0x14},
	{0x48, 0x08},
	{0x55, 0x00},
	{0x56, 0x28},
	{0x98, 0x03},
	{0x9A, 0xE0},
	{0x9C, 0x30},
	{0x9D, 0x61},
	{0xA2, 0xA4},
	{0xA3, 0xA4},
	{0xAF, 0x04},
	{0xE0, 0xD0},
	{0xF9, 0x00}
};

static const struct adv7511_config_data adv7511_ycbcr_config[] = {
	{0x41, 0x10},
	{0xD6, 0xC0},
	{0x15, 0x01},
	{0x16, 0xB9},
	{0x48, 0x08},
	{0x55, 0x20},
	{0x56, 0x28},
	{0x98, 0x03},
	{0x9A, 0xE0},
	{0x9C, 0x30},
	{0x9D, 0x61},
	{0xA2, 0xA4},
	{0xA3, 0xA4},
	{0xAF, 0x06},
	{0xE0, 0xD0},
	{0xF9, 0x00} 
};

static struct i2c_client *adv7511_client;


static int adv7511_configure(struct i2c_client *client)
{
	struct adv7511_data *data = i2c_get_clientdata(client);
	int i, ret;

	for (i = 0; i < data->adv7511_cfg_size; i++) {
		ret = i2c_smbus_write_byte_data(client,
			data->adv7511_cfg_data[i].address, data->adv7511_cfg_data[i].value);
		if (ret)
			break;
	}

	if (ret) {
		dev_warn(&client->dev,
			"failed configuring %s for %s\n",
				client->driver->id_table->name, data->format);
	} else {
		dev_info(&client->dev,
			"%s configured for DVI (%s)\n",
				client->driver->id_table->name, data->format);
	}

	return 0;
}

struct i2c_client *adv7511_get_client(void)
{
	return adv7511_client;
}
EXPORT_SYMBOL(adv7511_get_client);

static ssize_t show_format_attr(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct adv7511_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%s\n", data->format);
}

int adv7511_get_format(struct device *dev)
{
	int ret;
	char buf[FORMAT_NAME_SZ];

	if ((!dev) || (to_i2c_client(dev) != adv7511_client))
		return -EINVAL;

	show_format_attr(dev, NULL, buf);

	ret = strncmp(FORMAT_RGB, buf, (sizeof(FORMAT_RGB)-1));
	if (!ret)
		return ADV7511_RGB;
	ret = strncmp(FORMAT_YCbCr, buf, (sizeof(FORMAT_YCbCr)-1));
	if (!ret)
		return ADV7511_YCbCr;

	return 0;
}
EXPORT_SYMBOL(adv7511_get_format);

static ssize_t set_format_attr(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct adv7511_data *data = i2c_get_clientdata(client);
	char *c;
	int ret;

	mutex_lock(&data->lock);

	ret = 0;
	/* remove "new line" character */
	strcpy(data->format, buf);
	c = strchr(data->format, 0x0A);
	if (c)
		*c = 0;

	if (!strcmp(data->format, FORMAT_RGB)) {
		data->adv7511_cfg_data = adv7511_rgb_config;
		data->adv7511_cfg_size = ARRAY_SIZE(adv7511_rgb_config);
	} else if (!strcmp(data->format, FORMAT_YCbCr)) {
		data->adv7511_cfg_data = adv7511_ycbcr_config;
		data->adv7511_cfg_size = ARRAY_SIZE(adv7511_ycbcr_config);
	} else if (!strcmp(data->format, FORMAT_DISCONNECTED)) {
		data->adv7511_cfg_data = 0;
		data->adv7511_cfg_size = 0;
	} else {
		ret = -EINVAL;
		pr_err("%s string error!\n", client->driver->id_table->name);
	}

	if (!ret)
		ret = adv7511_configure(client);

	data->adv7511_cfg_data = NULL;
	data->adv7511_cfg_size = 0;

	mutex_unlock(&data->lock);

	if (!ret)
		ret = count;

	return ret;
}

int adv7511_set_format(struct device *dev, int type)
{
	int ret;

	if ((!dev) || (to_i2c_client(dev) != adv7511_client))
		return -EINVAL;

	ret = ADV7511_UNDEFINED;

	if (ADV7511_RGB == type)
		ret = set_format_attr(dev, NULL, FORMAT_RGB, 0);
	else if (ADV7511_YCbCr == type)
		ret = set_format_attr(dev, NULL, FORMAT_YCbCr, 0);

	return ret;
}
EXPORT_SYMBOL(adv7511_set_format);

static DEVICE_ATTR(format, (S_IWUSR | S_IRUGO),
	show_format_attr, set_format_attr);

static struct attribute *adv7511_attr[] = {
	&dev_attr_format.attr,
	NULL
};

static const struct i2c_device_id adv7511_id[] = {
	{ "adv7511", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, adv7511_id);

static int adv7511_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct adv7511_platform_data *pdata = client->dev.platform_data;
	struct adv7511_data *data;
	u32 const *prop;
	char *cptr;
	int ret, size;
	bool platform, of;

	if (client->dev.of_node) {
		of = true;
	} else {
		of = false;
		if (pdata)
			platform = true;
		else
			return -ENODEV;
	}

	data = kzalloc(sizeof(struct adv7511_data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto exit;
	}

	i2c_set_clientdata(client, data);

	mutex_init(&data->lock);

	/* Register sysfs hooks */
	data->attrs.attrs = adv7511_attr;
	ret = sysfs_create_group(&client->dev.kobj, &data->attrs);
	if (ret)
		goto exit_free;

	cptr = NULL;
	if (of) {
		prop = of_get_property(client->dev.of_node, "format", &size);
		if (prop)
			cptr = (char *)prop;
	} else if (platform) {
		cptr = pdata->format;
	}

	if (!cptr)
		goto exit_free;

	ret = set_format_attr(&client->dev, NULL, cptr, 0);
	if (ret)
		goto exit_free;

	data->adv7511_cfg_data = NULL;
	data->adv7511_cfg_size = 0;

	adv7511_client = client;

	return 0;

exit_free:
	kfree(data);
exit:
	return ret;
}

static int adv7511_remove(struct i2c_client *client)
{
	struct adv7511_data *data = i2c_get_clientdata(client);

	sysfs_remove_group(&client->dev.kobj, &data->attrs);

	kfree(data);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id i2c_adv7511_of_match[] = {
	{ .compatible = "adv7511" },
	{ },
};
MODULE_DEVICE_TABLE(of, i2c_adv7511_of_match);
#else
static const struct of_device_id i2c_adv7511_of_match = NULL;
#endif

static struct i2c_driver adv7511_driver = {
	.driver = {
		.name = "adv7511",
		.of_match_table = i2c_adv7511_of_match,
	},
	.probe = adv7511_probe,
	.remove = adv7511_remove,
	.id_table = adv7511_id,
};

static int __init adv7511_init(void)
{
	return i2c_add_driver(&adv7511_driver);
}

static void __exit adv7511_exit(void)
{
	i2c_del_driver(&adv7511_driver);
}

module_init(adv7511_init);
module_exit(adv7511_exit);

MODULE_AUTHOR("Davor Joja <davor.joja@logicbricks.com>");
MODULE_DESCRIPTION("Basic ADV7511 I2C driver");
MODULE_LICENSE("GPL v2");
