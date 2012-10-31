/* 
 * Simple Xylon logiBITBLIT Open Firmware driver
 * 
 * Author: Jamey Hicks
 * email: jamey.hicks@nokia.com
 *
 * Copyright 2012 Nokia, Inc.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#define DRIVER_NAME "xylonbitblit"
#define DEVICE_NAME "logibitblit"
#define DRIVER_DESCRIPTION "Xylon logiBITBLIT driver"
#define DRIVER_VERSION "0.1"

#define DEBUG
#ifdef DEBUG
#define driver_devel(format, ...) \
	do { \
		printk(KERN_INFO format, ## __VA_ARGS__); \
	} while (0)
#else
#define driver_devel(format, ...)
#endif

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/ion.h>
#include "../gpu/ion/ion_priv.h"

struct xylon_bitblit_data {
	struct platform_device *pdev;
        struct cdev cdev;
        struct ion_platform_heap heap;
        struct ion_client *ion_client;
};

static struct class *bitblit_class;
static dev_t bitblit_devt;
static struct ion_device *ion_dev;
static struct ion_heap *ion_heap;

int xylon_bitblit_open(struct inode *inode, struct file *filep)
{
        driver_devel("%s:%d", __FUNCTION__, __LINE__);
        return 0;
}

long xylon_bitblit_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
        driver_devel("%s:%d", __FUNCTION__, __LINE__);
        return 0;
}

int xylon_bitblit_release(struct inode *inode, struct file * filep)
{
        driver_devel("%s:%d", __FUNCTION__, __LINE__);
        return 0;
}

static const struct file_operations xylon_bitblit_fops = {
	.owner = THIS_MODULE,
	.open = xylon_bitblit_open,
	.unlocked_ioctl = xylon_bitblit_ioctl,
	.release = xylon_bitblit_release,
};


int xylon_bitblit_init_driver(struct xylon_bitblit_data *data)
{
	struct device *dev;
	struct resource *reg_res, *irq_res;
	driver_devel("%s\n", __func__);

	dev = &data->pdev->dev;

	reg_res = platform_get_resource(data->pdev, IORESOURCE_MEM, 0);
	irq_res = platform_get_resource(data->pdev, IORESOURCE_IRQ, 0);
	if ((!reg_res) || (!irq_res)) {
		pr_err("Error xylon_bitblit resources\n");
		return -ENODEV;
	}
        
	/* if (request_irq(common_data->xylon_bitblit_irq, xylon_bitblit_isr, */
	/* 		IRQF_TRIGGER_HIGH, DEVICE_NAME, afbi)) { */
	/* 	common_data->xylon_bitblit_irq = 0; */
	/* 	goto err_fb; */
	/* } */

	//mutex_init(&common_data->irq_mutex);
	//init_waitqueue_head(&common_data->xylon_bitblit_vsync.wait);
	//common_data->xylon_bitblit_use_ref = 0;

	return 0;

err_fb:
	/* if (common_data->xylon_bitblit_irq != 0) */
	/* 	free_irq(common_data->xylon_bitblit_irq, afbi); */
	/* if (layer_data->reg_base_virt) */
	/* 	iounmap(layer_data->reg_base_virt); */

err_mem:
	dev_set_drvdata(dev, NULL);

	return -1;
}

int xylon_bitblit_deinit_driver(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int i;

	driver_devel("%s\n", __func__);

#if 0
	if (common_data->xylon_bitblit_use_ref) {
		pr_err("Error xylon_bitblit in use\n");
		return -EINVAL;
	}
#endif
	//free_irq(common_data->xylon_bitblit_irq, afbi);

	dev_set_drvdata(dev, NULL);

	return 0;
}

static int xylon_bitblit_of_probe(struct platform_device *pdev)
{
        struct xylon_bitblit_data *data = kzalloc(sizeof(struct xylon_bitblit_data), GFP_ATOMIC);
        int i, rc;

        driver_devel("%s:%d found device tree entry", __FUNCTION__, __LINE__);

        data->pdev = pdev;
        cdev_init(&data->cdev, &xylon_bitblit_fops);
        cdev_add(&data->cdev, bitblit_devt, 1);
        int id = 0;
        device_create(bitblit_class, &pdev->dev, bitblit_devt, DRIVER_NAME "%d", DRIVER_NAME, id);

        data->heap.type = ION_HEAP_TYPE_CARVEOUT;
        data->heap.id = 0;
        data->heap.name = "zedboard";
        data->heap.base = 0x18000000;
        data->heap.size = 0x08000000;

        ion_heap = ion_heap_create(&data->heap);
        ion_device_add_heap(ion_dev, ion_heap);
        unsigned int heap_mask = ION_HEAP_CARVEOUT_MASK|ION_HEAP_SYSTEM_CONTIG_MASK;
        data->ion_client = ion_client_create(ion_dev, heap_mask, DRIVER_NAME);

        return xylon_bitblit_init_driver(data);
}

static int xylon_bitblit_of_remove(struct platform_device *pdev)
{
        return xylon_bitblit_deinit_driver(pdev);
}


static struct of_device_id xylon_bitblit_of_match[] __devinitdata = {
        { .compatible = "xylon,logibitblt-3.00.a" },
        {/* end of table */},
};
MODULE_DEVICE_TABLE(of, xylon_bitblit_of_match);


static struct platform_driver xylon_bitblit_of_driver = {
        .probe = xylon_bitblit_of_probe,
        .remove = xylon_bitblit_of_remove,
        .driver = {
                .owner = THIS_MODULE,
                .name = DEVICE_NAME,
                .of_match_table = xylon_bitblit_of_match,
        },
};

static int __init xylon_bitblit_of_init(void)
{
#ifndef MODULE
        char *option = NULL;
        /* Set internal module parameters */
        //xylon_bitblit_get_params(option);
#endif
        bitblit_class = class_create(THIS_MODULE, "logibitblit");
        alloc_chrdev_region(&bitblit_devt, 0, 1, "logibitblit");

        ion_dev = ion_device_create(NULL);

        if (platform_driver_register(&xylon_bitblit_of_driver)) {
                pr_err("Error xylon_bitblit driver registration\n");
                return -ENODEV;
        }

        return 0;
}

static void __exit xylon_bitblit_of_exit(void)
{
	class_destroy(bitblit_class);
        ion_device_destroy(ion_dev);
        ion_heap_destroy(ion_heap);
        platform_driver_unregister(&xylon_bitblit_of_driver);
}

#ifndef MODULE
late_initcall(xylon_bitblit_of_init);
#else
module_init(xylon_bitblit_of_init);
module_exit(xylon_bitblit_of_exit);
#endif

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION(DRIVER_DESCRIPTION);
MODULE_VERSION(DRIVER_VERSION);
