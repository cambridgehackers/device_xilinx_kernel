/*
 * Xylon logiCVC frame buffer driver internal data structures
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

#ifndef __XYLON_FB_DATA_H__
#define __XYLON_FB_DATA_H__


#include <linux/wait.h>
#include <linux/mutex.h>
#include <linux/fb.h>
#include "logicvc.h"


#define DRIVER_NAME "xylonfb"
#define DEVICE_NAME "logicvc"
#define DRIVER_DESCRIPTION "Xylon logiCVC frame buffer driver"
#define DRIVER_VERSION "1.2"

/* FB driver flags */
#define FB_DMA_BUFFER        0x01
#define FB_MEMORY_LE         0x02
#define FB_VMODE_INIT        0x10
#define FB_DEFAULT_VMODE_SET 0x20
#define FB_VMODE_SET         0x40
#define FB_RESERVED_0x100    0x100


#ifdef DEBUG
#define driver_devel(format, ...) \
	do { \
		printk(KERN_INFO format, ## __VA_ARGS__); \
	} while (0)
#else
#define driver_devel(format, ...)
#endif

struct xylonfb_layer_data;

#define VMODE_NAME_SZ 20
struct xylonfb_vmode_data {
	u32 ctrl_reg;
	struct fb_videomode fb_vmode;
	char fb_vmode_name[VMODE_NAME_SZ+1];
};

struct xylonfb_registers {
	u32 dtype_reg;
	u32 bg_reg;
	u32 unused_reg[3];
	u32 int_mask_reg;
};

struct xylonfb_layer_registers {
	u32 hoff_reg;
	u32 voff_reg;
	u32 hpos_reg;
	u32 vpos_reg;
	u32 width_reg;
	u32 height_reg;
	u32 alpha_reg;
	u32 ctrl_reg;
	u32 trans_reg;
};

struct xylonfb_register_access {
	u32 (*xylonfb_get_reg_val)
		(void *reg_base_virt, unsigned long offset,
		 struct xylonfb_layer_data *layer_data);
	void (*xylonfb_set_reg_val)
		(u32 value, void *reg_base_virt, unsigned long offset,
		 struct xylonfb_layer_data *layer_data);
};

struct xylonfb_layer_fix_data {
	unsigned int offset;
	unsigned short buffer_offset;
	unsigned short width;
	unsigned short height;
	unsigned char bpp;
	unsigned char bpp_virt;
	unsigned char layer_type;
	unsigned char alpha_mode;
	/* higher 4 bits: number of layer buffers, lower 4 bits: layer ID */
	unsigned char layer_fix_info;
};

struct xylonfb_sync {
	wait_queue_head_t wait;
	unsigned int cnt;
};

struct xylonfb_common_data {
	struct device *dev;
	struct mutex irq_mutex;
	struct xylonfb_register_access reg_access;
	struct xylonfb_registers *reg_list;
	struct xylonfb_sync xylonfb_vsync;
	struct xylonfb_vmode_data vmode_data;
	struct xylonfb_vmode_data vmode_data_current;
#ifdef CONFIG_ION
        struct ion_heap *ion_heap[3];
        struct ion_device *ion_dev;
#endif
	/* Delay after applying display power and
		before applying display signals */
	unsigned int power_on_delay;
	/* Delay after applying display signal and
		before applying display backlight power supply */
	unsigned int signal_on_delay;
	unsigned short xylonfb_flags;
	unsigned char layers;
	unsigned char xylonfb_irq;
	unsigned char xylonfb_use_ref;
	unsigned char xylonfb_used_layer;
	unsigned char bg_layer_bpp;
	unsigned char bg_layer_alpha_mode;
	/* higher 4 bits: display interface, lower 4 bits: display color space */
	unsigned char display_interface_type;
};

struct xylonfb_layer_data {
	struct xylonfb_common_data *xylonfb_cd;
	struct mutex layer_mutex;
	dma_addr_t reg_base_phys;
	dma_addr_t fb_phys;
	void *reg_base_virt;
	void *fb_virt;
	unsigned long fb_size;
	void *layer_reg_base_virt;
	void *layer_clut_base_virt;
	struct xylonfb_layer_fix_data layer_fix;
	struct xylonfb_layer_registers *layer_reg_list;
	unsigned char layer_ctrl_flags;
	unsigned char layer_use_ref;
};

struct xylonfb_init_data {
	struct platform_device *pdev;
	struct xylonfb_vmode_data vmode_data;
	struct xylonfb_layer_fix_data lfdata[LOGICVC_MAX_LAYERS];
	unsigned long vmem_base_addr;
	unsigned long vmem_high_addr;
	unsigned char layer_ctrl_flags[LOGICVC_MAX_LAYERS];
	unsigned char layers;
	unsigned char active_layer;
	unsigned char bg_layer_bpp;
	unsigned char bg_layer_alpha_mode;
	unsigned char display_interface_type;
	unsigned short flags;
	bool vmode_params_set;
};


/* xylonfb core interface functions */
extern int xylonfb_get_params(char *options);
extern int xylonfb_init_driver(struct xylonfb_init_data *init_data);
extern int xylonfb_deinit_driver(struct platform_device *pdev);
extern int xylonfb_ioctl(struct fb_info *fbi,
	unsigned int cmd, unsigned long arg);

#ifdef CONFIG_ION
int xylonfb_ion_probe(struct xylonfb_common_data *common_data);
#endif

#endif /* __XYLON_FB_DATA_H__ */
