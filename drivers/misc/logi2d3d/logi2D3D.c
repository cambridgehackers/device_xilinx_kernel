/*
* Copyright 2012 Xylon d.o.o.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

/*  This is a simple utility driver that is used for hardware synchronization and
    memory mapping of Xylon 2D and 3D IP cores.
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/ctype.h>
#include "logi2D3D.h"

#ifdef CONFIG_OF
/* For open firmware. */
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#endif


#define DRIVER_NAME "logi2D3D"

#define MEM_SIZE (0x1000000*64)

/* logiBITBLT defines */
#define BB_BASE_PHYS_DEF        0x40080000  /*< design specific - default value */
#define BB_REG_RANGE_DEF        0x140       /*< design specific - default value */
#define BB_ISR_OFFSET           0x38
#define BB_IER_OFFSET           0x3c
#define BB_CTRL0_OFFSET         0
#define BB_CTRL0_BUSY_MSK       0x80
#define BB_XXX_EN_INT           0x01
#define BB_XXX_INT              0x01
#define BB_IRQ_DEF              89          /*<  zynq int vector - design specific*/

/* logiBMP defines */
#define BMP_BASE_PHYS_DEF       0x40090000  /*< design specific  - default value  */
#define BMP_REG_RANGE_DEF       0x90        /*< design specific  - default value  */
#define BMP_IER_OFFSET          0x7c
#define BMP_ISR_OFFSET          0x80
#define BMP_CTRL_ST_OFFSET      0
#define BMP_CTRL_ST_READY_MSK   0x80
#define BMP_XXX_EN_INT          0x01
#define BMP_XXX_INT             0x01
#define BMP_IRQ_DEF             88          /*<  zynq int vector  - default value  */

/* logi3D defines */
#define L3D_BASE_PHYS_DEF       0x400B0000  /*< design specific - default value */
#define L3D_REG_RANGE_DEF       0x98        /*< design specific - default value */
#define L3D_ISR_OFFSET          0x88
#define L3D_IER_OFFSET          0x90
#define L3D_CTRL0_OFFSET        0
#define L3D_CTRL0_READY_MSK     0x80
#define L3D_XXX_EN_INT          0x01
#define L3D_XXX_INT             0x01
#define L3D_IRQ_DEF             87          /*<  zynq int vector - design specific*/

/*    printk(KERN_INFO "reg_write %lx -> %lx \n",val, address); \ */
#define reg_write(val, address) do { \
writel(val, address); } while(0)

struct logiBitBlt_dev{
    wait_queue_head_t wait_queue;
    int isr_status_reg;
    struct mutex mutex;
    void *base_virt;
    unsigned char irq_wait_flag;
    int irq_num;
};

struct logiBmp_dev{
    wait_queue_head_t wait_queue;
    int isr_status_reg;
    struct mutex mutex;
    void *base_virt;
    unsigned char irq_wait_flag;
    int irq_num;
};

struct logi3d_dev{
    wait_queue_head_t wait_queue;
    int isr_status_reg;
    struct mutex mutex;
    void *base_virt;
    int irq_num;
    unsigned char irq_wait_flag;
    unsigned char mmap_cached;
    unsigned char sem_created;
    wait_queue_head_t sem_wait_queue;
    struct mutex sem_mutex;
    struct file *sem_locking_fp;
};

struct logi2d3d_device {
    struct cdev cdev;

    struct logiBitBlt_dev bitblt_dev;
    struct logiBmp_dev bmp_dev;
    struct logi3d_dev l3d_dev;
};


static struct logi2d3d_device *l2d3d_stat;
static dev_t l2d3d_devid;
static struct class *l2d3d_class;
static bool in_use;


static irqreturn_t bb_isr(int irq, void *dev_id)
{
    struct logi2d3d_device *l2d3d = (struct logi2d3d_device *)dev_id;
    struct logiBitBlt_dev *bitblt = &l2d3d->bitblt_dev;
    u32 isr = readl(bitblt->base_virt + BB_ISR_OFFSET);
    bitblt->isr_status_reg = isr;
    if (isr & BB_XXX_INT) {
        writel(isr, bitblt->base_virt + BB_ISR_OFFSET);
        bitblt->irq_wait_flag = 1;
        wake_up_interruptible(&bitblt->wait_queue);
        return IRQ_HANDLED;
    }
    return IRQ_NONE;
}


static irqreturn_t bmp_isr(int irq, void *dev_id)
{
    struct logi2d3d_device *l2d3d = (struct logi2d3d_device *)dev_id;
    struct logiBmp_dev *bmp = &l2d3d->bmp_dev;
    u32 isr = readl(bmp->base_virt + BMP_ISR_OFFSET);
    bmp->isr_status_reg = isr;
    if (isr & BMP_XXX_INT) {
        writel(isr, bmp->base_virt + BMP_ISR_OFFSET);
        bmp->irq_wait_flag = 1;
        wake_up_interruptible(&bmp->wait_queue);
        return IRQ_HANDLED;
    }
    return IRQ_NONE;
}


static irqreturn_t l3d_isr(int irq, void *dev_id)
{
    struct logi2d3d_device *l2d3d = (struct logi2d3d_device *)dev_id;
    struct logi3d_dev *l3d = &l2d3d->l3d_dev;
    u32 isr = readl(l3d->base_virt + L3D_ISR_OFFSET);
    l3d->isr_status_reg = isr;
    if (isr & L3D_XXX_INT) {
        writel(isr, l3d->base_virt + L3D_ISR_OFFSET);
        l3d->irq_wait_flag = 1;
        wake_up_interruptible(&l3d->wait_queue);
        return IRQ_HANDLED;
    }
    return IRQ_NONE;
}


static int get_irq_num(struct device_node *dn)
{
    const unsigned int *val;
    int vallen;

    val = of_get_property(dn, "interrupts", &vallen);
    if (!val)
        return 0;

    /* On kernel version 3.3 interrupts are coded in dts "interrupts = <0 58 0>;"
       while on previous version it was "interrupts = <90 0>;" */
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,3,0)
    if (vallen < 1)
        return 0;
    return be32_to_cpu(val[0]);
#else
    if (vallen < 2)
        return 0;
    return be32_to_cpu(val[1]) + 32;
#endif
}


 // init logiBITBLT hw related stuff
static int init_bb(struct logi2d3d_device *l2d3d)
{
    struct device_node *dn;
    unsigned long address, size;
    int ret=0;
    struct logiBitBlt_dev *bitblt = &l2d3d->bitblt_dev;
    const unsigned int *val;
    int vallen;

    dn = of_find_node_by_name(NULL, "logibitblt");
    if (!dn)
    {
        return -1;
    }

    val  = of_get_property(dn, "reg", &vallen);
    address = be32_to_cpu(val[0]);
    size    = be32_to_cpu(val[1]);

    bitblt->irq_num = get_irq_num(dn);
    bitblt->base_virt = ioremap_nocache(address, size);
    if (bitblt->base_virt == NULL) {
        printk(KERN_ERR "%s ERROR ioremap_nocache 0x%lx failed\n", __func__, address);
        ret = -1;
    }
    if (ret== 0 && request_irq(bitblt->irq_num, bb_isr, IRQF_TRIGGER_HIGH, DRIVER_NAME, l2d3d)) {
        printk(KERN_ERR "%s error irq %d not available \n", __func__, bitblt->irq_num);
        //free_irq(irq, l2d3d);
        ret = -1;
    }

    if(ret == 0){
        bitblt->irq_wait_flag = 0;

        reg_write(BB_XXX_EN_INT, bitblt->base_virt + BB_IER_OFFSET);

        init_waitqueue_head(&bitblt->wait_queue);
        mutex_init(&bitblt->mutex);

        printk(KERN_INFO "%s BITBLT REG(%lx %lx), IRQ %d \n", DRIVER_NAME,  address, size, bitblt->irq_num);

    }
    return ret;
}

 // deinit logiBITBLT hw related stuff
static int deinit_bb(struct logi2d3d_device *l2d3d)
{
    struct logiBitBlt_dev *bitblt = &l2d3d->bitblt_dev;
    reg_write(0, bitblt->base_virt + BB_IER_OFFSET);
    free_irq(bitblt->irq_num, l2d3d);
    iounmap(bitblt->base_virt);
    return 0;
}


 // init logiBMP hw related stuff
static int init_bmp(struct logi2d3d_device *l2d3d)
{
    struct device_node *dn;
    unsigned long address, size;
    int ret=0;
    struct logiBmp_dev *bmp = &l2d3d->bmp_dev;
    const unsigned int *val;
    int vallen;

    dn = of_find_node_by_name(NULL, "logibmp");
    if (!dn)
    {
        return -1;
    }

    val  = of_get_property(dn, "reg", &vallen);
    address = be32_to_cpu(val[0]);
    size    = be32_to_cpu(val[1]);

    bmp->irq_num = get_irq_num(dn);
    bmp->base_virt = ioremap_nocache(address, size);

    if (bmp->base_virt == NULL) {
        printk(KERN_ERR "%s ERROR ioremap_nocache 0x%lx failed\n", __func__, address);
        ret = -1;
    }
    if (ret== 0 && request_irq(bmp->irq_num, bmp_isr, IRQF_TRIGGER_HIGH, DRIVER_NAME, l2d3d)) {
        printk(KERN_ERR "%s error irq %d not available \n", __func__, bmp->irq_num);
        //free_irq(irq, l2d3d);
        ret = -1;
    }

    if(ret == 0){
        bmp->irq_wait_flag = 0;
        reg_write(BMP_XXX_EN_INT, bmp->base_virt + BMP_IER_OFFSET);

        init_waitqueue_head(&bmp->wait_queue);
        mutex_init(&bmp->mutex);
        printk(KERN_INFO "%s BMP REG(%lx %lx), IRQ %d \n", DRIVER_NAME,  address, size, bmp->irq_num);
    }
    return ret;
}

 // deinit logiBMP hw related stuff
static int deinit_bmp(struct logi2d3d_device *l2d3d)
{
    struct logiBmp_dev *bmp = &l2d3d->bmp_dev;
    reg_write(0, bmp->base_virt + BMP_IER_OFFSET);
    free_irq(bmp->irq_num, l2d3d);
    iounmap(bmp->base_virt);
    return 0;
}

 // init logi3D hw related stuff
static int init_3d(struct logi2d3d_device *l2d3d)
{
    struct device_node *dn;
    unsigned long address, size;
    int ret=0;
    struct logi3d_dev *l3d = &l2d3d->l3d_dev;
    const unsigned int *val;
    int vallen;

    dn = of_find_node_by_name(NULL, "logi3d");
    if (!dn)
    {
        return -1;
    }

    val  = of_get_property(dn, "reg", &vallen);
    address = be32_to_cpu(val[0]);
    size    = be32_to_cpu(val[1]);

    l3d->irq_num = get_irq_num(dn);
    l3d->base_virt = ioremap_nocache(address, size);

    if (l3d->base_virt == NULL) {
        printk(KERN_ERR "%s ERROR ioremap_nocache 0x%lx failed\n", __func__, address);
        ret = -1;
    }
    if (ret== 0 && request_irq(l3d->irq_num, l3d_isr, IRQF_TRIGGER_HIGH, DRIVER_NAME, l2d3d)) {
        printk(KERN_ERR "%s error irq %d not available \n", __func__, l3d->irq_num);
        //free_irq(irq, l2d3d);
        ret = -1;
    }

    if(ret == 0){
        l3d->irq_wait_flag = 0;
        reg_write(~0, l3d->base_virt + L3D_IER_OFFSET); /* disable int */

        init_waitqueue_head(&l3d->wait_queue);
        mutex_init(&l3d->mutex);
        printk(KERN_INFO "%s L3D REG(%lx %lx), IRQ %d \n", DRIVER_NAME, address, size, l3d->irq_num);
    }

    l3d->mmap_cached = 0;
    l3d->sem_created = 0;
    init_waitqueue_head(&l3d->sem_wait_queue);
    mutex_init(&l3d->sem_mutex);
    l3d->sem_locking_fp = 0;

	return ret;
}

 // deinit logi3D hw related stuff
static int deinit_3d(struct logi2d3d_device *l2d3d)
{
    struct logi3d_dev *l3d = &l2d3d->l3d_dev;
    reg_write(~0, l3d->base_virt + L3D_IER_OFFSET); /* disable int */
    free_irq(l3d->irq_num, l2d3d);
    iounmap(l3d->base_virt);
    return 0;
}

 // closing logi3D file descriptor
static int close_3d(struct file *fp)
{
    struct logi2d3d_device *l2d3d = fp->private_data;
    struct logi3d_dev *l3d = &l2d3d->l3d_dev;

    if (fp == l3d->sem_locking_fp)
    {
        l3d->sem_locking_fp = 0;
        wake_up_interruptible(&l3d->sem_wait_queue);
    }
    
    return 0;
}


//////////////////////////////////////////

static int logi2d3d_open(struct inode *inode, struct file *fp)
{
    struct logi2d3d_device *l2d3d;

    l2d3d = container_of(inode->i_cdev, struct logi2d3d_device, cdev);
    fp->private_data = l2d3d;
    if (l2d3d == NULL)
        return -ENODEV;

    in_use = true;
    return 0;
}

static int logi2d3d_release(struct inode *inode, struct file *fp)
{
    struct logi2d3d_device *l2d3d = fp->private_data;

    if (l2d3d == NULL) {
        return -ENODEV;
    }
    else {
        close_3d(fp);
        in_use = false;
        return 0;
    }
}

static long logi2d3d_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
    struct logi2d3d_device *l2d3d = fp->private_data;
    struct logiBitBlt_dev *bitblt;
    struct logiBmp_dev *bmp;
    struct logi3d_dev *l3d;
    int ret = 0, ret_wait=1;

    switch (cmd) {
    case L2D3D_IOCTL_WAIT_BB_INT:
        bitblt = &l2d3d->bitblt_dev;
        mutex_lock(&bitblt->mutex);
        if(readl(bitblt->base_virt + BB_CTRL0_OFFSET) & BB_CTRL0_BUSY_MSK)
        {
            writel(0xff, bitblt->base_virt + BB_ISR_OFFSET);             /*clear all previous interrupts */
            bitblt->isr_status_reg = 0;
            bitblt->irq_wait_flag = 0;
            reg_write(BB_XXX_EN_INT, bitblt->base_virt + BB_IER_OFFSET); /* enable int */

            /* requred final check before sleep */
            if(readl(bitblt->base_virt + BB_CTRL0_OFFSET) & BB_CTRL0_BUSY_MSK)
            {
                ret_wait = wait_event_interruptible_timeout(bitblt->wait_queue, (bitblt->irq_wait_flag == 1), HZ);
            }

            reg_write(0, bitblt->base_virt + BB_IER_OFFSET);  /* disable int */
            if (ret_wait == 0){
                printk(KERN_ERR "logi2D3D: Xylon logiBITBLT INT wait timeout, l2d3d->isr_status_reg = 0x%x!\n", bitblt->isr_status_reg);
                ret = 1;
            }

            if(readl(bitblt->base_virt + BB_CTRL0_OFFSET) & BB_CTRL0_BUSY_MSK)
            {
                printk(KERN_ERR "logi2D3D: Xylon logiBITBLT ERR busy after wait??\n");
            }

        }
        else{
            u32 isr = readl(bitblt->base_virt + BB_ISR_OFFSET);
            writel(isr, bitblt->base_virt + BB_ISR_OFFSET);
            // printk(KERN_INFO "logi2D3D: Xylon logiBITBLT NO wait !\n");
        }
        mutex_unlock(&bitblt->mutex);
        break;

    case L2D3D_IOCTL_WAIT_BMP_INT:
        bmp = &l2d3d->bmp_dev;
        mutex_lock(&bmp->mutex);
        if(! (readl(bmp->base_virt + BMP_CTRL_ST_OFFSET) & BMP_CTRL_ST_READY_MSK))
        {
            writel(0xff, bmp->base_virt + BMP_ISR_OFFSET); /*clear all previous interrupts */
            bmp->isr_status_reg = 0;
            bmp->irq_wait_flag = 0;
            reg_write(BMP_XXX_EN_INT, bmp->base_virt + BMP_IER_OFFSET); /* enable int */

            /* requred final check before sleep */
            if(! (readl(bmp->base_virt + BMP_CTRL_ST_OFFSET) & BMP_CTRL_ST_READY_MSK))
            {
                ret_wait = wait_event_interruptible_timeout(bmp->wait_queue, (bmp->irq_wait_flag == 1), HZ/10);
            }

            reg_write( 0 , bmp->base_virt + BMP_IER_OFFSET); /* disable int */
            if (ret_wait == 0) {
                printk(KERN_INFO "BMP INT wait timeout\n");
                ret = 1;
            }
        }
        else{
            u32 isr = readl(bmp->base_virt + BMP_ISR_OFFSET);
            writel(isr, bmp->base_virt + BMP_ISR_OFFSET);
        }
        mutex_unlock(&bmp->mutex);
        break;

    case L2D3D_IOCTL_WAIT_L3D_INT:
        l3d = &l2d3d->l3d_dev;
        mutex_lock(&l3d->mutex);
        if(! (readl(l3d->base_virt + L3D_CTRL0_OFFSET) & L3D_CTRL0_READY_MSK))
        {
            writel(0xff, l3d->base_virt + L3D_ISR_OFFSET); /*clear all previous interrupts */
            l3d->isr_status_reg = 0;
            l3d->irq_wait_flag = 0;
            reg_write(~L3D_XXX_EN_INT, l3d->base_virt + L3D_IER_OFFSET); /* enable int */

            /* requred final check before sleep */
            if(! (readl(l3d->base_virt + L3D_CTRL0_OFFSET) & L3D_CTRL0_READY_MSK))
            {
                ret_wait = wait_event_interruptible_timeout(l3d->wait_queue, (l3d->irq_wait_flag == 1), HZ);
            }

            reg_write(~0 , l3d->base_virt + L3D_IER_OFFSET); /* disable int */
            if (ret_wait == 0) {
                printk(KERN_INFO "L3D INT wait timeout\n");
                ret = 1;
            }
        }
        else{
            u32 isr = readl(l3d->base_virt + L3D_ISR_OFFSET);
            writel(isr, l3d->base_virt + L3D_ISR_OFFSET);
        }
        mutex_unlock(&l3d->mutex);
        break;

    case L2D3D_IOCTL_L3D_SEMAPHORE:
        l3d = &l2d3d->l3d_dev;
        switch (arg) {
        case L3D_SEMAPHORE_CREATE:
            if (l3d->sem_created)
            {
                ret = -EINVAL;
                break;
            }
            l3d->sem_created = 1;
            /* no break - semaphore is created and then locked */
        case L3D_SEMAPHORE_LOCK:
            while (ret_wait)
            {
                mutex_lock(&l3d->sem_mutex);
                if (0 == l3d->sem_locking_fp)
                {
                    l3d->sem_locking_fp = fp;
                    ret_wait = 0;
                }
                mutex_unlock(&l3d->sem_mutex);

                if (ret_wait)
                {
                    if (0 == wait_event_interruptible_timeout(l3d->sem_wait_queue, (0 == l3d->sem_locking_fp), 3*HZ))
                        printk(KERN_ERR "logi3D wait timeout expired !!!\n");
                }
            }
            break;

        case L3D_SEMAPHORE_DESTROY:
            l3d->sem_created = 0;
            /* no break - must be unlocked on destroy */
        case L3D_SEMAPHORE_CLOSE:
        case L3D_SEMAPHORE_UNLOCK:
            l3d = &l2d3d->l3d_dev;
            if (fp == l3d->sem_locking_fp)
            {
                l3d->sem_locking_fp = 0;
                wake_up_interruptible(&l3d->sem_wait_queue);
            }
            break;

        case L3D_SEMAPHORE_OPEN:
            if (l3d->sem_created)
            {
                ret = -EINVAL;
            }
            break;

        default:
            ret = -EINVAL;
            break;
        }
        break;

    case L2D3D_IOCTL_L3D_MMAP_CACHED:
        l3d = &l2d3d->l3d_dev;
        l3d->mmap_cached = arg;
        break;

    default:
        ret = -EINVAL;
        break;
    }
    return ret;
}

static int logi2d3d_mmap(struct file *fp, struct vm_area_struct *vma)
{
    struct logi2d3d_device *l2d3d = fp->private_data;
    struct logi3d_dev *l3d = &l2d3d->l3d_dev;
    int err = -EINVAL;

    if (vma->vm_end - vma->vm_start <= MEM_SIZE) {
        vma->vm_flags |= VM_RESERVED | VM_IO;
        /* use cached mmaping only once per mmap, then return to default (non-cached) */
        if (l3d->mmap_cached)
            l3d->mmap_cached = 0;
        else
            vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
        err = remap_pfn_range(
            vma,
            vma->vm_start,
            vma->vm_pgoff,
            vma->vm_end - vma->vm_start,
            vma->vm_page_prot);
    }

    return err;
}


struct file_operations logi2d3d_fops = {
    .owner = THIS_MODULE,
    .open = logi2d3d_open,
    .release = logi2d3d_release,
    .unlocked_ioctl = logi2d3d_ioctl,
    .mmap = logi2d3d_mmap,
};



static int __init logi2d3d_init(void)
{
    struct device *device;
    int  err;

    l2d3d_stat = kzalloc(sizeof(*l2d3d_stat), GFP_KERNEL);
    if (NULL == l2d3d_stat)
        return -ENOMEM;

    err = alloc_chrdev_region(&l2d3d_devid, 0, 1, DRIVER_NAME);
    if (err)
        goto err_handle;

    l2d3d_class = class_create(THIS_MODULE, DRIVER_NAME);

    cdev_init(&l2d3d_stat->cdev, &logi2d3d_fops);
    l2d3d_stat->cdev.owner = THIS_MODULE;
    l2d3d_stat->cdev.ops = &logi2d3d_fops;
    err = cdev_add(&l2d3d_stat->cdev, l2d3d_devid, 1);
    if (err < 0) {
        printk(KERN_ERR "%s adding char device failed error %d\n",
            __func__, err);
        goto err_handle;
    }
    device = device_create(l2d3d_class, NULL, MKDEV(MAJOR(l2d3d_devid),0), "%s", DRIVER_NAME);
    if (!device) {
        printk(KERN_ERR "%s unable to create char device %s\n", __func__,
            DRIVER_NAME);
        goto err_handle;
    }
    printk(KERN_INFO "logi2D3D initialized\n");

    init_bb(l2d3d_stat);
    init_bmp(l2d3d_stat);
    init_3d(l2d3d_stat);

    return 0;

err_handle:
    kfree(l2d3d_stat);

    return err;
}

static void __exit logi2d3d_exit(void)
{
    int major = MAJOR(l2d3d_devid);
    if (in_use == true) {
        printk(KERN_INFO "logi2D3D driver in use\n");
        return;
    }

    deinit_bb(l2d3d_stat);
    deinit_bmp(l2d3d_stat);
    deinit_3d(l2d3d_stat);

    unregister_chrdev_region(l2d3d_devid, 1);

    device_destroy(l2d3d_class, MKDEV(major, 0));
    cdev_del(&l2d3d_stat->cdev);
    l2d3d_devid = 0;


    kfree(l2d3d_stat);
    class_destroy(l2d3d_class);
}


module_init(logi2d3d_init);
module_exit(logi2d3d_exit);

MODULE_LICENSE("GPL v2");
MODULE_VERSION("0.1");
