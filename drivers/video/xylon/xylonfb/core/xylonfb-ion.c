/*
 * drivers/video/xylonfb/core/xylonfb-ion.c
 *
 * Copyright (C) 2012 Nokia, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/ion.h>
#include <linux/slab.h>
#include "../../../../gpu/ion/ion_priv.h"
#include "xylonfb.h"

int xylonfb_ion_probe(struct xylonfb_common_data *common_data)
{
        driver_devel("%s:%d\n", __FUNCTION__, __LINE__);

        common_data->ion_dev = ion_device_create(NULL);
        driver_devel("%s:%d ion_dev=%p\n",
                     __FUNCTION__, __LINE__, common_data->ion_dev);

        struct ion_platform_heap *heap_data =
                kzalloc(sizeof(struct ion_platform_heap), GFP_KERNEL);
        heap_data->type = ION_HEAP_TYPE_SYSTEM_CONTIG;
        heap_data->id = 1;
        heap_data->name = "xylonfb-ion-heap";
        heap_data->base = 0x18000000; // not used for system_contig
        heap_data->size = 0x08000000; // not used for system_contig

        common_data->ion_heap = ion_heap_create(heap_data);
        driver_devel("%s:%d ion_heap=%p ops=%p\n",
                     __FUNCTION__, __LINE__, common_data->ion_heap,
                     common_data->ion_heap ? common_data->ion_heap->ops : 0);

        ion_device_add_heap(common_data->ion_dev, common_data->ion_heap);
        return 0;
}
