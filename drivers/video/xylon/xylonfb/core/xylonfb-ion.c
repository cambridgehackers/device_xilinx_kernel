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

struct ion_device *xylon_ion_device;

int xylonfb_ion_probe(struct xylonfb_common_data *common_data)
{
        int i;
        struct ion_platform_heap heap_data;

        driver_devel("%s:%d\n", __FUNCTION__, __LINE__);

        common_data->ion_dev = ion_device_create(NULL);
        xylon_ion_device = common_data->ion_dev;
        driver_devel("%s:%d ion_dev=%p\n",
                     __FUNCTION__, __LINE__, common_data->ion_dev);

        for (i = 0; i < 3; i++) {
                char name[22];
                snprintf(name, sizeof(name), "xylonfb-ion-heap-%d", i);
                switch (i) {
                case 0:
                        heap_data.type = ION_HEAP_TYPE_CARVEOUT;
                        break;
                case 1:
                        heap_data.type = ION_HEAP_TYPE_SYSTEM_CONTIG;
                        break;
                default:
                        heap_data.type = ION_HEAP_TYPE_SYSTEM;
                        break;
                }
                heap_data.id = i;
                heap_data.name = name;
                // fixme use devicetree
                heap_data.base = 0x18000000; // not used for system_contig or system
                heap_data.size = 0x08000000; // not used for system_contig or system

                common_data->ion_heap[i] = ion_heap_create(&heap_data);
                driver_devel("%s:%d ion_heap=%p ops=%p\n",
                             __FUNCTION__, __LINE__, common_data->ion_heap[i],
                             common_data->ion_heap[i] ? common_data->ion_heap[i]->ops : 0);

                ion_device_add_heap(common_data->ion_dev, common_data->ion_heap[i]);
        }
        return 0;
}
