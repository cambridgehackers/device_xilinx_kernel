/*
* logi2D3D.h
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


#ifndef __L3D_H__
#define __L3D_H__

/* these are obsolete
#define L2D3D_IOCTL_MAP_REG         0
#define L2D3D_IOCTL_MAP_MEM         1
#define L2D3D_IOCTL_MAP_FRAME       2
*/

#define L2D3D_IOCTL_WAIT_BB_INT     3
#define L2D3D_IOCTL_WAIT_BMP_INT    4
#define L2D3D_IOCTL_WAIT_L3D_INT    5

#define L2D3D_IOCTL_L3D_SEMAPHORE   6
#define L2D3D_IOCTL_L3D_MMAP_CACHED 7

/* Semaphore operation is defined by these values
   sent via first entry in ioctl argument list */
enum {
    L3D_SEMAPHORE_CREATE    = 0,
    L3D_SEMAPHORE_DESTROY   = 1,
    L3D_SEMAPHORE_LOCK      = 2,
    L3D_SEMAPHORE_UNLOCK    = 3,
    L3D_SEMAPHORE_OPEN      = 4,
    L3D_SEMAPHORE_CLOSE     = 5
};

#endif
