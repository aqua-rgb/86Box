/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          VideoLogic PowerVR Neon 250 (PMX1) 3D rendering pipeline.
 *
 *
 *
 * Authors: aqua-rgb <aqua-rgb@users.github.com>
 *
 *          Copyright 2025 aqua-rgb.
 */

#ifndef VID_NEON250_3D_H
#define VID_NEON250_3D_H

/* Forward declarations */
struct neon250_t;
typedef struct pvr_3d_state_t pvr_3d_state_t;

/* Functions for the 3D pipeline */
pvr_3d_state_t* pvr_3d_init(struct neon250_t* neon250);
void pvr_3d_close(pvr_3d_state_t* state);
void pvr_3d_write(pvr_3d_state_t* state, uint32_t addr, uint32_t val);
uint32_t pvr_3d_read(pvr_3d_state_t* state, uint32_t addr);
void pvr_3d_update_display(pvr_3d_state_t* state, int width, int height, 
                          int stride, int bpp, uint8_t* vram);

#endif /* VID_NEON250_3D_H */