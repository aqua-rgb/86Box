/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          VideoLogic PowerVR Neon 250 (PMX1) register handling.
 *
 *
 *
 * Authors: aqua-rgb <aqua-rgb@users.github.com>
 *
 *          Copyright 2025-03-21 aqua-rgb.
 */

#ifndef VID_NEON250_REG_H
#define VID_NEON250_REG_H

#include <86box/vid_neon250_3d.h>

/* Register space management */
void pvr_reg_init(pvr_3d_state_t* state);
void pvr_reg_close(pvr_3d_state_t* state);
void pvr_reg_reset(pvr_3d_state_t* state);

/* Register access */
void pvr_reg_write(pvr_3d_state_t* state, uint32_t addr, uint32_t val);
uint32_t pvr_reg_read(pvr_3d_state_t* state, uint32_t addr);

/* Status update functions */
void pvr_reg_render_complete(pvr_3d_state_t* state);
void pvr_reg_dma_complete(pvr_3d_state_t* state);
void pvr_reg_vblank(pvr_3d_state_t* state);

#endif /* VID_NEON250_REG_H */