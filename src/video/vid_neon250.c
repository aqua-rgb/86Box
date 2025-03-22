/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          VideoLogic PowerVR Neon 250 (PMX1) emulation.
 *
 *
 *
 * Authors: aqua-rgb
 *
 *          Copyright 2025 aqua-rgb.
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <wchar.h>
#include <86box/86box.h>
#include <86box/io.h>
#include <86box/mem.h>
#include <86box/rom.h>
#include <86box/pci.h>
#include <86box/device.h>
#include <86box/timer.h>
#include <86box/video.h>
#include <86box/vid_svga.h>
#include <86box/vid_neon250_3d.h>

#define BIOS_ROM_PATH "roms/video/powervr/n0020331.bin"

/* PowerVR Neon 250 registers */
#define PVR_DISP_BRDRCOLR 0x40
#define PVR_DISP_DIWMODE  0x44
#define PVR_DISP_DIWADDRL 0x50
#define PVR_DISP_DIWADDRS 0x54
#define PVR_DISP_DIWSIZE  0x5c
#define PVR_DISP_SYNCCONF 0xd0
#define PVR_DISP_BRDRHORZ 0xd4
#define PVR_DISP_SYNCSIZE 0xd8
#define PVR_DISP_BRDRVERT 0xdc
#define PVR_DISP_DIWCONF  0xe8
#define PVR_DISP_DIWHSTRT 0xec
#define PVR_DISP_DIWVSTRT 0xf0
#define PVR_DISP_PIXDEPTH 0x108

/* PCI device IDs */
#define PCI_VENDOR_NEC    0x1033
#define PCI_DEVICE_NEON250 0x0067

typedef struct neon250_t {
    svga_t      svga;
    rom_t       bios_rom;
    
    uint8_t     regs[4096];  /* Internal registers */
    int         index;       /* Current register index */
    uint32_t    memory_size; /* Video memory size (8, 16, or 32 MB) */
    
    /* PowerVR specific registers */
    uint32_t    disp_start;  /* Address of image within VRAM */
    uint32_t    borderstart_h;
    uint32_t    borderstop_h;
    uint32_t    borderstart_v;
    uint32_t    borderstop_v;
    uint32_t    diwstart_h;  /* Horizontal offset of display field */
    uint32_t    diwstart_v;  /* Vertical offset of display field */
    uint8_t     is_interlaced; /* Interlaced display mode */
    uint8_t     is_lowres;   /* Horizontal pixel-doubling enabled */
    
    mem_mapping_t mmio_mapping;
    mem_mapping_t framebuffer_mapping;

    pvr_3d_state_t *pvr_3d;
    
    /* PCI-specific */
    uint8_t      pci_regs[256];
    uint8_t      int_line;
    int          card;
    
} neon250_t;

static video_timings_t timing_neon250 = {
    .type      = VIDEO_PCI,
    .write_b   = 2,
    .write_w   = 2,
    .write_l   = 4,
    .read_b    = 2,
    .read_w    = 2,
    .read_l    = 4
};

/* Forward declarations */
static void neon250_recalctimings(svga_t *svga);
static void neon250_out(uint16_t addr, uint8_t val, void *priv);
static uint8_t neon250_in(uint16_t addr, void *priv);

static void neon250_mmio_write(uint32_t addr, uint8_t val, void *priv)
{
    neon250_t *neon250 = (neon250_t *)priv;
    addr &= 0xffff;
    
    if (addr >= 0x00100000 && addr < 0x00110000) {
        /* Pass to 3D pipeline */
        pvr_3d_write(neon250->pvr_3d, addr, val);
        return;
    }

    switch (addr) {
        case PVR_DISP_DIWADDRL:
            neon250->regs[addr] = val;
            neon250->disp_start = (neon250->disp_start & 0xffffff00) | val;
            break;
        case PVR_DISP_DIWADDRL + 1:
            neon250->regs[addr] = val;
            neon250->disp_start = (neon250->disp_start & 0xffff00ff) | (val << 8);
            break;
        case PVR_DISP_DIWADDRL + 2:
            neon250->regs[addr] = val;
            neon250->disp_start = (neon250->disp_start & 0xff00ffff) | (val << 16);
            break;
        case PVR_DISP_DIWADDRL + 3:
            neon250->regs[addr] = val;
            neon250->disp_start = (neon250->disp_start & 0x00ffffff) | (val << 24);
            break;
        
        /* Store all register writes for reference */
        default:
            neon250->regs[addr] = val;
            break;
    }
    
    if (addr >= PVR_DISP_BRDRHORZ && addr <= PVR_DISP_DIWVSTRT + 3) {
        neon250_recalctimings(&neon250->svga);
    }
}

static uint8_t neon250_mmio_read(uint32_t addr, void *priv)
{
    neon250_t *neon250 = (neon250_t *)priv;
    addr &= 0xffff;
    
    if (addr >= 0x00100000 && addr < 0x00110000) {
        /* Pass to 3D pipeline */
        return pvr_3d_read(neon250->pvr_3d, addr);
    }

    return neon250->regs[addr];
}

static void neon250_mmio_write_w(uint32_t addr, uint16_t val, void *priv)
{
    neon250_mmio_write(addr, val & 0xff, priv);
    neon250_mmio_write(addr + 1, val >> 8, priv);
}

static void neon250_mmio_write_l(uint32_t addr, uint32_t val, void *priv)
{
    neon250_mmio_write(addr, val & 0xff, priv);
    neon250_mmio_write(addr + 1, (val >> 8) & 0xff, priv);
    neon250_mmio_write(addr + 2, (val >> 16) & 0xff, priv);
    neon250_mmio_write(addr + 3, (val >> 24) & 0xff, priv);
}

static uint16_t neon250_mmio_read_w(uint32_t addr, void *priv)
{
    return neon250_mmio_read(addr, priv) | (neon250_mmio_read(addr + 1, priv) << 8);
}

static uint32_t neon250_mmio_read_l(uint32_t addr, void *priv)
{
    return neon250_mmio_read(addr, priv) | 
           (neon250_mmio_read(addr + 1, priv) << 8) | 
           (neon250_mmio_read(addr + 2, priv) << 16) | 
           (neon250_mmio_read(addr + 3, priv) << 24);
}

static void neon250_out(uint16_t addr, uint8_t val, void *priv)
{
    neon250_t *neon250 = (neon250_t *)priv;
    svga_t *svga = &neon250->svga;
    uint8_t old;
    
    if (((addr & 0xfff0) == 0x3d0 || (addr & 0xfff0) == 0x3b0) && !(svga->miscout & 1))
        addr ^= 0x60;
    
    switch (addr) {
        case 0x3c0:
        case 0x3c1:
            if (!svga->attrff) {
                svga->attraddr = val & 31;
                if ((val & 0x20) != svga->attr_palette_enable) {
                    svga->fullchange = 3;
                    svga->attr_palette_enable = val & 0x20;
                    svga_recalctimings(svga);
                }
            } else {
                svga->attrregs[svga->attraddr & 31] = val;
                if (svga->attraddr < 16)
                    svga->fullchange = changeframecount;
                if (svga->attraddr == 0x10 || svga->attraddr == 0x14 || svga->attraddr < 0x10) {
                    for (uint8_t c = 0; c < 16; c++) {
                        if (svga->attrregs[0x10] & 0x80)
                            svga->egapal[c] = (svga->attrregs[c] & 0xf) | ((svga->attrregs[0x14] & 0xf) << 4);
                        else
                            svga->egapal[c] = (svga->attrregs[c] & 0x3f) | ((svga->attrregs[0x14] & 0xc) << 4);
                    }
                    svga->fullchange = changeframecount;
                }
            }
            svga->attrff ^= 1;
            return;
        
        case 0x3D4:
            svga->crtcreg = val;
            return;
            
        case 0x3D5:
            if ((svga->crtcreg < 7) && (svga->crtc[0x11] & 0x80))
                return;
            if ((svga->crtcreg == 7) && (svga->crtc[0x11] & 0x80))
                val = (svga->crtc[7] & ~0x10) | (val & 0x10);
                
            old = svga->crtc[svga->crtcreg];
            svga->crtc[svga->crtcreg] = val;
            
            if (old != val) {
                if (svga->crtcreg < 0xe || svga->crtcreg > 0x10) {
                    if ((svga->crtcreg == 0xc) || (svga->crtcreg == 0xd)) {
                        svga->fullchange = 3;
                        svga->ma_latch = ((svga->crtc[0xc] << 8) | svga->crtc[0xd]) + ((svga->crtc[8] & 0x60) >> 5);
                    } else {
                        svga->fullchange = changeframecount;
                        neon250_recalctimings(svga);
                    }
                }
            }
            return;
            
        default:
            break;
    }
    
    svga_out(addr, val, svga);
}

static uint8_t neon250_in(uint16_t addr, void *priv)
{
    neon250_t *neon250 = (neon250_t *)priv;
    svga_t *svga = &neon250->svga;
    uint8_t temp = 0xff;
    
    if (((addr & 0xfff0) == 0x3d0 || (addr & 0xfff0) == 0x3b0) && !(svga->miscout & 1))
        addr ^= 0x60;
    
    switch (addr) {
        case 0x3c5:
            if (svga->seqaddr == 0x10)
                return 0x01; /* PowerVR ID? */
            break;
            
        case 0x3D4:
            temp = svga->crtcreg;
            break;
            
        case 0x3D5:
            temp = svga->crtc[svga->crtcreg];
            break;
            
        default:
            temp = svga_in(addr, svga);
            break;
    }
    
    return temp;
}

static void neon250_recalctimings(svga_t *svga)
{
    neon250_t *neon250 = (neon250_t *)container_of(svga, neon250_t, svga);
    
    /* Calculate timing parameters based on CRTC registers */
    svga->hdisp = svga->crtc[1] - ((svga->crtc[5] & 0x60) >> 5);
    svga->hdisp++;
    
    svga->dispend = svga->crtc[0x12] | ((svga->crtc[0x07] & 0x02) << 7) | ((svga->crtc[0x07] & 0x40) << 3);
    svga->dispend++;
    
    if (neon250->is_lowres) {
        svga->hdisp <<= 1;
    }
    
    /* Use these values if direct register access is implemented */
    neon250->borderstart_h = (neon250_mmio_read_l(PVR_DISP_BRDRHORZ, neon250) >> 16) & 0xffff;
    neon250->borderstop_h = neon250_mmio_read_l(PVR_DISP_BRDRHORZ, neon250) & 0xffff;
    neon250->borderstart_v = (neon250_mmio_read_l(PVR_DISP_BRDRVERT, neon250) >> 16) & 0xffff;
    neon250->borderstop_v = neon250_mmio_read_l(PVR_DISP_BRDRVERT, neon250) & 0xffff;
    
    /* Ensure the screen size is reasonable */
    if (svga->hdisp == 0)
        svga->hdisp = 640;
    if (svga->dispend == 0)
        svga->dispend = 480;
        
    /* Calculate vertical timing parameters */
    svga->vtotal = svga->crtc[6] + ((svga->crtc[7] & 1) << 8) + ((svga->crtc[7] & 0x20) << 4) + 1;
    svga->vsyncstart = svga->crtc[0x10] + ((svga->crtc[7] & 4) << 6) + ((svga->crtc[7] & 0x80) << 2);
    
    if (neon250->is_interlaced)
        svga->vtotal *= 2;

    /* Update 3D pipeline's framebuffer parameters */
    if (neon250->pvr_3d) {
        pvr_3d_update_display(neon250->pvr_3d, svga->hdisp, svga->dispend,
                             svga->rowoffset, svga->bpp, svga->vram);
    }
}

static uint8_t neon250_pci_read(int func, int addr, void *priv)
{
    neon250_t *neon250 = (neon250_t *)priv;
    uint8_t ret = 0xff;
    
    if (func > 0)
        return ret;
        
    if (addr >= 0 && addr < 256)
        ret = neon250->pci_regs[addr];
        
    return ret;
}

static void neon250_pci_write(int func, int addr, uint8_t val, void *priv)
{
    neon250_t *neon250 = (neon250_t *)priv;
    
    if (func > 0)
        return;
        
    if (addr >= 0 && addr < 256) {
        switch (addr) {
            case PCI_COMMAND:
                neon250->pci_regs[PCI_REG_COMMAND] = val & 0x37; /* Only bits 0-2 of the command register are writable */
                
                /* Update memory mapping based on PCI command register */
                if (val & PCI_COMMAND_MEM) {
                    mem_mapping_enable(&neon250->svga.mapping);
                    mem_mapping_enable(&neon250->mmio_mapping);
                } else {
                    mem_mapping_disable(&neon250->svga.mapping);
                    mem_mapping_disable(&neon250->mmio_mapping);
                }
                break;
                
            case 0x10: case 0x11: case 0x12: case 0x13: /* Base address 0 - framebuffer */
                /* Handle BAR writes */
                if (addr == 0x10)
                    neon250->pci_regs[addr] = (val & 0xf0) | 0x08; /* 64MB alignment, memory space */
                else if (addr == 0x13)
                    neon250->pci_regs[addr] = val;
                
                /* Update mapping */
                uint32_t base = (neon250->pci_regs[0x13] << 24) | 
                               (neon250->pci_regs[0x12] << 16) | 
                               (neon250->pci_regs[0x11] << 8) | 
                               (neon250->pci_regs[0x10] & 0xf0);
                               
                if (neon250->pci_regs[PCI_COMMAND] & PCI_COMMAND_MEM) {
                    mem_mapping_set_addr(&neon250->svga.mapping, base, 64 << 20);
                }
                break;
                
            case 0x14: case 0x15: case 0x16: case 0x17: /* Base address 1 - MMIO */
                /* Handle BAR writes */
                if (addr == 0x14)
                    neon250->pci_regs[addr] = (val & 0xf0) | 0x00; /* 64KB alignment, memory space */
                else if (addr == 0x17)
                    neon250->pci_regs[addr] = val;
                else if (addr == 0x16)
                    neon250->pci_regs[addr] = val;
                else if (addr == 0x15)
                    neon250->pci_regs[addr] = val;
                
                /* Update mapping */
                uint32_t mmio_base = (neon250->pci_regs[0x17] << 24) | 
                                    (neon250->pci_regs[0x16] << 16) | 
                                    (neon250->pci_regs[0x15] << 8) | 
                                    (neon250->pci_regs[0x14] & 0xf0);
                
                if (neon250->pci_regs[PCI_COMMAND] & PCI_COMMAND_MEM) {
                    mem_mapping_set_addr(&neon250->mmio_mapping, mmio_base, 0x10000);
                }
                break;

            case 0x30: case 0x32: case 0x33:
                neon250->pci_regs[addr] = val;
            if (neon250->pci_regs[0x30] & 0x01) {
                uint32_t addr = (neon250->pci_regs[0x32] << 16) | (neon250->pci_regs[0x33] << 24);
                mem_mapping_set_addr(&neon250->bios_rom.mapping, addr, neon250->bios_rom.mapping.size);
            } else
                mem_mapping_disable(&neon250->bios_rom.mapping);
            break;
                
            case 0x3c: /* Interrupt line */
                neon250->pci_regs[addr] = val;
                neon250->int_line = val;
                break;
                
            /* Writable AGP registers */
            case 0x44: case 0x45: case 0x46: case 0x47:
                neon250->pci_regs[addr] = val;
                break;
                
            default:
                /* Allow writes to other registers */
                neon250->pci_regs[addr] = val;
                break;
        }
    }
}

static void *neon250_init(const device_t *info)
{
    neon250_t *neon250 = calloc(1, sizeof(neon250_t));
    
    neon250->memory_size = device_get_config_int("memory") << 20;
    
    /* Set up SVGA */
    svga_init(info, &neon250->svga, neon250, neon250->memory_size,
              neon250_recalctimings, neon250_in, neon250_out, NULL, NULL);
    
    /* Map the MMIO registers at 0xA05F8000 */
    mem_mapping_add(&neon250->mmio_mapping, 0xA05F8000, 0x10000,
                    neon250_mmio_read, neon250_mmio_read_w, neon250_mmio_read_l,
                    neon250_mmio_write, neon250_mmio_write_w, neon250_mmio_write_l,
                    NULL, MEM_MAPPING_EXTERNAL, neon250);
    
    /* Set up the framebuffer */
    mem_mapping_set_handler(&neon250->svga.mapping,
                           svga_read, svga_readw, svga_readl,
                           svga_write, svga_writew, svga_writel);
    mem_mapping_set_p(&neon250->svga.mapping, neon250);
    
    /* Load the BIOS ROM */
    rom_init(&neon250->bios_rom, BIOS_ROM_PATH, 0xc0000, 0x20000, 0xffff, 0, MEM_MAPPING_EXTERNAL);
    
    /* Set up default display mode */
    neon250->svga.bpp = 16;
    neon250->is_interlaced = 0;
    neon250->is_lowres = 0;
    
    /* Set up PCI */
    neon250->card = pci_add_card(PCI_ADD_AGP, neon250_pci_read, neon250_pci_write, neon250, &neon250->pci_slot);
    
    /* Configure PCI registers */
    neon250->pci_regs[0x00] = PCI_VENDOR_NEC & 0xff;
    neon250->pci_regs[0x01] = (PCI_VENDOR_NEC >> 8) & 0xff;
    neon250->pci_regs[0x02] = PCI_DEVICE_NEON250 & 0xff;
    neon250->pci_regs[0x03] = (PCI_DEVICE_NEON250 >> 8) & 0xff;
    
    neon250->pci_regs[0x04] = neon250->pci_regs[0x04] & 0x37;
    neon250->pci_regs[0x05] = neon250->pci_regs[0x05];
    
    neon250->pci_regs[0x06] = 0x90;
    neon250->pci_regs[0x07] = 0x02;
    
    neon250->pci_regs[0x08] = 0x02; /* Revision ID */
    neon250->pci_regs[0x09] = 0x00; /* Programming interface */
    
    neon250->pci_regs[0x0a] = 0x00; /* Subclass - VGA compatible controller */
    neon250->pci_regs[0x0b] = 0x03; /* Class code - Display controller */
    
    neon250->pci_regs[0x0c] = 0x00; /* Cache line size */
    neon250->pci_regs[0x0d] = 0x20; /* Latency timer - 32 clocks */
    neon250->pci_regs[0x0e] = 0x00; /* Header type - normal */
    neon250->pci_regs[0x0f] = 0x00; /* BIST */
    
    /* PowerVR uses memory-mapped BARs */
    neon250->pci_regs[0x10] = 0x08; /* BAR 0 - 64MB aperture for framebuffer, memory mapped */
    neon250->pci_regs[0x11] = 0x00;
    neon250->pci_regs[0x12] = 0x00;
    neon250->pci_regs[0x13] = 0xa0;
    
    neon250->pci_regs[0x14] = 0x00; /* BAR 1 - 16KB MMIO registers */
    neon250->pci_regs[0x15] = 0x80;
    neon250->pci_regs[0x16] = 0x5f;
    neon250->pci_regs[0x17] = 0xa0;
    
    neon250->pci_regs[0x2c] = 0x00; /* Subsystem vendor ID */
    neon250->pci_regs[0x2d] = 0x10;
    neon250->pci_regs[0x2e] = 0x20; /* Subsystem ID */
    neon250->pci_regs[0x2f] = 0x01;
    
    neon250->pci_regs[0x30] = (neon250->pci_regs[0x30] & 0x01);; /* Expansion ROM base address */
    neon250->pci_regs[0x31] = 0x00;
    neon250->pci_regs[0x32] = neon250->pci_regs[0x32];
    neon250->pci_regs[0x33] = neon250->pci_regs[0x33];

    neon250->pci_regs[0x34] = 0x60;
    
    neon250->pci_regs[0x3c] = 0x01; /* Interrupt line - IRQ 1 */
    neon250->pci_regs[0x3d] = 0x01; /* Interrupt pin - INTA# */
    
    /* AGP specific registers */
    neon250->pci_regs[0x40] = 0x02; /* AGP 1.0 */
    neon250->pci_regs[0x41] = 0x00;
    neon250->pci_regs[0x42] = 0x10; /* AGP Capability */
    neon250->pci_regs[0x43] = 0x00;
    
    neon250->pci_regs[0x44] = 0x03; /* AGP Status */
    neon250->pci_regs[0x45] = 0x02; /* AGP 1x and 2x */
    neon250->pci_regs[0x46] = 0x00;
    neon250->pci_regs[0x47] = 0x1f; /* Maximum 31 requests */
    
    /* Power management capability */
    neon250->pci_regs[0x60] = 0x01; /* Capability ID */
    neon250->pci_regs[0x61] = 0x40; /* Next capability pointer */
    neon250->pci_regs[0x62] = 0x21; /* PM capability */
    neon250->pci_regs[0x63] = 0x06; /* PM control/status */
    
    video_inform(VIDEO_FLAG_TYPE_SPECIAL, &timing_neon250);
    
    /* Start with common 640x480 VGA mode */
    neon250->svga.seqregs[0x01] = 0x01; /* 8 dot mode */
    neon250->svga.seqregs[0x04] = 0x0e; /* Chain 4 */

    neon250->pvr_3d = pvr_3d_init(neon250);
    
    return neon250;
}

static void neon250_close(void *priv)
{
    neon250_t *neon250 = (neon250_t *)priv;
    
    /* Close the 3D pipeline */
    if (neon250->pvr_3d) {
        pvr_3d_close(neon250->pvr_3d);
        neon250->pvr_3d = NULL;
    }

    svga_close(&neon250->svga);
    free(neon250);
}

static void neon250_speed_changed(void *priv)
{
    neon250_t *neon250 = (neon250_t *)priv;
    
    svga_recalctimings(&neon250->svga);
}

static void neon250_force_redraw(void *priv)
{
    neon250_t *neon250 = (neon250_t *)priv;
    
    neon250->svga.fullchange = changeframecount;
}

static int neon250_available(void)
{
    return rom_present(BIOS_ROM_PATH);
}

static const device_config_t neon250_config[] = {
    {
        .name = "memory",
        .description = "Memory size",
        .type = CONFIG_SELECTION,
        .default_int = 16,
        .selection = {
            {
                .description = "8 MB",
                .value = 8
            },
            {
                .description = "16 MB",
                .value = 16
            },
            {
                .description = "32 MB",
                .value = 32
            },
            {
                .description = ""
            }
        }
    },
    {
        .type = CONFIG_END
    }
};

const device_t neon250_device = {
    .name = "VideoLogic PowerVR Neon 250",
    .internal_name = "pvr_neon250",
    .flags = DEVICE_AGP,
    .local = 0,
    .init = neon250_init,
    .close = neon250_close,
    .reset = NULL,
    .available = neon250_available,
    .speed_changed = neon250_speed_changed,
    .force_redraw = neon250_force_redraw,
    .config = neon250_config
};