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
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <86box/86box.h>
#include <86box/mem.h>
#include <86box/timer.h>
#include <86box/device.h>
#include <86box/plat.h>
#include <86box/video.h>
#include <86box/vid_svga.h>
#include <86box/vid_neon250_3d.h>
#include <86box/vid_neon250_reg.h>

/* Debug logging */
#define PVR_LOG(fmt, ...) pclog("PowerVR Neon 250: " fmt, ##__VA_ARGS__)
#define PVR_DEBUG(fmt, ...) do { if (neon250_debug) pclog("PowerVR Neon 250 DEBUG: " fmt, ##__VA_ARGS__); } while (0)

/* Debug flag - set to 1 to enable detailed register logging */
static int neon250_debug = 0;

/* 
 * PowerVR Register Sets:
 * 
 * 1. Core Control Registers
 * 2. Polygon Engine Registers
 * 3. Texture Processing Unit Registers
 * 4. Rendering Engine Registers
 * 5. PCI Interface Registers
 * 6. Video Output Registers
 * 7. DMA Controller Registers
 * 8. Interrupt Control Registers
 */

/* Register Space Definitions */
#define PVR_CORE_BASE      0x000000  /* 0x000000-0x000FFF: Core control */
#define PVR_POLY_BASE      0x001000  /* 0x001000-0x001FFF: Polygon engine */
#define PVR_TEX_BASE       0x002000  /* 0x002000-0x002FFF: Texture processing */
#define PVR_RENDER_BASE    0x003000  /* 0x003000-0x003FFF: Rendering engine */
#define PVR_PCI_BASE       0x004000  /* 0x004000-0x004FFF: PCI interface */
#define PVR_VIDEO_BASE     0x005000  /* 0x005000-0x005FFF: Video output */
#define PVR_DMA_BASE       0x006000  /* 0x006000-0x006FFF: DMA controller */
#define PVR_INT_BASE       0x007000  /* 0x007000-0x007FFF: Interrupt control */

/* Core Control Registers */
#define PVR_CORE_ID        0x000     /* Chip ID */
#define PVR_CORE_REVISION  0x004     /* Chip revision */
#define PVR_CORE_RESET     0x008     /* Soft reset control */
#define PVR_CORE_STATUS    0x00C     /* Current status */
#define PVR_CORE_CONFIG    0x010     /* Configuration */
#define PVR_CORE_MEMCFG    0x014     /* Memory configuration */
#define PVR_CORE_CLOCK     0x018     /* Clock control */
#define PVR_CORE_POWER     0x01C     /* Power management */

/* Polygon Engine Registers */
#define PVR_POLY_CONTROL   0x000     /* Geometry control */
#define PVR_POLY_STATUS    0x004     /* Geometry status */
#define PVR_POLY_VERTEX    0x008     /* Vertex data input */
#define PVR_POLY_NORMAL    0x00C     /* Normal data input */
#define PVR_POLY_COLOR     0x010     /* Color data input */
#define PVR_POLY_TEXCOORD  0x014     /* Texture coordinate input */
#define PVR_POLY_CLIP      0x018     /* Clipping control */
#define PVR_POLY_FOG       0x01C     /* Fog control */
#define PVR_POLY_LIGHTING  0x020     /* Lighting control */
#define PVR_POLY_CULLMODE  0x024     /* Backface culling mode */
#define PVR_POLY_CONTEXT   0x028     /* Polygon context */
#define PVR_POLY_LISTADDR  0x02C     /* Object list base address */
#define PVR_POLY_LISTSIZE  0x030     /* Object list size */

/* Texture Processing Unit Registers */
#define PVR_TEX_CONTROL    0x000     /* Texture control */
#define PVR_TEX_STATUS     0x004     /* Texture status */
#define PVR_TEX_ADDR       0x008     /* Texture address */
#define PVR_TEX_FORMAT     0x00C     /* Texture format */
#define PVR_TEX_FILTER     0x010     /* Texture filtering */
#define PVR_TEX_WRAP       0x014     /* Texture wrapping */
#define PVR_TEX_BORDER     0x018     /* Texture border color */
#define PVR_TEX_LOD        0x01C     /* Level of detail control */
#define PVR_TEX_CACHE      0x020     /* Texture cache control */
#define PVR_TEX_PALETTE    0x024     /* Palette control */
#define PVR_TEX_ENV        0x028     /* Texture environment */
#define PVR_TEX_TRANSPARENCY 0x02C   /* Transparency control */

/* Rendering Engine Registers */
#define PVR_RENDER_CONTROL  0x000    /* Rendering control */
#define PVR_RENDER_STATUS   0x004    /* Rendering status */
#define PVR_RENDER_ZBUFFER  0x008    /* Z-buffer control */
#define PVR_RENDER_BLEND    0x00C    /* Alpha blending */
#define PVR_RENDER_SHADE    0x010    /* Shading control */
#define PVR_RENDER_DITHER   0x014    /* Dithering control */
#define PVR_RENDER_TILECFG  0x018    /* Tile configuration */
#define PVR_RENDER_PIXFMT   0x01C    /* Output pixel format */
#define PVR_RENDER_OPFLAGS  0x020    /* Operation flags */
#define PVR_RENDER_ZBIASVAL 0x024    /* Z-bias value */
#define PVR_RENDER_FOGCOLOR 0x028    /* Fog color */
#define PVR_RENDER_FOGDIST  0x02C    /* Fog distance */

/* PCI Interface Registers */
#define PVR_PCI_STATUS      0x000    /* PCI status */
#define PVR_PCI_CONTROL     0x004    /* PCI control */
#define PVR_PCI_CONFIG      0x008    /* PCI configuration */
#define PVR_PCI_ARBITER     0x00C    /* Bus arbitration */
#define PVR_PCI_MEMWIN      0x010    /* Memory window control */
#define PVR_PCI_BUSMASTER   0x014    /* Bus master control */
#define PVR_PCI_TIMEOUT     0x018    /* Timeout values */

/* Video Output Registers */
#define PVR_VIDEO_CONTROL   0x000    /* Video control */
#define PVR_VIDEO_SYNC      0x004    /* Sync generation */
#define PVR_VIDEO_HPOS      0x008    /* Horizontal parameters */
#define PVR_VIDEO_VPOS      0x00C    /* Vertical parameters */
#define PVR_VIDEO_BORDER    0x010    /* Border color */
#define PVR_VIDEO_DISP      0x014    /* Display enable control */
#define PVR_VIDEO_FBADDR    0x018    /* Framebuffer address */
#define PVR_VIDEO_STRIDE    0x01C    /* Framebuffer stride */
#define PVR_VIDEO_SCALE     0x020    /* Scaling control */
#define PVR_VIDEO_OFFSET    0x024    /* Display offset */
#define PVR_VIDEO_COLORKEY  0x028    /* Color key */
#define PVR_VIDEO_CURSOR    0x02C    /* Cursor control */
#define PVR_VIDEO_CURSORPOS 0x030    /* Cursor position */
#define PVR_VIDEO_CURSORDATA 0x034   /* Cursor pattern */
#define PVR_VIDEO_PALETTE   0x038    /* Palette access */

/* DMA Controller Registers */
#define PVR_DMA_CONTROL     0x000    /* DMA control */
#define PVR_DMA_STATUS      0x004    /* DMA status */
#define PVR_DMA_SRC         0x008    /* Source address */
#define PVR_DMA_DEST        0x00C    /* Destination address */
#define PVR_DMA_COUNT       0x010    /* Transfer count */
#define PVR_DMA_NEXT        0x014    /* Next descriptor */
#define PVR_DMA_BURST       0x018    /* Burst mode control */
#define PVR_DMA_PRIORITY    0x01C    /* DMA priority */

/* Interrupt Control Registers */
#define PVR_INT_STATUS      0x000    /* Interrupt status */
#define PVR_INT_MASK        0x004    /* Interrupt mask */
#define PVR_INT_CLEAR       0x008    /* Interrupt clear */
#define PVR_INT_CONFIG      0x00C    /* Interrupt configuration */

/* Register value definitions */

/* Core ID and Revision */
#define PVR_CHIP_ID         0x004E4543  /* "NEC" */
#define PVR_CHIP_REVISION   0x00000100  /* Version 1.00 */

/* Core Reset bits */
#define PVR_RESET_CORE      (1 << 0)    /* Core reset */
#define PVR_RESET_GEO       (1 << 1)    /* Geometry engine reset */
#define PVR_RESET_TEX       (1 << 2)    /* Texture engine reset */
#define PVR_RESET_RENDER    (1 << 3)    /* Rendering engine reset */
#define PVR_RESET_VIDEO     (1 << 4)    /* Video output reset */
#define PVR_RESET_DMA       (1 << 5)    /* DMA controller reset */
#define PVR_RESET_ALL       0x0000003F  /* Reset all subsystems */

/* Core Status bits */
#define PVR_STAT_BUSY       (1 << 0)    /* Chip is busy */
#define PVR_STAT_GEO_BUSY   (1 << 1)    /* Geometry engine busy */
#define PVR_STAT_TEX_BUSY   (1 << 2)    /* Texture engine busy */
#define PVR_STAT_RENDER_BUSY (1 << 3)   /* Rendering engine busy */
#define PVR_STAT_DMA_BUSY   (1 << 4)    /* DMA controller busy */
#define PVR_STAT_VBLANK     (1 << 5)    /* In vertical blanking */
#define PVR_STAT_FIFO_EMPTY (1 << 6)    /* Command FIFO empty */
#define PVR_STAT_FIFO_FULL  (1 << 7)    /* Command FIFO full */
#define PVR_STAT_ERROR      (1 << 8)    /* Error condition */

/* Core Configuration bits */
#define PVR_CFG_TILE_SIZE_8  0x00000000  /* 8x8 tile size */
#define PVR_CFG_TILE_SIZE_16 0x00000001  /* 16x16 tile size */
#define PVR_CFG_TILE_SIZE_32 0x00000002  /* 32x32 tile size */
#define PVR_CFG_TILE_SIZE_64 0x00000003  /* 64x64 tile size */
#define PVR_CFG_FIFO_SIZE_256 0x00000000 /* 256 entry FIFO */
#define PVR_CFG_FIFO_SIZE_512 0x00000004 /* 512 entry FIFO */
#define PVR_CFG_FIFO_SIZE_1K  0x00000008 /* 1K entry FIFO */
#define PVR_CFG_FIFO_SIZE_2K  0x0000000C /* 2K entry FIFO */
#define PVR_CFG_SINGLE_PASS   0x00000000 /* Single pass rendering */
#define PVR_CFG_MULTI_PASS    0x00000010 /* Multi-pass rendering */
#define PVR_CFG_DITHER_OFF    0x00000000 /* Dithering disabled */
#define PVR_CFG_DITHER_ON     0x00000020 /* Dithering enabled */
#define PVR_CFG_TRIPLE_BUFFER 0x00000040 /* Triple buffering */
#define PVR_CFG_FAST_CLEAR    0x00000080 /* Fast clear enabled */

/* Polygon Control bits */
#define PVR_POLY_Z_ON        (1 << 0)    /* Z-buffer enabled */
#define PVR_POLY_TEXTURE     (1 << 1)    /* Texturing enabled */
#define PVR_POLY_BLEND       (1 << 2)    /* Alpha blending enabled */
#define PVR_POLY_GOURAUD     (1 << 3)    /* Gouraud shading */
#define PVR_POLY_FOG         (1 << 4)    /* Fog enabled */
#define PVR_POLY_ALPHA_TEST  (1 << 5)    /* Alpha testing enabled */
#define PVR_POLY_CULL_CW     (1 << 6)    /* Cull clockwise faces */
#define PVR_POLY_CULL_CCW    (1 << 7)    /* Cull counter-clockwise faces */
#define PVR_POLY_FRONT_CW    (1 << 8)    /* Front face is clockwise */
#define PVR_POLY_PERSPECTIVE (1 << 9)    /* Perspective correction */
#define PVR_POLY_UV_FLIP     (1 << 10)   /* Flip texture coordinates */
#define PVR_POLY_LIGHTING    (1 << 11)   /* Enable lighting */
#define PVR_POLY_SPECULAR    (1 << 12)   /* Enable specular lighting */

/* Texture Format bits */
#define PVR_TEXFMT_ARGB1555  0x00000000  /* ARGB1555 format */
#define PVR_TEXFMT_RGB565    0x00000001  /* RGB565 format */
#define PVR_TEXFMT_ARGB4444  0x00000002  /* ARGB4444 format */
#define PVR_TEXFMT_YUV422    0x00000003  /* YUV422 format */
#define PVR_TEXFMT_BUMP      0x00000004  /* Bump map format */
#define PVR_TEXFMT_PAL4BPP   0x00000005  /* 4bpp palettized */
#define PVR_TEXFMT_PAL8BPP   0x00000006  /* 8bpp palettized */
#define PVR_TEXFMT_ARGB8888  0x00000007  /* ARGB8888 format */
#define PVR_TEXFMT_SIZE_8    0x00000000  /* 8x8 texture size */
#define PVR_TEXFMT_SIZE_16   0x00000010  /* 16x16 texture size */
#define PVR_TEXFMT_SIZE_32   0x00000020  /* 32x32 texture size */
#define PVR_TEXFMT_SIZE_64   0x00000030  /* 64x64 texture size */
#define PVR_TEXFMT_SIZE_128  0x00000040  /* 128x128 texture size */
#define PVR_TEXFMT_SIZE_256  0x00000050  /* 256x256 texture size */
#define PVR_TEXFMT_SIZE_512  0x00000060  /* 512x512 texture size */
#define PVR_TEXFMT_SIZE_1024 0x00000070  /* 1024x1024 texture size */
#define PVR_TEXFMT_MIPMAP    0x00000100  /* Mipmapped texture */
#define PVR_TEXFMT_TWIDDLED  0x00000200  /* Twiddled texture */
#define PVR_TEXFMT_VQ        0x00000400  /* Vector quantized texture */
#define PVR_TEXFMT_STRIDE    0x00000800  /* Strided (non-twiddled) texture */

/* Texture Filter bits */
#define PVR_FILTER_POINT     0x00000000  /* Point sampling */
#define PVR_FILTER_BILINEAR  0x00000001  /* Bilinear filtering */
#define PVR_FILTER_TRILINEAR 0x00000002  /* Trilinear filtering */
#define PVR_FILTER_ANISO_2X  0x00000003  /* 2x anisotropic filtering */
#define PVR_FILTER_ANISO_4X  0x00000004  /* 4x anisotropic filtering */
#define PVR_FILTER_MIN_POINT 0x00000000  /* Point minification */
#define PVR_FILTER_MIN_LINEAR 0x00000010 /* Linear minification */
#define PVR_FILTER_MAG_POINT 0x00000000  /* Point magnification */
#define PVR_FILTER_MAG_LINEAR 0x00000100 /* Linear magnification */
#define PVR_FILTER_MIP_POINT 0x00000000  /* Point mipmap selection */
#define PVR_FILTER_MIP_LINEAR 0x00001000 /* Linear mipmap blending */

/* Texture Wrap bits */
#define PVR_WRAP_REPEAT      0x00000001  /* Repeat texture */
#define PVR_WRAP_CLAMP       0x00000002  /* Clamp texture */
#define PVR_WRAP_MIRROR      0x00000003  /* Mirror texture */
#define PVR_WRAP_U_SHIFT     0           /* U wrap mode shift */
#define PVR_WRAP_V_SHIFT     2           /* V wrap mode shift */

/* Render Control bits */
#define PVR_RENDER_START     (1 << 0)    /* Start rendering */
#define PVR_RENDER_ENABLE    (1 << 1)    /* Enable rendering */
#define PVR_RENDER_RESET     (1 << 2)    /* Reset rendering engine */
#define PVR_RENDER_OPAQUE    (1 << 3)    /* Render opaque polygons */
#define PVR_RENDER_TRANS     (1 << 4)    /* Render translucent polygons */
#define PVR_RENDER_PUNCHTHRU (1 << 5)    /* Render punch-through polygons */
#define PVR_RENDER_MODIFIER  (1 << 6)    /* Render modifier volumes */

/* Z-buffer Control bits */
#define PVR_Z_NEVER          0x00000000  /* Never pass Z test */
#define PVR_Z_LESS           0x00000001  /* Less than Z test */
#define PVR_Z_EQUAL          0x00000002  /* Equal Z test */
#define PVR_Z_LEQUAL         0x00000003  /* Less than or equal Z test */
#define PVR_Z_GREATER        0x00000004  /* Greater than Z test */
#define PVR_Z_NOTEQUAL       0x00000005  /* Not equal Z test */
#define PVR_Z_GEQUAL         0x00000006  /* Greater than or equal Z test */
#define PVR_Z_ALWAYS         0x00000007  /* Always pass Z test */
#define PVR_Z_WRITE          0x00000010  /* Enable Z writes */
#define PVR_Z_FULLINT        0x00000000  /* Full-resolution Z buffer */
#define PVR_Z_HALFINT        0x00000100  /* Half-resolution Z buffer */

/* Alpha Blend bits */
#define PVR_BLEND_ZERO       0x00000000  /* Zero */
#define PVR_BLEND_ONE        0x00000001  /* One */
#define PVR_BLEND_SRC_ALPHA  0x00000002  /* Source alpha */
#define PVR_BLEND_INV_SRC_ALPHA 0x00000003 /* Inverted source alpha */
#define PVR_BLEND_DST_ALPHA  0x00000004  /* Destination alpha */
#define PVR_BLEND_INV_DST_ALPHA 0x00000005 /* Inverted destination alpha */
#define PVR_BLEND_SRC_COLOR  0x00000006  /* Source color */
#define PVR_BLEND_INV_SRC_COLOR 0x00000007 /* Inverted source color */
#define PVR_BLEND_DST_COLOR  0x00000008  /* Destination color */
#define PVR_BLEND_INV_DST_COLOR 0x00000009 /* Inverted destination color */
#define PVR_BLEND_SRC_ALPHA_SAT 0x0000000A /* Source alpha saturate */
#define PVR_BLEND_SRC_SHIFT  0           /* Source blend factor shift */
#define PVR_BLEND_DST_SHIFT  4           /* Destination blend factor shift */

/* DMA Control bits */
#define PVR_DMA_START        (1 << 0)    /* Start DMA transfer */
#define PVR_DMA_ENABLE       (1 << 1)    /* Enable DMA */
#define PVR_DMA_RESET        (1 << 2)    /* Reset DMA controller */
#define PVR_DMA_SUSPEND      (1 << 3)    /* Suspend DMA */
#define PVR_DMA_TO_VRAM      (1 << 4)    /* Transfer to VRAM */
#define PVR_DMA_FROM_VRAM    (1 << 5)    /* Transfer from VRAM */
#define PVR_DMA_CHAIN        (1 << 6)    /* Chain mode enabled */
#define PVR_DMA_INTERRUPT    (1 << 7)    /* Generate interrupt on completion */

/* DMA Status bits */
#define PVR_DMA_BUSY         (1 << 0)    /* DMA transfer in progress */
#define PVR_DMA_DONE         (1 << 1)    /* DMA transfer complete */
#define PVR_DMA_SUSPENDED    (1 << 2)    /* DMA transfer suspended */
#define PVR_DMA_ERROR        (1 << 3)    /* DMA error occurred */

/* Interrupt Status/Mask/Clear bits */
#define PVR_INT_VBLANK       (1 << 0)    /* Vertical blanking interrupt */
#define PVR_INT_RENDER_DONE  (1 << 1)    /* Rendering complete interrupt */
#define PVR_INT_DMA_DONE     (1 << 2)    /* DMA complete interrupt */
#define PVR_INT_ERROR        (1 << 3)    /* Error interrupt */
#define PVR_INT_PCI          (1 << 4)    /* PCI interrupt */
#define PVR_INT_FIFO_OVER    (1 << 5)    /* FIFO overflow */
#define PVR_INT_FIFO_UNDER   (1 << 6)    /* FIFO underflow */
#define PVR_INT_MASTER       (1 << 31)   /* Master interrupt enable */

/* Register implementation structures */

/* Complete register space for PowerVR Neon 250 */
typedef struct pvr_reg_space_t {
    uint32_t core[1024];     /* Core control registers */
    uint32_t poly[1024];     /* Polygon engine registers */
    uint32_t tex[1024];      /* Texture engine registers */
    uint32_t render[1024];   /* Rendering engine registers */
    uint32_t pci[1024];      /* PCI interface registers */
    uint32_t video[1024];    /* Video output registers */
    uint32_t dma[1024];      /* DMA controller registers */
    uint32_t interrupt[1024]; /* Interrupt controller registers */
} pvr_reg_space_t;

/* Initialize the register space with default values */
void pvr_reg_init(pvr_3d_state_t* state) {
    pvr_reg_space_t* regs = (pvr_reg_space_t*)calloc(1, sizeof(pvr_reg_space_t));
    
    if (!regs) {
        PVR_LOG("Failed to allocate register space memory\n");
        return;
    }
    
    /* Store the register space in the 3D state */
    state->registers = regs;
    
    /* Initialize core registers with default values */
    regs->core[PVR_CORE_ID / 4] = PVR_CHIP_ID;
    regs->core[PVR_CORE_REVISION / 4] = PVR_CHIP_REVISION;
    regs->core[PVR_CORE_STATUS / 4] = PVR_STAT_FIFO_EMPTY;
    regs->core[PVR_CORE_CONFIG / 4] = PVR_CFG_TILE_SIZE_32 | PVR_CFG_FIFO_SIZE_1K;
    
    /* Initialize polygon engine registers */
    regs->poly[PVR_POLY_CONTROL / 4] = 0; /* No features enabled by default */
    
    /* Initialize texture engine registers */
    regs->tex[PVR_TEX_FORMAT / 4] = PVR_TEXFMT_ARGB1555 | PVR_TEXFMT_SIZE_256;
    regs->tex[PVR_TEX_FILTER / 4] = PVR_FILTER_BILINEAR;
    regs->tex[PVR_TEX_WRAP / 4] = (PVR_WRAP_REPEAT << PVR_WRAP_U_SHIFT) | 
                                 (PVR_WRAP_REPEAT << PVR_WRAP_V_SHIFT);
    
    /* Initialize rendering engine registers */
    regs->render[PVR_RENDER_ZBUFFER / 4] = PVR_Z_LESS | PVR_Z_WRITE | PVR_Z_FULLINT;
    regs->render[PVR_RENDER_BLEND / 4] = (PVR_BLEND_SRC_ALPHA << PVR_BLEND_SRC_SHIFT) | 
                                        (PVR_BLEND_INV_SRC_ALPHA << PVR_BLEND_DST_SHIFT);
    
    /* Initialize DMA controller registers */
    regs->dma[PVR_DMA_CONTROL / 4] = 0; /* DMA disabled by default */
    
    /* Initialize interrupt controller registers */
    regs->interrupt[PVR_INT_STATUS / 4] = 0; /* No pending interrupts */
    regs->interrupt[PVR_INT_MASK / 4] = 0;   /* All interrupts masked */
    
    PVR_LOG("Register space initialized\n");
}

/* Free the register space memory */
void pvr_reg_close(pvr_3d_state_t* state) {
    if (state->registers) {
        free(state->registers);
        state->registers = NULL;
    }
}

/* Reset all registers to their default values */
void pvr_reg_reset(pvr_3d_state_t* state) {
    pvr_reg_space_t* regs = (pvr_reg_space_t*)state->registers;
    
    if (!regs) {
        PVR_LOG("Register space not allocated\n");
        return;
    }
    
    /* Reset all register blocks */
    memset(regs->core, 0, 1024 * sizeof(uint32_t));
    memset(regs->poly, 0, 1024 * sizeof(uint32_t));
    memset(regs->tex, 0, 1024 * sizeof(uint32_t));
    memset(regs->render, 0, 1024 * sizeof(uint32_t));
    memset(regs->pci, 0, 1024 * sizeof(uint32_t));
    memset(regs->video, 0, 1024 * sizeof(uint32_t));
    memset(regs->dma, 0, 1024 * sizeof(uint32_t));
    memset(regs->interrupt, 0, 1024 * sizeof(uint32_t));
    
    /* Re-initialize with default values */
    regs->core[PVR_CORE_ID / 4] = PVR_CHIP_ID;
    regs->core[PVR_CORE_REVISION / 4] = PVR_CHIP_REVISION;
    regs->core[PVR_CORE_STATUS / 4] = PVR_STAT_FIFO_EMPTY;
    regs->core[PVR_CORE_CONFIG / 4] = PVR_CFG_TILE_SIZE_32 | PVR_CFG_FIFO_SIZE_1K;
    
    regs->poly[PVR_POLY_CONTROL / 4] = 0;
    
    regs->tex[PVR_TEX_FORMAT / 4] = PVR_TEXFMT_ARGB1555 | PVR_TEXFMT_SIZE_256;
    regs->tex[PVR_TEX_FILTER / 4] = PVR_FILTER_BILINEAR;
    regs->tex[PVR_TEX_WRAP / 4] = (PVR_WRAP_REPEAT << PVR_WRAP_U_SHIFT) | 
                                 (PVR_WRAP_REPEAT << PVR_WRAP_V_SHIFT);
    
    regs->render[PVR_RENDER_ZBUFFER / 4] = PVR_Z_LESS | PVR_Z_WRITE | PVR_Z_FULLINT;
    regs->render[PVR_RENDER_BLEND / 4] = (PVR_BLEND_SRC_ALPHA << PVR_BLEND_SRC_SHIFT) | 
                                        (PVR_BLEND_INV_SRC_ALPHA << PVR_BLEND_DST_SHIFT);
    
    regs->dma[PVR_DMA_CONTROL / 4] = 0;
    
    regs->interrupt[PVR_INT_STATUS / 4] = 0;
    regs->interrupt[PVR_INT_MASK / 4] = 0;
    
    PVR_LOG("Registers reset to default values\n");
}

/* Write to a register */
void pvr_reg_write(pvr_3d_state_t* state, uint32_t addr, uint32_t val) {
    pvr_reg_space_t* regs = (pvr_reg_space_t*)state->registers;
    uint32_t reg_set = (addr >> 12) & 0xF;
    uint32_t reg_addr = addr & 0xFFF;
    uint32_t reg_index = reg_addr / 4;
    
    /* Validate address */
    if (reg_addr & 3) {
        PVR_LOG("Unaligned register write at %08X = %08X\n", addr, val);
        return;
    }
    
    if (neon250_debug) {
        PVR_DEBUG("Register write: [%08X] = %08X\n", addr, val);
    }
    
    /* Process register write based on register set */
    switch (reg_set) {
        case 0: /* Core Control Registers */
            switch (reg_addr) {
                case PVR_CORE_RESET:
                    /* Handle reset request */
                    if (val & PVR_RESET_CORE) {
                        PVR_LOG("Core reset requested\n");
                        pvr_reg_reset(state);
                        pvr_3d_reset(state);
                    } else {
                        /* Store the reset value anyway */
                        regs->core[reg_index] = val;
                        
                        /* Partial reset of subsystems */
                        if (val & PVR_RESET_GEO) {
                            PVR_LOG("Geometry engine reset\n");
                            memset(regs->poly, 0, 1024 * sizeof(uint32_t));
                        }
                        if (val & PVR_RESET_TEX) {
                            PVR_LOG("Texture engine reset\n");
                            memset(regs->tex, 0, 1024 * sizeof(uint32_t));
                        }
                        if (val & PVR_RESET_RENDER) {
                            PVR_LOG("Rendering engine reset\n");
                            memset(regs->render, 0, 1024 * sizeof(uint32_t));
                        }
                        if (val & PVR_RESET_DMA) {
                            PVR_LOG("DMA controller reset\n");
                            memset(regs->dma, 0, 1024 * sizeof(uint32_t));
                        }
                    }
                    break;
                    
                case PVR_CORE_STATUS:
                    /* Status register is mostly read-only */
                    PVR_LOG("Attempted write to read-only status register: %08X\n", val);
                    break;
                    
                case PVR_CORE_CONFIG:
                    PVR_LOG("Core configuration set to %08X\n", val);
                    regs->core[reg_index] = val;
                    /* Update tile size based on configuration */
                    switch (val & 0x3) {
                        case PVR_CFG_TILE_SIZE_8:
                            state->tile_size = 8;
                            break;
                        case PVR_CFG_TILE_SIZE_16:
                            state->tile_size = 16;
                            break;
                        case PVR_CFG_TILE_SIZE_32:
                            state->tile_size = 32;
                            break;
                        case PVR_CFG_TILE_SIZE_64:
                            state->tile_size = 64;
                            break;
                    }
                    PVR_LOG("Tile size set to %dx%d\n", state->tile_size, state->tile_size);
                    break;
                    
                default:
                    /* Store value for other core registers */
                    regs->core[reg_index] = val;
                    break;
            }
            break;
            
        case 1: /* Polygon Engine Registers */
            switch (reg_addr) {
                case PVR_POLY_CONTROL:
                    regs->poly[reg_index] = val;
                    PVR_DEBUG("Polygon control set to %08X\n", val);
                    /* Update polygon engine state */
                    state->poly_control = val;
                    break;
                    
                case PVR_POLY_VERTEX:
                    regs->poly[reg_index] = val;
                    /* Add vertex data to the command queue */
                    pvr_3d_process_command(state, 0x01, val);
                    break;
                    
                case PVR_POLY_NORMAL:
                    regs->poly[reg_index] = val;
                    /* Add normal data to the command queue */
                    pvr_3d_process_command(state, 0x03, val);
                    break;
                    
                case PVR_POLY_COLOR:
                    regs->poly[reg_index] = val;
                    /* Add color data to the command queue */
                    pvr_3d_process_command(state, 0x04, val);
                    break;
                    
                case PVR_POLY_TEXCOORD:
                    regs->poly[reg_index] = val;
                    /* Add texture coordinate data to the command queue */
                    pvr_3d_process_command(state, 0x02, val);
                    break;
                    
                default:
                    /* Store value for other polygon registers */
                    regs->poly[reg_index] = val;
                    break;
            }
            break;
            
        case 2: /* Texture Processing Unit Registers */
            switch (reg_addr) {
                case PVR_TEX_CONTROL:
                    regs->tex[reg_index] = val;
                    state->tex_control = val;
                    PVR_DEBUG("Texture control set to %08X\n", val);
                    break;
                    
                case PVR_TEX_ADDR:
                    regs->tex[reg_index] = val;
                    state->tex_addr = val;
                    PVR_DEBUG("Texture address set to %08X\n", val);
                    break;
                    
                case PVR_TEX_FORMAT:
                    regs->tex[reg_index] = val;
                    state->tex_config = val;
                    PVR_DEBUG("Texture format set to %08X\n", val);
                    
                    /* Update texture state based on format */
                    uint32_t format = val & 0xF;
                    uint32_t size_code = (val >> 4) & 0xF;
                    uint32_t width, height;
                    
                    /* Determine texture dimensions */
                    switch (size_code) {
                        case 0: width = height = 8; break;
                        case 1: width = height = 16; break;
                        case 2: width = height = 32; break;
                        case 3: width = height = 64; break;
                        case 4: width = height = 128; break;
                        case 5: width = height = 256; break;
                        case 6: width = height = 512; break;
                        case 7: width = height = 1024; break;
                        default: width = height = 256; break;
                    }
                    
                    /* Update current texture */
                    state->textures[state->current_texture].width = width;
                    state->textures[state->current_texture].height = height;
                    state->textures[state->current_texture].format = format;
                    
                    PVR_DEBUG("Texture size: %dx%d, format: %d\n", 
                             width, height, format);
                    break;
                    
                case PVR_TEX_FILTER:
                    regs->tex[reg_index] = val;
                    state->tex_filter = val;
                    PVR_DEBUG("Texture filter set to %08X\n", val);
                    break;
                    
                default:
                    /* Store value for other texture registers */
                    regs->tex[reg_index] = val;
                    break;
            }
            break;
            
        case 3: /* Rendering Engine Registers */
            switch (reg_addr) {
                case PVR_RENDER_CONTROL:
                    regs->render[reg_index] = val;
                    state->render_control = val;
                    
                    /* Check if rendering should start */
                    if (val & PVR_RENDER_START) {
                        PVR_LOG("Starting 3D frame rendering\n");
                        /* Set status flags */
                        regs->core[PVR_CORE_STATUS / 4] |= PVR_STAT_BUSY | PVR_STAT_RENDER_BUSY;
                        
                        /* Start the rendering process */
                        pvr_3d_process_command(state, 0x10, 0);
                    }
                    
                    /* Check if rendering should be reset */
                    if (val & PVR_RENDER_RESET) {
                        PVR_LOG("Resetting render engine\n");
                        /* Clear rendering state */
                        memset(regs->render, 0, 1024 * sizeof(uint32_t));
                        regs->render[reg_index] = val & ~PVR_RENDER_RESET;
                        regs->core[PVR_CORE_STATUS / 4] &= ~(PVR_STAT_BUSY | PVR_STAT_RENDER_BUSY);
                    }
                    break;
                    
                case PVR_RENDER_ZBUFFER:
                    regs->render[reg_index] = val;
                    state->z_compare = val;
                    PVR_DEBUG("Z-buffer control set to %08X\n", val);
                    break;
                    
                case PVR_RENDER_BLEND:
                    regs->render[reg_index] = val;
                    state->blend_mode = val;
                    PVR_DEBUG("Alpha blend control set to %08X\n", val);
                    break;
                    
                default:
                    /* Store value for other rendering registers */
                    regs->render[reg_index] = val;
                    break;
            }
            break;
            
        case 4: /* PCI Interface Registers */
            /* Store all PCI registers */
            regs->pci[reg_index] = val;
            break;
            
        case 5: /* Video Output Registers */
            switch (reg_addr) {
                case PVR_VIDEO_CONTROL:
                    regs->video[reg_index] = val;
                    PVR_DEBUG("Video control set to %08X\n", val);
                    break;
                    
                case PVR_VIDEO_FBADDR:
                    regs->video[reg_index] = val;
                    state->disp_start = val;
                    PVR_DEBUG("Framebuffer address set to %08X\n", val);
                    break;
                    
                case PVR_VIDEO_STRIDE:
                    regs->video[reg_index] = val;
                    PVR_DEBUG("Framebuffer stride set to %08X\n", val);
                    /* Update stride if different from current SVGA stride */
                    if (val != state->fb_stride) {
                        state->fb_stride = val;
                    }
                    break;
                    
                default:
                    /* Store value for other video registers */
                    regs->video[reg_index] = val;
                    break;
            }
            break;
            
        case 6: /* DMA Controller Registers */
            switch (reg_addr) {
                case PVR_DMA_CONTROL:
                    regs->dma[reg_index] = val;
                    state->dma_control = val;
                    
                    /* Check if DMA should start */
                    if (val & PVR_DMA_START) {
                        PVR_LOG("Starting DMA transfer\n");
                        /* Set status flags */
                        regs->dma[PVR_DMA_STATUS / 4] |= PVR_DMA_BUSY;
                        regs->core[PVR_CORE_STATUS / 4] |= PVR_STAT_DMA_BUSY;
                        
                        /* Start the DMA transfer */
                        pvr_3d_dma_transfer(state);
                    }
                    
                    /* Check if DMA should be reset */
                    if (val & PVR_DMA_RESET) {
                        PVR_LOG("Resetting DMA controller\n");
                        /* Clear DMA state */
                        memset(regs->dma, 0, 1024 * sizeof(uint32_t));
                        regs->dma[reg_index] = val & ~PVR_DMA_RESET;
                        regs->dma[PVR_DMA_STATUS / 4] &= ~PVR_DMA_BUSY;
                        regs->core[PVR_CORE_STATUS / 4] &= ~PVR_STAT_DMA_BUSY;
                    }
                    break;
                    
                case PVR_DMA_SRC:
                    regs->dma[reg_index] = val;
                    state->dma_src_addr = val;
                    PVR_DEBUG("DMA source address set to %08X\n", val);
                    break;
                    
                case PVR_DMA_DEST:
                    regs->dma[reg_index] = val;
                    state->dma_dest_addr = val;
                    PVR_DEBUG("DMA destination address set to %08X\n", val);
                    break;
                    
                case PVR_DMA_COUNT:
                    regs->dma[reg_index] = val;
                    state->dma_size = val;
                    PVR_DEBUG("DMA transfer size set to %08X\n", val);
                    break;
                    
                default:
                    /* Store value for other DMA registers */
                    regs->dma[reg_index] = val;
                    break;
            }
            break;
            
        case 7: /* Interrupt Controller Registers */
            switch (reg_addr) {
                case PVR_INT_MASK:
                    regs->interrupt[reg_index] = val;
                    PVR_DEBUG("Interrupt mask set to %08X\n", val);
                    break;
                    
                case PVR_INT_CLEAR:
                    /* Clear the specified interrupt flags */
                    regs->interrupt[PVR_INT_STATUS / 4] &= ~val;
                    PVR_DEBUG("Cleared interrupt flags: %08X\n", val);
                    break;
                    
                default:
                    /* Store value for other interrupt registers */
                    regs->interrupt[reg_index] = val;
                    break;
            }
            break;
            
        default:
            PVR_LOG("Write to invalid register set %d, addr %08X = %08X\n", 
                    reg_set, reg_addr, val);
            break;
    }
}

/* Read from a register */
uint32_t pvr_reg_read(pvr_3d_state_t* state, uint32_t addr) {
    pvr_reg_space_t* regs = (pvr_reg_space_t*)state->registers;
    uint32_t reg_set = (addr >> 12) & 0xF;
    uint32_t reg_addr = addr & 0xFFF;
    uint32_t reg_index = reg_addr / 4;
    uint32_t val = 0xFFFFFFFF; /* Default to all bits set for undefined registers */
    
    /* Validate address */
    if (reg_addr & 3) {
        PVR_LOG("Unaligned register read at %08X\n", addr);
        return 0xFFFFFFFF;
    }
    
    /* Read from appropriate register set */
    switch (reg_set) {
        case 0: /* Core Control Registers */
            /* Special handling for status register to reflect current state */
            if (reg_addr == PVR_CORE_STATUS) {
                /* Update VBLANK status based on display state */
                if (state->neon250->svga.cgastat & 8) {
                    regs->core[reg_index] |= PVR_STAT_VBLANK;
                } else {
                    regs->core[reg_index] &= ~PVR_STAT_VBLANK;
                }
            }
            val = regs->core[reg_index];
            break;
            
        case 1: /* Polygon Engine Registers */
            val = regs->poly[reg_index];
            break;
            
        case 2: /* Texture Processing Unit Registers */
            val = regs->tex[reg_index];
            break;
            
        case 3: /* Rendering Engine Registers */
            val = regs->render[reg_index];
            break;
            
        case 4: /* PCI Interface Registers */
            val = regs->pci[reg_index];
            break;
            
        case 5: /* Video Output Registers */
            val = regs->video[reg_index];
            break;
            
        case 6: /* DMA Controller Registers */
            val = regs->dma[reg_index];
            break;
            
        case 7: /* Interrupt Controller Registers */
            val = regs->interrupt[reg_index];
            break;
            
        default:
            PVR_LOG("Read from invalid register set %d, addr %08X\n", reg_set, reg_addr);
            break;
    }
    
    if (neon250_debug) {
        PVR_DEBUG("Register read: [%08X] = %08X\n", addr, val);
    }
    
    return val;
}

/* Called when rendering completes to update status registers */
void pvr_reg_render_complete(pvr_3d_state_t* state) {
    pvr_reg_space_t* regs = (pvr_reg_space_t*)state->registers;
    
    /* Clear busy flags */
    regs->core[PVR_CORE_STATUS / 4] &= ~(PVR_STAT_BUSY | PVR_STAT_RENDER_BUSY);
    
    /* Set render complete flag in render status */
    regs->render[PVR_RENDER_STATUS / 4] |= 1;
    
    /* Generate interrupt if enabled */
    if (regs->interrupt[PVR_INT_MASK / 4] & PVR_INT_RENDER_DONE) {
        regs->interrupt[PVR_INT_STATUS / 4] |= PVR_INT_RENDER_DONE;
        /* TODO: Trigger physical interrupt if needed */
    }
}

/* Called when DMA completes to update status registers */
void pvr_reg_dma_complete(pvr_3d_state_t* state) {
    pvr_reg_space_t* regs = (pvr_reg_space_t*)state->registers;
    
    /* Clear busy flags */
    regs->dma[PVR_DMA_STATUS / 4] &= ~PVR_DMA_BUSY;
    regs->dma[PVR_DMA_STATUS / 4] |= PVR_DMA_DONE;
    regs->core[PVR_CORE_STATUS / 4] &= ~PVR_STAT_DMA_BUSY;
    
    /* Generate interrupt if enabled */
    if (regs->interrupt[PVR_INT_MASK / 4] & PVR_INT_DMA_DONE) {
        regs->interrupt[PVR_INT_STATUS / 4] |= PVR_INT_DMA_DONE;
        /* TODO: Trigger physical interrupt if needed */
    }
}

/* Called at VBlank to update status and possibly trigger interrupt */
void pvr_reg_vblank(pvr_3d_state_t* state) {
    pvr_reg_space_t* regs = (pvr_reg_space_t*)state->registers;
    
    /* Set VBLANK bit in status register */
    regs->core[PVR_CORE_STATUS / 4] |= PVR_STAT_VBLANK;
    
    /* Generate interrupt if enabled */
    if (regs->interrupt[PVR_INT_MASK / 4] & PVR_INT_VBLANK) {
        regs->interrupt[PVR_INT_STATUS / 4] |= PVR_INT_VBLANK;
        /* TODO: Trigger physical interrupt if needed */
    }
}

/* Register interface updates for vid_neon250_3d.h */