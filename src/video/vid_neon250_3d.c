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

/* PowerVR 3D pipeline registers */
#define PVR_REG_BASE             0x00100000

/* 3D Engine Registers */
#define PVR_3D_RESET             0x00
#define PVR_3D_STATUS            0x04
#define PVR_3D_CONFIG            0x08

/* Polygon Setup Registers */
#define PVR_POLY_BASE            0x00101000
#define PVR_POLY_VERTEX_CMD      0x00
#define PVR_POLY_TEXTURE_CMD     0x04
#define PVR_POLY_CONTROL         0x08
#define PVR_POLY_STATUS          0x0C

/* Texture Mapping Unit Registers */
#define PVR_TEX_BASE             0x00102000
#define PVR_TEX_CONTROL          0x00
#define PVR_TEX_ADDR             0x04
#define PVR_TEX_CONFIG           0x08
#define PVR_TEX_FILTER           0x0C

/* Rendering Engine Registers */
#define PVR_RENDER_BASE          0x00103000
#define PVR_RENDER_CONTROL       0x00
#define PVR_RENDER_STATUS        0x04
#define PVR_RENDER_Z_COMPARE     0x08
#define PVR_RENDER_BLEND         0x0C

/* DMA Controller Registers */
#define PVR_DMA_BASE             0x00104000
#define PVR_DMA_CONTROL          0x00
#define PVR_DMA_SRC_ADDR         0x04
#define PVR_DMA_DEST_ADDR        0x08
#define PVR_DMA_SIZE             0x0C
#define PVR_DMA_STATUS           0x10

/* 3D Status Register bits */
#define PVR_3D_STAT_RUNNING      (1 << 0)
#define PVR_3D_STAT_BUSY         (1 << 1)
#define PVR_3D_STAT_VBLANK       (1 << 2)
#define PVR_3D_STAT_FIFO_EMPTY   (1 << 3)
#define PVR_3D_STAT_FIFO_FULL    (1 << 4)
#define PVR_3D_STAT_DMA_ACTIVE   (1 << 5)

/* Polygon Control Register bits */
#define PVR_POLY_CTRL_ZBUFFER    (1 << 0)
#define PVR_POLY_CTRL_TEXTURE    (1 << 1)
#define PVR_POLY_CTRL_BLEND      (1 << 2)
#define PVR_POLY_CTRL_GOURAUD    (1 << 3)
#define PVR_POLY_CTRL_FOG        (1 << 4)
#define PVR_POLY_CTRL_ALPHA_TEST (1 << 5)
#define PVR_POLY_CTRL_CULL_CW    (1 << 6)
#define PVR_POLY_CTRL_CULL_CCW   (1 << 7)

/* Pipeline Constants */
#define PVR_MAX_VERTICES         2048
#define PVR_MAX_POLYGONS         2048
#define PVR_MAX_TEXTURES         256
#define PVR_FIFO_SIZE            4096
#define PVR_CMDLIST_SIZE         8192

typedef struct {
    float x, y, z, w;         /* Position */
    float r, g, b, a;         /* Color */
    float u, v;               /* Texture coordinates */
} pvr_vertex_t;

typedef struct {
    int               num_vertices;
    pvr_vertex_t      vertices[3];  /* Triangle vertices */
    uint32_t          texture_addr;
    uint32_t          control_flags;
    uint32_t          z_sort_value; /* For tile-based rendering */
} pvr_polygon_t;

typedef struct {
    uint32_t           width;
    uint32_t           height;
    uint32_t           format;
    uint32_t           addr;
    uint8_t*           data;
} pvr_texture_t;

typedef struct {
    uint32_t           command;
    uint32_t           data;
} pvr_fifo_entry_t;

/* The Neon 250 uses a tile-based deferred rendering approach */
typedef struct {
    int                x, y;
    int                width, height;
    pvr_polygon_t**    polygon_list; /* List of polygons affecting this tile */
    int                num_polygons;
    int                max_polygons;
} pvr_tile_t;

typedef struct {
    /* Current pipeline state */
    uint32_t           control_reg;
    uint32_t           status_reg;
    uint32_t           config_reg;
    
    /* Polygon setup state */
    uint32_t           poly_control;
    uint32_t           poly_status;
    int                current_vertex;
    pvr_vertex_t       vertex_buffer[PVR_MAX_VERTICES];
    pvr_polygon_t      polygon_buffer[PVR_MAX_POLYGONS];
    int                num_polygons;
    
    /* Texture mapping state */
    uint32_t           tex_control;
    uint32_t           tex_addr;
    uint32_t           tex_config;
    uint32_t           tex_filter;
    pvr_texture_t      textures[PVR_MAX_TEXTURES];
    int                current_texture;
    
    /* Rendering state */
    uint32_t           render_control;
    uint32_t           render_status;
    uint32_t           z_compare;
    uint32_t           blend_mode;
    
    /* Command FIFO */
    pvr_fifo_entry_t   fifo[PVR_FIFO_SIZE];
    int                fifo_read_ptr;
    int                fifo_write_ptr;
    int                fifo_entries;
    
    /* Tile-based rendering structures */
    pvr_tile_t**       tiles;
    int                num_tiles_x;
    int                num_tiles_y;
    int                tile_size;
    
    /* DMA */
    uint32_t           dma_control;
    uint32_t           dma_src_addr;
    uint32_t           dma_dest_addr;
    uint32_t           dma_size;
    uint32_t           dma_status;
    
    /* Framebuffer related */
    uint8_t*           framebuffer;
    uint32_t           fb_width;
    uint32_t           fb_height;
    uint32_t           fb_stride;
    uint32_t           fb_format;
    
    /* Z-buffer */
    uint16_t*          z_buffer;
    
    /* Timer for emulating rendering time */
    pc_timer_t         render_timer;

    void*              registers;    /* Register space (pvr_reg_space_t*) */
    
    /* Reference to parent neon250 device */
    struct neon250_t*  neon250;
} pvr_3d_state_t;

/* Forward declarations */
static void pvr_3d_reset(pvr_3d_state_t* state);
static void pvr_3d_process_command(pvr_3d_state_t* state, uint32_t command, uint32_t data);
static void pvr_3d_push_fifo(pvr_3d_state_t* state, uint32_t command, uint32_t data);
static void pvr_3d_process_fifo(pvr_3d_state_t* state);
static void pvr_3d_setup_polygon(pvr_3d_state_t* state);
static void pvr_3d_render_tile(pvr_3d_state_t* state, pvr_tile_t* tile);
static void pvr_3d_render_polygon(pvr_3d_state_t* state, pvr_polygon_t* poly);
static void pvr_3d_draw_triangle(pvr_3d_state_t* state, pvr_polygon_t* poly);
static void pvr_3d_timer_callback(void* priv);
static void pvr_3d_dma_transfer(pvr_3d_state_t* state);
static void pvr_3d_distribute_to_tiles(pvr_3d_state_t* state, pvr_polygon_t* poly);
static void pvr_3d_render_scene(pvr_3d_state_t* state);

/* Creates and initializes the 3D pipeline state */
pvr_3d_state_t* pvr_3d_init(struct neon250_t* neon250) {
    pvr_3d_state_t* state = calloc(1, sizeof(pvr_3d_state_t));
    
    if (!state)
        return NULL;
    
    state->neon250 = neon250;
    
    /* Initialize register space */
    pvr_reg_init(state);

    /* Reset to initial state */
    pvr_3d_reset(state);
    
    /* Initialize the rendering timer */
    timer_add(&state->render_timer, pvr_3d_timer_callback, state, 0);
    
    /* Set initial framebuffer parameters based on current SVGA state */
    state->fb_width = neon250->svga.hdisp;
    state->fb_height = neon250->svga.dispend;
    state->fb_stride = neon250->svga.rowoffset;
    state->fb_format = neon250->svga.bpp;
    state->framebuffer = neon250->svga.vram;
    
    /* Allocate Z-buffer */
    state->z_buffer = calloc(state->fb_width * state->fb_height, sizeof(uint16_t));
    
    /* Set up tiled rendering - PowerVR typically used 32x32 pixel tiles */
    state->tile_size = 32;
    state->num_tiles_x = (state->fb_width + state->tile_size - 1) / state->tile_size;
    state->num_tiles_y = (state->fb_height + state->tile_size - 1) / state->tile_size;
    
    /* Allocate tile array */
    state->tiles = calloc(state->num_tiles_x * state->num_tiles_y, sizeof(pvr_tile_t*));
    
    /* Initialize tiles */
    for (int y = 0; y < state->num_tiles_y; y++) {
        for (int x = 0; x < state->num_tiles_x; x++) {
            int tile_idx = y * state->num_tiles_x + x;
            state->tiles[tile_idx] = calloc(1, sizeof(pvr_tile_t));
            
            pvr_tile_t* tile = state->tiles[tile_idx];
            tile->x = x * state->tile_size;
            tile->y = y * state->tile_size;
            tile->width = state->tile_size;
            tile->height = state->tile_size;
            
            /* Handle edge tiles */
            if (tile->x + tile->width > state->fb_width)
                tile->width = state->fb_width - tile->x;
                
            if (tile->y + tile->height > state->fb_height)
                tile->height = state->fb_height - tile->y;
                
            /* Allocate initial polygon list for this tile */
            tile->max_polygons = 64; /* Start with space for 64 polygons */
            tile->polygon_list = calloc(tile->max_polygons, sizeof(pvr_polygon_t*));
            tile->num_polygons = 0;
        }
    }
    
    pclog("PowerVR Neon 250: 3D pipeline initialized with %dx%d framebuffer, %dx%d tiles\n", 
          state->fb_width, state->fb_height, state->num_tiles_x, state->num_tiles_y);
          
    return state;
}

/* Resets the 3D pipeline to initial state */
static void pvr_3d_reset(pvr_3d_state_t* state) {
    state->control_reg = 0;
    state->status_reg = PVR_3D_STAT_FIFO_EMPTY;
    state->config_reg = 0;
    
    state->poly_control = 0;
    state->poly_status = 0;
    state->current_vertex = 0;
    state->num_polygons = 0;
    
    state->tex_control = 0;
    state->tex_addr = 0;
    state->tex_config = 0;
    state->tex_filter = 0;
    state->current_texture = 0;
    
    state->render_control = 0;
    state->render_status = 0;
    state->z_compare = 0;
    state->blend_mode = 0;
    
    state->fifo_read_ptr = 0;
    state->fifo_write_ptr = 0;
    state->fifo_entries = 0;
    
    state->dma_control = 0;
    state->dma_src_addr = 0;
    state->dma_dest_addr = 0;
    state->dma_size = 0;
    state->dma_status = 0;

    /* Reset registers */
    pvr_reg_reset(state);
    
    /* Clear the tile polygon lists */
    if (state->tiles) {
        for (int i = 0; i < state->num_tiles_x * state->num_tiles_y; i++) {
            if (state->tiles[i]) {
                state->tiles[i]->num_polygons = 0;
            }
        }
    }
}

/* Free resources used by the 3D pipeline */
void pvr_3d_close(pvr_3d_state_t* state) {
    if (!state)
        return;
    
    /* Free register space */
    pvr_reg_close(state);

    /* Free the Z-buffer */
    if (state->z_buffer) {
        free(state->z_buffer);
        state->z_buffer = NULL;
    }
    
    /* Free the tiles */
    if (state->tiles) {
        for (int i = 0; i < state->num_tiles_x * state->num_tiles_y; i++) {
            if (state->tiles[i]) {
                if (state->tiles[i]->polygon_list)
                    free(state->tiles[i]->polygon_list);
                free(state->tiles[i]);
            }
        }
        free(state->tiles);
        state->tiles = NULL;
    }
    
    free(state);
}

/* Write to a 3D pipeline register */
void pvr_3d_write(pvr_3d_state_t* state, uint32_t addr, uint32_t val) {
    addr &= 0xfffff;  /* 20-bit address space for 3D registers */

    pvr_reg_write(state, addr, val);
    
    /* Base 3D engine registers */
    if (addr >= PVR_REG_BASE && addr < PVR_REG_BASE + 0x1000) {
        uint32_t reg_addr = addr - PVR_REG_BASE;
        
        switch (reg_addr) {
            case PVR_3D_RESET:
                if (val & 1) {
                    pclog("PowerVR Neon 250: 3D pipeline reset\n");
                    pvr_3d_reset(state);
                }
                break;
                
            case PVR_3D_STATUS:
                /* Status register is read-only */
                break;
                
            case PVR_3D_CONFIG:
                state->config_reg = val;
                pclog("PowerVR Neon 250: 3D config register set to %08x\n", val);
                break;
                
            default:
                pclog("PowerVR Neon 250: Unhandled write to 3D register %08x = %08x\n", addr, val);
                break;
        }
    }
    /* Polygon Setup registers */
    else if (addr >= PVR_POLY_BASE && addr < PVR_POLY_BASE + 0x1000) {
        uint32_t reg_addr = addr - PVR_POLY_BASE;
        
        switch (reg_addr) {
            case PVR_POLY_VERTEX_CMD:
                pvr_3d_process_command(state, 0x01, val); /* Vertex command */
                break;
                
            case PVR_POLY_TEXTURE_CMD:
                pvr_3d_process_command(state, 0x02, val); /* Texture command */
                break;
                
            case PVR_POLY_CONTROL:
                state->poly_control = val;
                break;
                
            default:
                pclog("PowerVR Neon 250: Unhandled write to polygon register %08x = %08x\n", addr, val);
                break;
        }
    }
    /* Texture mapping registers */
    else if (addr >= PVR_TEX_BASE && addr < PVR_TEX_BASE + 0x1000) {
        uint32_t reg_addr = addr - PVR_TEX_BASE;
        
        switch (reg_addr) {
            case PVR_TEX_CONTROL:
                state->tex_control = val;
                break;
                
            case PVR_TEX_ADDR:
                state->tex_addr = val;
                break;
                
            case PVR_TEX_CONFIG:
                state->tex_config = val;
                break;
                
            case PVR_TEX_FILTER:
                state->tex_filter = val;
                break;
                
            default:
                pclog("PowerVR Neon 250: Unhandled write to texture register %08x = %08x\n", addr, val);
                break;
        }
    }
    /* Rendering Engine registers */
    else if (addr >= PVR_RENDER_BASE && addr < PVR_RENDER_BASE + 0x1000) {
        uint32_t reg_addr = addr - PVR_RENDER_BASE;
        
        switch (reg_addr) {
            case PVR_RENDER_CONTROL:
                state->render_control = val;
                
                /* If bit 0 is set, start rendering the frame */
                if (val & 1) {
                    pclog("PowerVR Neon 250: Starting 3D frame rendering\n");
                    state->status_reg |= PVR_3D_STAT_RUNNING;
                    state->status_reg |= PVR_3D_STAT_BUSY;
                    
                    /* Schedule the rendering to complete after a realistic delay */
                    timer_set_delay_u64(&state->render_timer, 200 * TIMER_USEC);
                }
                break;
                
            case PVR_RENDER_Z_COMPARE:
                state->z_compare = val;
                break;
                
            case PVR_RENDER_BLEND:
                state->blend_mode = val;
                break;
                
            default:
                pclog("PowerVR Neon 250: Unhandled write to render register %08x = %08x\n", addr, val);
                break;
        }
    }
    /* DMA registers */
    else if (addr >= PVR_DMA_BASE && addr < PVR_DMA_BASE + 0x1000) {
        uint32_t reg_addr = addr - PVR_DMA_BASE;
        
        switch (reg_addr) {
            case PVR_DMA_CONTROL:
                state->dma_control = val;
                
                /* If bit 0 is set, start DMA transfer */
                if (val & 1) {
                    pclog("PowerVR Neon 250: Starting DMA transfer\n");
                    state->status_reg |= PVR_3D_STAT_DMA_ACTIVE;
                    state->dma_status |= 1;  /* DMA active flag */
                    
                    /* Execute the DMA transfer */
                    pvr_3d_dma_transfer(state);
                }
                break;
                
            case PVR_DMA_SRC_ADDR:
                state->dma_src_addr = val;
                break;
                
            case PVR_DMA_DEST_ADDR:
                state->dma_dest_addr = val;
                break;
                
            case PVR_DMA_SIZE:
                state->dma_size = val;
                break;
                
            default:
                pclog("PowerVR Neon 250: Unhandled write to DMA register %08x = %08x\n", addr, val);
                break;
        }
    }
    else {
        pclog("PowerVR Neon 250: Unhandled write to 3D pipeline address %08x = %08x\n", addr, val);
    }
}

/* Read from a 3D pipeline register */
uint32_t pvr_3d_read(pvr_3d_state_t* state, uint32_t addr) {
    uint32_t ret = 0xffffffff;
    addr &= 0xfffff;  /* 20-bit address space for 3D registers */

    return pvr_reg_read(state, addr);
    
    /* Base 3D engine registers */
    if (addr >= PVR_REG_BASE && addr < PVR_REG_BASE + 0x1000) {
        uint32_t reg_addr = addr - PVR_REG_BASE;
        
        switch (reg_addr) {
            case PVR_3D_RESET:
                ret = 0; /* Reset register is write-only */
                break;
                
            case PVR_3D_STATUS:
                ret = state->status_reg;
                break;
                
            case PVR_3D_CONFIG:
                ret = state->config_reg;
                break;
                
            default:
                pclog("PowerVR Neon 250: Unhandled read from 3D register %08x\n", addr);
                break;
        }
    }
    /* Polygon Setup registers */
    else if (addr >= PVR_POLY_BASE && addr < PVR_POLY_BASE + 0x1000) {
        uint32_t reg_addr = addr - PVR_POLY_BASE;
        
        switch (reg_addr) {
            case PVR_POLY_CONTROL:
                ret = state->poly_control;
                break;
                
            case PVR_POLY_STATUS:
                ret = state->poly_status;
                break;
                
            default:
                pclog("PowerVR Neon 250: Unhandled read from polygon register %08x\n", addr);
                break;
        }
    }
    /* Texture mapping registers */
    else if (addr >= PVR_TEX_BASE && addr < PVR_TEX_BASE + 0x1000) {
        uint32_t reg_addr = addr - PVR_TEX_BASE;
        
        switch (reg_addr) {
            case PVR_TEX_CONTROL:
                ret = state->tex_control;
                break;
                
            case PVR_TEX_ADDR:
                ret = state->tex_addr;
                break;
                
            case PVR_TEX_CONFIG:
                ret = state->tex_config;
                break;
                
            case PVR_TEX_FILTER:
                ret = state->tex_filter;
                break;
                
            default:
                pclog("PowerVR Neon 250: Unhandled read from texture register %08x\n", addr);
                break;
        }
    }
    /* Rendering Engine registers */
    else if (addr >= PVR_RENDER_BASE && addr < PVR_RENDER_BASE + 0x1000) {
        uint32_t reg_addr = addr - PVR_RENDER_BASE;
        
        switch (reg_addr) {
            case PVR_RENDER_CONTROL:
                ret = state->render_control;
                break;
                
            case PVR_RENDER_STATUS:
                ret = state->render_status;
                break;
                
            case PVR_RENDER_Z_COMPARE:
                ret = state->z_compare;
                break;
                
            case PVR_RENDER_BLEND:
                ret = state->blend_mode;
                break;
                
            default:
                pclog("PowerVR Neon 250: Unhandled read from render register %08x\n", addr);
                break;
        }
    }
    /* DMA registers */
    else if (addr >= PVR_DMA_BASE && addr < PVR_DMA_BASE + 0x1000) {
        uint32_t reg_addr = addr - PVR_DMA_BASE;
        
        switch (reg_addr) {
            case PVR_DMA_CONTROL:
                ret = state->dma_control;
                break;
                
            case PVR_DMA_SRC_ADDR:
                ret = state->dma_src_addr;
                break;
                
            case PVR_DMA_DEST_ADDR:
                ret = state->dma_dest_addr;
                break;
                
            case PVR_DMA_SIZE:
                ret = state->dma_size;
                break;
                
            case PVR_DMA_STATUS:
                ret = state->dma_status;
                break;
                
            default:
                pclog("PowerVR Neon 250: Unhandled read from DMA register %08x\n", addr);
                break;
        }
    }
    else {
        pclog("PowerVR Neon 250: Unhandled read from 3D pipeline address %08x\n", addr);
    }
    
    return ret;
}

/* Process a command sent to the 3D pipeline */
static void pvr_3d_process_command(pvr_3d_state_t* state, uint32_t command, uint32_t data) {
    /* Add to command FIFO */
    pvr_3d_push_fifo(state, command, data);
    
    /* Update status if FIFO is now not empty */
    if (state->fifo_entries > 0) {
        state->status_reg &= ~PVR_3D_STAT_FIFO_EMPTY;
    }
    
    /* Process the FIFO if not busy rendering */
    if (!(state->status_reg & PVR_3D_STAT_RUNNING)) {
        pvr_3d_process_fifo(state);
    }
}

/* Add an entry to the command FIFO */
static void pvr_3d_push_fifo(pvr_3d_state_t* state, uint32_t command, uint32_t data) {
    /* Check if FIFO is full */
    if (state->fifo_entries >= PVR_FIFO_SIZE) {
        pclog("PowerVR Neon 250: Command FIFO overflow\n");
        state->status_reg |= PVR_3D_STAT_FIFO_FULL;
        return;
    }
    
    /* Add command to FIFO */
    state->fifo[state->fifo_write_ptr].command = command;
    state->fifo[state->fifo_write_ptr].data = data;
    
    /* Update write pointer and entry count */
    state->fifo_write_ptr = (state->fifo_write_ptr + 1) % PVR_FIFO_SIZE;
    state->fifo_entries++;
    
    /* Set FIFO full status if necessary */
    if (state->fifo_entries >= PVR_FIFO_SIZE) {
        state->status_reg |= PVR_3D_STAT_FIFO_FULL;
    }
}

/* Process commands in the FIFO */
static void pvr_3d_process_fifo(pvr_3d_state_t* state) {
    /* Process up to 32 commands at once to prevent infinite loops */
    int commands_processed = 0;
    
    while (state->fifo_entries > 0 && commands_processed < 32) {
        pvr_fifo_entry_t* entry = &state->fifo[state->fifo_read_ptr];
        uint32_t command = entry->command;
        uint32_t data = entry->data;
        
        /* Execute the command */
        switch (command) {
            case 0x01: /* Vertex data */
                {
                    /* Extract vertex components based on current format */
                    float x = ((float)((data >> 0) & 0x3ff)) / 1024.0f * state->fb_width;
                    float y = ((float)((data >> 10) & 0x3ff)) / 1024.0f * state->fb_height;
                    float z = ((float)((data >> 20) & 0xfff)) / 4096.0f;
                    
                    /* Store in the current vertex */
                    state->vertex_buffer[state->current_vertex].x = x;
                    state->vertex_buffer[state->current_vertex].y = y;
                    state->vertex_buffer[state->current_vertex].z = z;
                    state->vertex_buffer[state->current_vertex].w = 1.0f; /* Default w */
                    
                    /* When we have 3 vertices, create a polygon */
                    state->current_vertex++;
                    if (state->current_vertex >= 3) {
                        pvr_3d_setup_polygon(state);
                        state->current_vertex = 0;
                    }
                }
                break;
                
            case 0x02: /* Texture data */
                {
                    /* Process texture command based on the type in top 8 bits */
                    uint8_t tex_cmd_type = (data >> 24) & 0xff;
                    
                    switch (tex_cmd_type) {
                        case 0x01: /* Texture address */
                            state->textures[state->current_texture].addr = data & 0xffffff;
                            break;
                            
                        case 0x02: /* Texture UV coordinates */
                            {
                                float u = ((float)((data >> 0) & 0xfff)) / 4096.0f;
                                float v = ((float)((data >> 12) & 0xfff)) / 4096.0f;
                                
                                /* Store in the current vertex */
                                state->vertex_buffer[state->current_vertex].u = u;
                                state->vertex_buffer[state->current_vertex].v = v;
                            }
                            break;
                            
                        case 0x03: /* Texture format */
                            state->textures[state->current_texture].format = data & 0xffffff;
                            break;
                            
                        default:
                            pclog("PowerVR Neon 250: Unknown texture command type %02x\n", tex_cmd_type);
                            break;
                    }
                }
                break;
                
            case 0x03: /* Color data */
                {
                    /* Extract RGBA components */
                    float r = ((float)((data >> 0) & 0xff)) / 255.0f;
                    float g = ((float)((data >> 8) & 0xff)) / 255.0f;
                    float b = ((float)((data >> 16) & 0xff)) / 255.0f;
                    float a = ((float)((data >> 24) & 0xff)) / 255.0f;
                    
                    /* Store in the current vertex */
                    state->vertex_buffer[state->current_vertex].r = r;
                    state->vertex_buffer[state->current_vertex].g = g;
                    state->vertex_buffer[state->current_vertex].b = b;
                    state->vertex_buffer[state->current_vertex].a = a;
                }
                break;
                
            case 0x10: /* Start rendering */
                pclog("PowerVR Neon 250: Start rendering command received\n");
                state->status_reg |= PVR_3D_STAT_RUNNING;
                state->status_reg |= PVR_3D_STAT_BUSY;
                
                /* Start rendering the collected polygons */
                pvr_3d_render_scene(state);
                
                /* Set a timer to simulate rendering time */
                timer_set_delay_u64(&state->render_timer, 200 * TIMER_USEC);
                break;
                
            default:
                pclog("PowerVR Neon 250: Unknown command %08x with data %08x\n", command, data);
                break;
        }
        
        /* Update read pointer and entry count */
        state->fifo_read_ptr = (state->fifo_read_ptr + 1) % PVR_FIFO_SIZE;
        state->fifo_entries--;
        commands_processed++;
        
        /* Clear FIFO full status */
        state->status_reg &= ~PVR_3D_STAT_FIFO_FULL;
        
        /* Set FIFO empty status if necessary */
        if (state->fifo_entries == 0) {
            state->status_reg |= PVR_3D_STAT_FIFO_EMPTY;
        }
    }
}

/* Set up a polygon from the current set of vertices */
static void pvr_3d_setup_polygon(pvr_3d_state_t* state) {
    /* Only process if we have enough polygons */
    if (state->num_polygons >= PVR_MAX_POLYGONS) {
        pclog("PowerVR Neon 250: Maximum polygon count exceeded\n");
        return;
    }
    
    pvr_polygon_t* poly = &state->polygon_buffer[state->num_polygons];
    
    /* Copy vertices from the vertex buffer */
    poly->num_vertices = 3;
    memcpy(poly->vertices, state->vertex_buffer, 3 * sizeof(pvr_vertex_t));
    
    /* Set polygon properties */
    poly->texture_addr = state->textures[state->current_texture].addr;
    poly->control_flags = state->poly_control;
    
    /* Calculate z-sort value (average Z of vertices for tile-based rendering) */
    poly->z_sort_value = (uint32_t)(
        (poly->vertices[0].z + poly->vertices[1].z + poly->vertices[2].z) * 1365.0f
    );
    
    /* Distribute this polygon to the appropriate tiles */
    pvr_3d_distribute_to_tiles(state, poly);
    
    state->num_polygons++;
}

/* Distribute a polygon to the tiles it intersects */
static void pvr_3d_distribute_to_tiles(pvr_3d_state_t* state, pvr_polygon_t* poly) {
    /* Find the bounding box of the polygon */
    float min_x = poly->vertices[0].x;
    float min_y = poly->vertices[0].y;
    float max_x = poly->vertices[0].x;
    float max_y = poly->vertices[0].y;
    
    for (int i = 1; i < poly->num_vertices; i++) {
        if (poly->vertices[i].x < min_x) min_x = poly->vertices[i].x;
        if (poly->vertices[i].y < min_y) min_y = poly->vertices[i].y;
        if (poly->vertices[i].x > max_x) max_x = poly->vertices[i].x;
        if (poly->vertices[i].y > max_y) max_y = poly->vertices[i].y;
    }
    
    /* Determine the tiles that the polygon intersects */
    int start_tile_x = (int)min_x / state->tile_size;
    int start_tile_y = (int)min_y / state->tile_size;
    int end_tile_x = (int)(max_x + state->tile_size - 1) / state->tile_size;
    int end_tile_y = (int)(max_y + state->tile_size - 1) / state->tile_size;
    
    /* Clamp to the tile grid */
    if (start_tile_x < 0) start_tile_x = 0;
    if (start_tile_y < 0) start_tile_y = 0;
    if (end_tile_x >= state->num_tiles_x) end_tile_x = state->num_tiles_x - 1;
    if (end_tile_y >= state->num_tiles_y) end_tile_y = state->num_tiles_y - 1;
    
    /* Add polygon to each intersecting tile */
    for (int y = start_tile_y; y <= end_tile_y; y++) {
        for (int x = start_tile_x; x <= end_tile_x; x++) {
            int tile_idx = y * state->num_tiles_x + x;
            pvr_tile_t* tile = state->tiles[tile_idx];
            
            /* Check if we need to expand the polygon list */
            if (tile->num_polygons >= tile->max_polygons) {
                tile->max_polygons *= 2;
                tile->polygon_list = realloc(tile->polygon_list, 
                                             tile->max_polygons * sizeof(pvr_polygon_t*));
            }
            
            /* Add the polygon to this tile's list */
            tile->polygon_list[tile->num_polygons] = poly;
            tile->num_polygons++;
        }
    }
}

/* Render the entire scene using tile-based rendering */
static void pvr_3d_render_scene(pvr_3d_state_t* state) {
    pclog("PowerVR Neon 250: Beginning scene render with %d polygons\n", state->num_polygons);
    
    /* Clear the Z-buffer */
    if (state->z_buffer) {
        memset(state->z_buffer, 0xFF, state->fb_width * state->fb_height * sizeof(uint16_t));
    }
    
    /* For each tile, render all its polygons */
    for (int y = 0; y < state->num_tiles_y; y++) {
        for (int x = 0; x < state->num_tiles_x; x++) {
            int tile_idx = y * state->num_tiles_x + x;
            pvr_tile_t* tile = state->tiles[tile_idx];
            
            if (tile->num_polygons > 0) {
                pvr_3d_render_tile(state, tile);
            }
        }
    }
    
    /* Reset polygon count */
    state->num_polygons = 0;
    
    /* Clear all tile polygon counts */
    for (int i = 0; i < state->num_tiles_x * state->num_tiles_y; i++) {
        state->tiles[i]->num_polygons = 0;
    }
    
    /* Tell the SVGA system that the display has changed */
    state->neon250->svga.fullchange = changeframecount;
}

/* Render all polygons in a tile */
static void pvr_3d_render_tile(pvr_3d_state_t* state, pvr_tile_t* tile) {
    /* Sort polygons by Z for proper transparency handling */
    /* A real PowerVR would use a binning algorithm for OpB, but this is simplified */
    
    /* Simple bubble sort of polygons by z_sort_value (largest/furthest first) */
    for (int i = 0; i < tile->num_polygons - 1; i++) {
        for (int j = 0; j < tile->num_polygons - i - 1; j++) {
            if (tile->polygon_list[j]->z_sort_value < tile->polygon_list[j+1]->z_sort_value) {
                /* Swap pointers */
                pvr_polygon_t* temp = tile->polygon_list[j];
                tile->polygon_list[j] = tile->polygon_list[j+1];
                tile->polygon_list[j+1] = temp;
            }
        }
    }
    
    /* Render all polygons in the tile */
    for (int i = 0; i < tile->num_polygons; i++) {
        pvr_3d_render_polygon(state, tile->polygon_list[i]);
    }
}

/* Render a single polygon */
static void pvr_3d_render_polygon(pvr_3d_state_t* state, pvr_polygon_t* poly) {
    /* Check if backface culling is enabled */
    if (poly->control_flags & (PVR_POLY_CTRL_CULL_CW | PVR_POLY_CTRL_CULL_CCW)) {
        /* Calculate cross product to determine facing */
        float ax = poly->vertices[1].x - poly->vertices[0].x;
        float ay = poly->vertices[1].y - poly->vertices[0].y;
        float bx = poly->vertices[2].x - poly->vertices[0].x;
        float by = poly->vertices[2].y - poly->vertices[0].y;
        
        float cross = ax * by - ay * bx;
        
        /* Cull if the polygon is facing the wrong way */
        if ((cross < 0 && (poly->control_flags & PVR_POLY_CTRL_CULL_CCW)) ||
            (cross > 0 && (poly->control_flags & PVR_POLY_CTRL_CULL_CW))) {
            return;
        }
    }
    
    /* Draw the triangle */
    pvr_3d_draw_triangle(state, poly);
}

/* Simple edge function for barycentric coordinates */
static float edge_function(float x0, float y0, float x1, float y1, float x2, float y2) {
    return (x2 - x0) * (y1 - y0) - (y2 - y0) * (x1 - x0);
}

/* Draw a 3D triangle with texturing and shading */
static void pvr_3d_draw_triangle(pvr_3d_state_t* state, pvr_polygon_t* poly) {
    /* Get vertex positions */
    float x0 = poly->vertices[0].x;
    float y0 = poly->vertices[0].y;
    float x1 = poly->vertices[1].x;
    float y1 = poly->vertices[1].y;
    float x2 = poly->vertices[2].x;
    float y2 = poly->vertices[2].y;
    
    /* Find bounding box */
    int minX = (int)floorf(fminf(fminf(x0, x1), x2));
    int minY = (int)floorf(fminf(fminf(y0, y1), y2));
    int maxX = (int)ceilf(fmaxf(fmaxf(x0, x1), x2));
    int maxY = (int)ceilf(fmaxf(fmaxf(y0, y1), y2));
    
    /* Clip to framebuffer boundaries */
    if (minX < 0) minX = 0;
    if (minY < 0) minY = 0;
    if (maxX > state->fb_width) maxX = state->fb_width;
    if (maxY > state->fb_height) maxY = state->fb_height;
    
    /* Calculate area of the triangle */
    float area = edge_function(x0, y0, x1, y1, x2, y2);
    
    /* Skip rendering if triangle is degenerate */
    if (fabsf(area) < 0.000001f) {
        return;
    }
    
    /* Precompute 1/area for barycentric coordinates */
    float inv_area = 1.0f / area;
    
    /* Rasterize the triangle using barycentric coordinates */
    for (int y = minY; y < maxY; y++) {
        for (int x = minX; x < maxX; x++) {
            /* Compute barycentric coordinates */
            float w0 = edge_function(x1, y1, x2, y2, x + 0.5f, y + 0.5f) * inv_area;
            float w1 = edge_function(x2, y2, x0, y0, x + 0.5f, y + 0.5f) * inv_area;
            float w2 = edge_function(x0, y0, x1, y1, x + 0.5f, y + 0.5f) * inv_area;
            
            /* Skip pixels outside the triangle */
            if (w0 < 0 || w1 < 0 || w2 < 0) {
                continue;
            }
            
            /* Interpolate Z value for depth testing */
            float z = w0 * poly->vertices[0].z + w1 * poly->vertices[1].z + w2 * poly->vertices[2].z;
            
            /* Convert to fixed-point depth value (0-65535) */
            uint16_t depth = (uint16_t)(z * 65535.0f);
            
            /* Perform depth test if enabled */
            if (poly->control_flags & PVR_POLY_CTRL_ZBUFFER) {
                uint16_t* z_ptr = &state->z_buffer[y * state->fb_width + x];
                
                /* Compare with existing depth value */
                if (depth > *z_ptr) {
                    continue; /* Fail depth test */
                }
                
                /* Update Z-buffer */
                *z_ptr = depth;
            }
            
            /* Calculate pixel color */
            uint32_t color;
            
            if (poly->control_flags & PVR_POLY_CTRL_TEXTURE) {
                /* Interpolate texture coordinates */
                float u = w0 * poly->vertices[0].u + w1 * poly->vertices[1].u + w2 * poly->vertices[2].u;
                float v = w0 * poly->vertices[0].v + w1 * poly->vertices[1].v + w2 * poly->vertices[2].v;
                
                /* Clamp UVs to [0,1] */
                u = fmaxf(0.0f, fminf(1.0f, u));
                v = fmaxf(0.0f, fminf(1.0f, v));
                
                /* Fetch texture color (simplified) */
                uint32_t tex_color = 0x00FFFFFF; /* Default to white */
                
                /* Blend texture with vertex color if Gouraud shading is enabled */
                if (poly->control_flags & PVR_POLY_CTRL_GOURAUD) {
                    /* Interpolate vertex colors */
                    float r = w0 * poly->vertices[0].r + w1 * poly->vertices[1].r + w2 * poly->vertices[2].r;
                    float g = w0 * poly->vertices[0].g + w1 * poly->vertices[1].g + w2 * poly->vertices[2].g;
                    float b = w0 * poly->vertices[0].b + w1 * poly->vertices[1].b + w2 * poly->vertices[2].b;
                    float a = w0 * poly->vertices[0].a + w1 * poly->vertices[1].a + w2 * poly->vertices[2].a;
                    
                    /* Modulate texture with vertex color */
                    uint8_t tr = ((tex_color >> 16) & 0xFF);
                    uint8_t tg = ((tex_color >> 8) & 0xFF);
                    uint8_t tb = (tex_color & 0xFF);
                    
                    r = (r * tr) / 255.0f;
                    g = (g * tg) / 255.0f;
                    b = (b * tb) / 255.0f;
                    
                    uint8_t ir = (uint8_t)(r * 255.0f);
                    uint8_t ig = (uint8_t)(g * 255.0f);
                    uint8_t ib = (uint8_t)(b * 255.0f);
                    uint8_t ia = (uint8_t)(a * 255.0f);
                    
                    color = (ia << 24) | (ir << 16) | (ig << 8) | ib;
                } else {
                    /* Just use the texture color */
                    color = tex_color;
                }
            } else if (poly->control_flags & PVR_POLY_CTRL_GOURAUD) {
                /* Gouraud shading without texture */
                float r = w0 * poly->vertices[0].r + w1 * poly->vertices[1].r + w2 * poly->vertices[2].r;
                float g = w0 * poly->vertices[0].g + w1 * poly->vertices[1].g + w2 * poly->vertices[2].g;
                float b = w0 * poly->vertices[0].b + w1 * poly->vertices[1].b + w2 * poly->vertices[2].b;
                float a = w0 * poly->vertices[0].a + w1 * poly->vertices[1].a + w2 * poly->vertices[2].a;
                
                uint8_t ir = (uint8_t)(r * 255.0f);
                uint8_t ig = (uint8_t)(g * 255.0f);
                uint8_t ib = (uint8_t)(b * 255.0f);
                uint8_t ia = (uint8_t)(a * 255.0f);
                
                color = (ia << 24) | (ir << 16) | (ig << 8) | ib;
            } else {
                /* Flat shading - use color from first vertex */
                uint8_t r = (uint8_t)(poly->vertices[0].r * 255.0f);
                uint8_t g = (uint8_t)(poly->vertices[0].g * 255.0f);
                uint8_t b = (uint8_t)(poly->vertices[0].b * 255.0f);
                uint8_t a = (uint8_t)(poly->vertices[0].a * 255.0f);
                
                color = (a << 24) | (r << 16) | (g << 8) | b;
            }
            
            /* Write the pixel to the framebuffer according to current bit depth */
            int offset = y * state->fb_stride + x;
            
            switch (state->fb_format) {
                case 16: /* 16-bit color (RGB565) */
                    {
                        uint16_t r = ((color >> 16) & 0xFF) >> 3;
                        uint16_t g = ((color >> 8) & 0xFF) >> 2;
                        uint16_t b = (color & 0xFF) >> 3;
                        
                        uint16_t pixel = (r << 11) | (g << 5) | b;
                        *((uint16_t*)&state->framebuffer[offset * 2]) = pixel;
                    }
                    break;
                    
                case 24: /* 24-bit color (RGB888) */
                    state->framebuffer[offset * 3 + 0] = color & 0xFF;          /* B */
                    state->framebuffer[offset * 3 + 1] = (color >> 8) & 0xFF;   /* G */
                    state->framebuffer[offset * 3 + 2] = (color >> 16) & 0xFF;  /* R */
                    break;
                    
                case 32: /* 32-bit color (ARGB8888) */
                    *((uint32_t*)&state->framebuffer[offset * 4]) = color;
                    break;
                    
                default:
                    break;
            }
        }
    }
}

/* Timer callback for when rendering is completed */
static void pvr_3d_timer_callback(void* priv) {
    pvr_3d_state_t* state = (pvr_3d_state_t*)priv;
    
    /* Clear rendering active flags */
    state->status_reg &= ~PVR_3D_STAT_RUNNING;
    state->status_reg &= ~PVR_3D_STAT_BUSY;
    
    /* Update registers to reflect render completion */
    pvr_reg_render_complete(state);

    
    /* Set render complete status */
    state->render_status |= 1;
    
    pclog("PowerVR Neon 250: 3D rendering complete\n");
    
    /* Process any pending commands in the FIFO */
    pvr_3d_process_fifo(state);
}

/* Perform a DMA transfer */
static void pvr_3d_dma_transfer(pvr_3d_state_t* state) {
    /* Simplified DMA transfer - just copy memory */
    uint32_t src_addr = state->dma_src_addr;
    uint32_t dest_addr = state->dma_dest_addr;
    uint32_t size = state->dma_size;
    
    /* Update registers to reflect DMA completion */
    pvr_reg_dma_complete(state);

    pclog("PowerVR Neon 250: DMA transfer from %08x to %08x, size %08x\n", 
          src_addr, dest_addr, size);
    
    /* Safety check for size */
    if (size > 16 * 1024 * 1024) {
        pclog("PowerVR Neon 250: DMA transfer size too large, limiting to 16MB\n");
        size = 16 * 1024 * 1024;
    }
    
    /* Check if source and destination are within our memory range */
    if (src_addr < state->neon250->memory_size && 
        dest_addr < state->neon250->memory_size && 
        (dest_addr + size) <= state->neon250->memory_size) {
        
        /* Perform the DMA copy */
        memcpy(&state->neon250->svga.vram[dest_addr], 
               &state->neon250->svga.vram[src_addr], 
               size);
    } else {
        pclog("PowerVR Neon 250: DMA transfer outside valid memory range\n");
    }
    
    /* Update status registers */
    state->status_reg &= ~PVR_3D_STAT_DMA_ACTIVE;
    state->dma_status &= ~1; /* Clear active flag */
    state->dma_status |= 2;  /* Set complete flag */
}

/* Update the 3D pipeline state when display parameters change */
void pvr_3d_update_display(pvr_3d_state_t* state, int width, int height, 
                          int stride, int bpp, uint8_t* vram) {
    state->fb_width = width;
    state->fb_height = height;
    state->fb_stride = stride;
    state->fb_format = bpp;
    state->framebuffer = vram;
    
    /* Resize Z-buffer if necessary */
    if (state->z_buffer) {
        free(state->z_buffer);
        state->z_buffer = calloc(width * height, sizeof(uint16_t));
    }
    
    /* Re-initialize tile structure if necessary */
    if (state->tiles) {
        /* Free existing tiles */
        for (int i = 0; i < state->num_tiles_x * state->num_tiles_y; i++) {
            if (state->tiles[i]) {
                if (state->tiles[i]->polygon_list)
                    free(state->tiles[i]->polygon_list);
                free(state->tiles[i]);
            }
        }
        free(state->tiles);
        
        /* Calculate new tiling */
        state->num_tiles_x = (width + state->tile_size - 1) / state->tile_size;
        state->num_tiles_y = (height + state->tile_size - 1) / state->tile_size;
        
        /* Allocate new tiles */
        state->tiles = calloc(state->num_tiles_x * state->num_tiles_y, sizeof(pvr_tile_t*));
        
        /* Initialize tiles */
        for (int y = 0; y < state->num_tiles_y; y++) {
            for (int x = 0; x < state->num_tiles_x; x++) {
                int tile_idx = y * state->num_tiles_x + x;
                state->tiles[tile_idx] = calloc(1, sizeof(pvr_tile_t));
                
                pvr_tile_t* tile = state->tiles[tile_idx];
                tile->x = x * state->tile_size;
                tile->y = y * state->tile_size;
                tile->width = state->tile_size;
                tile->height = state->tile_size;
                
                /* Handle edge tiles */
                if (tile->x + tile->width > width)
                    tile->width = width - tile->x;
                    
                if (tile->y + tile->height > height)
                    tile->height = height - tile->y;
                    
                /* Allocate initial polygon list for this tile */
                tile->max_polygons = 64; /* Start with space for 64 polygons */
                tile->polygon_list = calloc(tile->max_polygons, sizeof(pvr_polygon_t*));
                tile->num_polygons = 0;
            }
        }
    }
}