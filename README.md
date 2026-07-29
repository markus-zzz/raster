# What aims to be a most basic GPU implemented in a cheap FPGA

Some time ago I wrote
https://www.zzzconsulting.se/2021/03/26/basic-gpu-part-1.html and now a few
years later, with the help of awesome AI, I felt it would be nice to try it
out.

https://github.com/user-attachments/assets/689c444d-43c9-43ad-b34e-12494b35cf30


## Overview

A small tile-based rasterizing GPU. A soft CPU (PicoRV32) drives it; the fixed
-function hardware does the heavy lifting. All object/scene data lives in SDRAM,
which is shared between the CPU, the two GPU passes and the display via an
arbiter (the display has highest priority).

Per frame:

1. **CPU** computes the per-frame transform matrices, writes them (and, once at
   start-up, the meshes and a descriptor list) into SDRAM, then kicks the two
   passes and waits for each to finish.
2. **Geometry pass** (`geom_front` + `geom_engine`) walks the descriptor list,
   transforms/culls/lights/projects each triangle, and *bins* it into the tiles
   its bounding box overlaps.
3. **Rasterization pass** (`rast_front` + `rasterizer`) walks the tiles, and for
   each tile rasterizes the binned triangles into a small on-chip tile buffer
   (with a depth test), then streams the tile out to the framebuffer in SDRAM.
4. **Display** (`display_ctrl`) scans the framebuffer out to the LCD. Rendering
   is double-buffered.

```
  CPU (PicoRV32)                SDRAM                         LCD
  ┌────────────┐   matrices   ┌───────────────┐
  │ build desc │─────────────▶│ descriptors   │
  │ list, mats │              │ vertices/faces│
  └─────┬──────┘              │ matrices      │
        │ MMIO: bases,        └───────────────┘
        │ desc head, start           ▲  │
        ▼                            │  ▼
  ┌───────────────┐   TRI recs  ┌───────────────┐   pixels    ┌───────────────┐
  │ geometry pass │────────────▶│ TRI / BINLIST │────────────▶│ raster pass   │
  │ geom_front +  │   bin lists │ / BIN counts  │  bins       │ rast_front +  │
  │ geom_engine   │             └───────────────┘             │ rasterizer    │
  └───────────────┘                                           └──────┬────────┘
                                 ┌───────────────┐   scanout         │ tile dump
                                 │ framebuffer   │◀─── display_ctrl ◀┘
                                 │ (double-buf)  │─────────────────────────▶ LCD
                                 └───────────────┘
```

Tiles are `64 x 64`. The panel is `320 x 480`, so the tile grid is
`5 x 8 = 40` tiles. Because `480` is not a multiple of `64` the grid covers
`512` rows, so the framebuffer is allocated `320 x 512` (only `320 x 480` is
scanned out).


## Fixed-point formats

- **Q20.12** (`int32`, 20 integer / 12 fractional bits, `1.0 = 4096`) is the
  main format: vertices, matrices, normals and colours. Multiply is
  `(a*b) >> 12` with a 64-bit intermediate.
- **Q11.5** (`int16`) is used for the screen-space triangle record the
  rasterizer consumes (see below).
- **Screen coordinates** are stored with 4 fractional (sub-pixel) bits:
  `x_sub = x_pixel * 16 + 8` (the `+8` centres the sample on the pixel).
- **Angles** are binary: a full turn is `2^32`, so wrap is free. `sin`/`cos`
  come from a 256-entry quarter-wave LUT (Q20.12) with linear interpolation and
  quadrant reconstruction.


## Scene data formats

All multi-byte fields are little-endian; SDRAM is addressed in 16-bit
halfwords and accessed in 8-halfword bursts. Base addresses passed around
(descriptor fields, MMIO registers) are halfword addresses.

**Vertex** (`GpuVertex_t`, 16 bytes):

| field   | type  | notes                 |
|---------|-------|-----------------------|
| x, y, z | Q20.12| object-space position |
| _pad    | u32   | pad to 16 bytes       |

**Face** (`GpuFace_t`): three `u16` vertex indices, a face normal
`(nx, ny, nz)` and a solid face colour `(r, g, b)` — all Q20.12. Winding is
CCW-from-outside and the stored normal is the *inward* normal
`-(v1-v0) x (v2-v0)`, which folds in the `det = -1` of the view matrix's Y-flip
so back-face culling keeps the right faces.

**Matrix** (`GpuMatrix_t`): `4 x 4` Q20.12, row-major. Only the top three rows
are used (affine; row 3 is implicit). Column 3 is the translation.

### Descriptor list

Geometry work is a null-terminated linked list of command descriptors
(`GeomDesc_t`) in SDRAM. The CPU builds it once and points the hardware at the
head via an MMIO register.

```c
struct GeomDesc_t {
  uint32_t cmd;        // GEOM_DESC_CMD_SET = 0, GEOM_DESC_CMD_OBJ = 1
  uint32_t next;       // halfword address of next descriptor; 0 = end of list
  union {
    struct {           // SET: set output targets + light, delimits a frame
      uint32_t tri_base;      // where triangle records go
      uint32_t binlist_base;  // per-tile triangle-index lists
      uint32_t bin_base;      // per-tile triangle counts
      uint32_t light_dir[3];  // negated, normalised light direction (Q20.12)
    } set;
    struct {           // OBJ: transform + bin one mesh
      uint32_t vertex_base;
      uint32_t faces_base;
      uint32_t matrix_base;
      uint16_t num_faces;
    } obj;
  };
};
```

- `SET` latches the output bases and light, and delimits a frame: if a region
  was already active it is first *finalised* (partial bins flushed, per-tile
  counts written) to the old bases.
- `OBJ` transforms and bins one mesh, *accumulating* into the shared per-tile
  buckets (no reset). Multiple `OBJ`s therefore share tiles and depth-sort
  correctly against each other.
- End of list finalises the current region.

So one list can carry several objects and even several frames' worth of work
(`SET` … objects … `SET` … objects …).


## Geometry pass

For each `OBJ` descriptor `geom_front` reads the matrix and, per face, the face
record and its three vertices, and feeds `geom_engine`, which:

1. Transforms the 3 vertices and the normal by the matrix. The four dot
   products are issued back-to-back through a pipelined 4-lane multiplier
   (`dot4`) — 12 transform dot products stream through in ~15 cycles.
2. **Back-face culls**: drop the triangle if the transformed normal's
   `z >= 0` (camera looks toward `-Z`).
3. **Lights**: `intensity = 0.2 + 0.8 * max(0, dot(n, -L))`, applied to the
   per-face colour.
4. **Projects**: `sx = 80 * x_view + W/2`, `sy = 80 * y_view + H/2`
   (orthographic scale), and a depth `z = 275 + 225 * z_view` (clamped to
   `>= 1`).
5. Sets up the screen-space **depth plane** `iz00 + ddx*x + ddy*y` through the
   three vertices. The two gradient divides share one denominator (the triangle
   area) and are done with a normalised Newton-Raphson reciprocal + multiply
   (`recipdiv`) rather than a long iterative divide.

The result is a **triangle record** (`TRI`), 16 halfwords = 32 bytes = two
bursts:

| word(s) | contents                                        | format |
|---------|-------------------------------------------------|--------|
| 0..5    | screen (x, y) of the 3 vertices                 | sub-pixel (×16) |
| 6       | depth at pixel origin (`iz00`)                  | Q11.5  |
| 7, 8    | depth gradients d/dx, d/dy                      | Q11.5  |
| 9       | `{g, b}` (8 bits each)                          | RGB    |
| 10      | `{0, r}`                                        | RGB    |
| 11..15  | unused                                          |        |

`geom_front` then bins the triangle: for every tile its bounding box overlaps
it appends the triangle's global index to that tile's list. Bin entries are
buffered 8-at-a-time on-chip and flushed as aligned bursts. On finalise the
per-tile counts are written out.

Output layout in SDRAM:
- `TRI_BASE`: 16-halfword record per triangle, indexed by global triangle id.
- `BINLIST_BASE`: per-tile lists of triangle ids, stride `MAX_FACES_PER_TILE`
  (1024) halfwords per tile.
- `BIN_BASE`: one triangle count per tile.


## Rasterization pass

`rast_front` walks the tiles. For each tile it reads the count, clears the
on-chip tile buffer, then for each binned triangle fetches its `TRI` record and
hands it to `rasterizer`, which:

- Walks the tile in `2 x 2` pixel quads using incremental edge functions and
  the top-left fill rule.
- Interpolates the depth plane per pixel and does a **z-test** against the
  tile's depth buffer (larger value = nearer wins), so triangles from different
  objects occlude correctly.
- Converts the 24-bit face colour to **RGB565** for storage.

When the tile is done it is streamed out to the framebuffer in SDRAM.
`display_ctrl` continuously reads the framebuffer (RGB565) through a FIFO and
drives the LCD; the CPU flips `R_DISP_FB_BASE` between the two buffers each
frame.


## Misc

Create an `.mp4` video from the simulator's captured frames:
```
$ ffmpeg -framerate 60 -i lcd-%04d.png -c:v libx264 -crf 25 -vf "format=yuv420p" -movflags +faststart output.mp4
```


## Credits

- Claude Opus 4.6-4.8
- https://github.com/YosysHQ/picorv32
- https://www.scratchapixel.com/
