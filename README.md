# What aims to be a most basic GPU implemented in a cheap FPGA

Some time ago I wrote
https://www.zzzconsulting.se/2021/03/26/basic-gpu-part-1.html and now a few
years later, with the help of awesome AI, I felt it would be nice to try it
out.

## Geometry processing (the plan)

### Input

- List of all vertices `(x,y,z)` in `Q16.16` format i.e. `3 x 32-bits = 12
  bytes` per vertex.
- `4x4` transformation matrix in `Q16.16` format i.e. `16 x 32-bits = 64 bytes`.
- List of all faces/triangles with `16-bit` index for each vertex `(v0,v1,v2)` i.e.
  `3 x 16-bits = 6 bytes` per face/triangle.
- One directional light source `(x,y,z)` in `Q16.16` format i.e. `3 x 32-bits = 12
  bytes`.

### Intermediate

- Compute face normals
- Back face culling
- Lighting

### Output

Put transformed triangle vertices in bin list for all tiles that overlap the
primitive's bounding box.

For each face/triangle:
- Three vertices transformed to screen space `(x,y)` i.e. `3 x 2 x 16-bits = 12
  bytes`.
- One solid color per face in `RGB565` format i.e. `1 x 16-bits = 2 bytes`.
- `1/z` for each vertex i.e. `3 x 16-bits = 6 bytes`.

So `20 bytes` in total for each bin list entry.


## Misc

Create `.mp4` video of generated frames
```
$ ffmpeg -framerate 10 -i frame_%03d.ppm -c:v libx264 -crf 25 -vf "format=yuv420p" -movflags +faststart output.mp4
```

https://github.com/user-attachments/assets/c3a629ee-509a-4543-88f7-aaa2811e204b

