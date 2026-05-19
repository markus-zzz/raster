# What aims to be a most basic GPU implemented in a cheap FPGA

Some time ago I wrote
https://www.zzzconsulting.se/2021/03/26/basic-gpu-part-1.html and now a few
years later, with the help of awesome AI, I felt it would be nice to try it
out.


## Misc

Create `.mp4` video of generated frames
```
$ ffmpeg -framerate 10 -i frame_%03d.ppm -c:v libx264 -crf 25 -vf "format=yuv420p" -movflags +faststart output.mp4
```

https://github.com/user-attachments/assets/c3a629ee-509a-4543-88f7-aaa2811e204b

