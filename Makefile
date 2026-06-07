CXX = g++
VERILATOR = ~/work/install/bin/verilator

all: system

system: system_top.sv gpu_top.sv raster_top.sv rasterizer.sv dpram.sv sdram_ctrl.sv sdram_model.sv tb_system.cpp
	$(VERILATOR) --cc --exe --build -j 0 --trace-fst -Wall -CFLAGS '-std=c++20 -O3' \
    -Wno-UNUSEDPARAM -Wno-UNUSEDSIGNAL -Wno-WIDTHTRUNC -Wno-WIDTHEXPAND \
		--top-module system_top $^ -o Vsystem_top
	cp obj_dir/Vsystem_top $@

clean:
	rm -f system frame_*.ppm
	rm -rf obj_dir
