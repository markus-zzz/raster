CXX = g++
VERILATOR = ~/work/install/bin/verilator

all: raster sim gpu

raster: raster.cpp
	$(CXX) -O2 -o $@ $<

sim: raster_top.sv rasterizer.sv dpram.sv tb_rasterizer.cpp
	$(VERILATOR) --cc --exe --build -j 0 -Wall -Wno-UNUSEDPARAM -Wno-UNUSEDSIGNAL \
		--top-module raster_top \
		raster_top.sv rasterizer.sv dpram.sv tb_rasterizer.cpp
	cp obj_dir/Vraster_top sim

gpu: gpu_top.sv raster_top.sv rasterizer.sv dpram.sv tb_gpu.cpp
	$(VERILATOR) --cc --exe --build -j 0 --trace-fst -Wall -Wno-UNUSEDPARAM -Wno-UNUSEDSIGNAL \
		-Wno-WIDTHTRUNC -Wno-WIDTHEXPAND \
		--top-module gpu_top \
		gpu_top.sv raster_top.sv rasterizer.sv dpram.sv tb_gpu.cpp \
		-o Vgpu_top
	cp obj_dir/Vgpu_top gpu

system: system_top.sv gpu_top.sv raster_top.sv rasterizer.sv dpram.sv sdram_ctrl.sv sdram_model.sv tb_system.cpp
	$(VERILATOR) --cc --exe --build -j 0 --trace-fst -Wall -Wno-UNUSEDPARAM -Wno-UNUSEDSIGNAL \
		-Wno-WIDTHTRUNC -Wno-WIDTHEXPAND \
		--top-module system_top \
		system_top.sv gpu_top.sv raster_top.sv rasterizer.sv dpram.sv sdram_ctrl.sv sdram_model.sv tb_system.cpp \
		-o Vsystem_top
	cp obj_dir/Vsystem_top system

test: sim raster
	./raster
	./sim

clean:
	rm -f raster sim out.ppm out_hw.ppm
	rm -rf obj_dir
