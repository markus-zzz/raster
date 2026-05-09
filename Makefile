CXX = g++
VERILATOR = ~/work/install/bin/verilator

all: raster sim

raster: raster.cpp
	$(CXX) -O2 -o $@ $<

sim: raster_top.sv rasterizer.sv dpram.sv tb_rasterizer.cpp
	$(VERILATOR) --cc --exe --build -j 0 -Wall -Wno-UNUSEDPARAM -Wno-UNUSEDSIGNAL \
		--top-module raster_top \
		raster_top.sv rasterizer.sv dpram.sv tb_rasterizer.cpp
	cp obj_dir/Vraster_top sim

test: sim raster
	./raster
	./sim

clean:
	rm -f raster sim out.ppm out_hw.ppm
	rm -rf obj_dir
