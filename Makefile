CXX = g++
VERILATOR = ~/work/install/bin/verilator

SRCS = system_top.sv gpu_top.sv raster_top.sv rasterizer.sv dpram.sv \
       sdram_ctrl.sv sdram_model.sv arbiter.sv display_ctrl.sv \
       geom_front.sv geom_engine.sv tb_system.cpp

VFLAGS = --cc --exe --build -j 0 --trace-fst -Wall \
    -Wno-UNUSEDPARAM -Wno-UNUSEDSIGNAL -Wno-WIDTHTRUNC -Wno-WIDTHEXPAND \
    -Wno-DECLFILENAME \
    --top-module system_top

all: system

# Normal build: testbench processes geometry into SDRAM at runtime and also
# dumps sdram_init.hex.
system: $(SRCS)
	$(VERILATOR) $(VFLAGS) -CFLAGS '-std=c++20 -O3' \
		$(SRCS) -o Vsystem_top
	cp obj_dir/Vsystem_top $@

# Init build: SDRAM is preloaded from sdram_init.hex via $readmemh (set through
# the SDRAM_INIT_FILE parameter). The testbench skips runtime geometry
# population (SDRAM_INIT_MODE), so every frame renders the same preloaded image.
# Uses a separate obj dir so it does not clobber the normal build.
system_init: $(SRCS)
	$(VERILATOR) $(VFLAGS) -CFLAGS '-std=c++20 -O3 -DSDRAM_INIT_MODE' \
		-GSDRAM_INIT_FILE='"sdram_init.hex"' \
		--Mdir obj_dir_init \
		$(SRCS) -o Vsystem_top
	cp obj_dir_init/Vsystem_top $@

clean:
	rm -f system system_init frame_*.ppm
	rm -rf obj_dir obj_dir_init
