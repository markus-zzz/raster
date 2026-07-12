CXX = g++
VERILATOR = ~/work/install/bin/verilator

SRCS = system_top.sv gpu_top.sv raster_top.sv rasterizer.sv spram.sv dpram.sv \
       sdram_ctrl.sv sdram_model.sv arbiter.sv display_ctrl.sv \
       geom_front.sv geom_engine.sv sdram_loader.sv fb_pattern.sv sdram_memtest.sv \
       picorv32.v tb_system.cpp

VFLAGS = --cc --exe --build -j 0 --trace-fst -Wno-fatal \
    -Wno-UNUSEDPARAM -Wno-UNUSEDSIGNAL -Wno-WIDTHTRUNC -Wno-WIDTHEXPAND \
    -Wno-DECLFILENAME \
    --top-module system_top

all: run

# 1) Verilate + build the sim executable (does not need the hex files yet;
#    the CPU ROM is loaded from bios.vh at sim runtime via $readmemh).
system: $(SRCS)
	$(VERILATOR) $(VFLAGS) -CFLAGS '-std=c++20 -O3' \
		$(SRCS) -o Vsystem_top
	cp obj_dir/Vsystem_top $@

# 2) The testbench emits the firmware's static-input header (mesh + light).
#    (--emit-inputs returns before constructing the DUT, so bios.vh isn't needed.)
sdram_inputs.hex: system suzanne.obj
	./system --emit-inputs

# 3) Build the CPU ROM (bios.vh) from the firmware + generated header + sine LUT.
bios.vh: sdram_inputs.hex fw/main.c fw/start.S fw/gen_sintab.py fw/system.ld fw/Makefile
	cp sdram_inputs.hex fw/
	$(MAKE) -C fw
	cp fw/bios.vh .

# 4) Run the sim: the CPU loads SDRAM, then computes+publishes a matrix per
#    frame via the handshake; the render pipeline draws each frame.
run: system bios.vh
	./system

clean:
	rm -f system frame_*.ppm sdram_inputs.hex bios.vh
	rm -rf obj_dir
	-$(MAKE) -C fw clean
