`default_nettype none

// System top: gpu_top + display_ctrl -> arbiter -> sdram_ctrl -> sdram_model
// TB uses backdoor access to sdram_model.mem_inst.mem[] for pre-loading
// and readback.

module system_top #(
    parameter FRAME_W   = 320,
    parameter FRAME_H   = 200,
    parameter TILE_W    = 64,
    parameter TILE_H    = 64,
    parameter SUBPIXEL  = 4,
    parameter MEM_AW    = 24,
    parameter NUM_MATRICES = 90,      // animation: matrices cycled one per frame
    parameter INPUT_WORDS  = 23680,   // halfwords of read-only inputs (mult. of 8)
    parameter INPUT_ROM_FILE = "sdram_inputs.hex", // on-chip ROM image -> SDRAM at startup
    parameter SIM_MODEL    = 1,       // 1: internal behavioural SDRAM model (sim)
                                      // 0: drive the external SDRAM pins (FPGA)
    parameter TEST_MODE    = 0,       // 0: normal render
                                      // 1: framebuffer colour-bar pattern
                                      // 2: self-checking SDRAM memory test
    parameter READ_LAT_ADJ = 3        // SDRAM read-capture latency. 3 matches the
                                      // behavioural model (sim); the real chip
                                      // needs a value found on hardware (set via
                                      // top.sv for the FPGA build).
) (
    input  wire clk,
    input  wire rst,
    // GPU control
    input  wire start,
    input  wire [15:0] nfaces,   // number of faces for the geometry pass
    output logic done,
    // Display controller enable (TB toggles this to apply bus load)
    input  wire display_enable,
    input  wire display_frame_start,
    input  wire display_pix_ce,
    output logic [15:0] display_pix_data,
    output logic        display_pix_valid,
    // SDRAM bus ready (high while controller is idle, no requests in flight,
    // no read pipeline data outstanding). TB uses this to know when it is
    // safe to restart the display capture.
    output logic        sdram_idle,

    // External SDRAM interface. The controller always drives these; on FPGA
    // (SIM_MODEL=0) they go to the physical pins, in simulation (SIM_MODEL=1)
    // they are wired to an internal behavioural model instead.
    output logic        sdram_clk,
    output logic        sdram_cke,
    output logic        sdram_cs_n,
    output logic        sdram_ras_n,
    output logic        sdram_cas_n,
    output logic        sdram_we_n,
    output logic [1:0]  sdram_ba,
    output logic [12:0] sdram_addr,
    output logic        sdram_dqm,
    output logic [15:0] sdram_dq_out,   // controller -> pins (write data)
    output logic        sdram_dq_oe,    // controller drives DQ this cycle
    input  wire  [15:0] sdram_dq_in     // pins -> controller (read data)
);

    // GPU master
    logic [MEM_AW-1:0] gpu_addr;
    logic              gpu_req;
    logic              gpu_we;
    logic [15:0]       gpu_wr_data;
    logic              gpu_wr_data_req;
    logic [15:0]       gpu_rd_data;
    logic              gpu_rd_valid;
    logic              gpu_ready;

    // Display master
    logic [MEM_AW-1:0] disp_addr;
    logic              disp_req;
    logic              disp_we;
    logic [15:0]       disp_wr_data;
    logic              disp_wr_data_req;
    logic [15:0]       disp_rd_data;
    logic              disp_rd_valid;
    logic              disp_ready;

    // m1 (geometry) master
    logic [MEM_AW-1:0] geom_addr;
    logic              geom_req;
    logic              geom_we;
    logic [15:0]       geom_wr_data;
    logic              geom_wr_data_req;
    logic [15:0]       geom_rd_data;
    logic              geom_rd_valid;
    logic              geom_ready;

    // m3 (startup SDRAM loader) master
    logic [MEM_AW-1:0] load_addr;
    logic              load_req;
    logic              load_we;
    logic [15:0]       load_wr_data;
    logic              load_wr_data_req;
    logic [15:0]       load_rd_data;
    logic              load_rd_valid;
    logic              load_ready;
    logic              load_start, load_done;

    // Arbiter -> SDRAM controller
    logic [MEM_AW-1:0] mem_addr;
    logic              mem_req;
    logic              mem_we;
    logic [15:0]       mem_wr_data;
    logic              mem_wr_data_req;
    logic [15:0]       mem_rd_data;
    logic              mem_rd_valid;
    logic              mem_ready;

    // SDRAM physical signals
    // DQ read source for the controller: internal model in sim, external pins
    // on FPGA. The controller's write outputs (sdram_dq_out/oe) are ports.
    logic [15:0] sdram_dq_model_out;
    logic        sdram_dq_model_oe;
    wire  [15:0] sdram_dq_to_ctrl = SIM_MODEL ? sdram_dq_model_out : sdram_dq_in;

    // Frame sequencing: one-time SDRAM load, then geometry pass, then raster.
    typedef enum logic [1:0] { SYS_IDLE, SYS_LOAD, SYS_GEOM, SYS_RASTER } sys_t;
    sys_t  sys_state;
    logic  geom_start, gpu_start, geom_done, gpu_done;
    logic  loaded;            // inputs have been copied into SDRAM
    logic [15:0] frame_ctr;   // selects the per-frame matrix (0..NUM_MATRICES-1)

    always_ff @(posedge clk) begin
        if (rst) begin
            sys_state <= SYS_IDLE; geom_start <= 0; gpu_start <= 0; done <= 0;
            frame_ctr <= 0; loaded <= 0; load_start <= 0;
        end else begin
            geom_start <= 0; gpu_start <= 0; done <= 0; load_start <= 0;
            case (sys_state)
                SYS_IDLE:   if (start) begin
                                if (!loaded) begin load_start <= 1; sys_state <= SYS_LOAD; end
                                else if (TEST_MODE != 0) done <= 1;  // static FB, nothing to redo
                                else         begin geom_start <= 1; sys_state <= SYS_GEOM; end
                            end
                SYS_LOAD:   if (load_done) begin
                                loaded <= 1;
                                if (TEST_MODE != 0) begin done <= 1; sys_state <= SYS_IDLE; end
                                else begin geom_start <= 1; sys_state <= SYS_GEOM; end
                            end
                SYS_GEOM:   if (geom_done) begin gpu_start <= 1; sys_state <= SYS_RASTER; end
                SYS_RASTER: if (gpu_done) begin
                                done <= 1;
                                frame_ctr <= (frame_ctr == NUM_MATRICES-1) ? 16'd0
                                                                           : frame_ctr + 16'd1;
                                sys_state <= SYS_IDLE;
                            end
                default:    sys_state <= SYS_IDLE;
            endcase
        end
    end

    geom_front #(
        .MEM_AW(MEM_AW),
        .NTX((FRAME_W + TILE_W - 1) / TILE_W),
        .NTY((FRAME_H + TILE_H - 1) / TILE_H),
        .TILE_W(TILE_W), .TILE_H(TILE_H),
        .MAX_FACES_PER_TILE(1024),
        .MATRIX_STRIDE(24),
        .MATRIX_BASE(24'h02_0000), .LIGHT_BASE(24'h02_0880),
        .VTX_BASE(24'h02_1000),    .FACE_BASE(24'h02_2000),
        .TRI_BASE(24'h00_0000),    .BIN_BASE(24'h00_4000),
        .BINLIST_BASE(24'h00_5000)
    ) geom (
        .clk(clk), .rst(rst), .start(geom_start), .nfaces(nfaces),
        .mat_index(frame_ctr), .done(geom_done),
        .mem_addr(geom_addr), .mem_req(geom_req), .mem_we(geom_we),
        .mem_wr_data(geom_wr_data), .mem_wr_data_req(geom_wr_data_req),
        .mem_rd_data(geom_rd_data), .mem_rd_valid(geom_rd_valid), .mem_ready(geom_ready)
    );

    // m2 master: normally the startup loader; in a TEST_MODE, a framebuffer
    // colour-bar writer (1) or a self-checking memory test (2) instead. All are
    // one-shot startup masters driven by load_start/load_done onto the same
    // arbiter port.
    generate if (TEST_MODE == 1) begin : g_pattern
        fb_pattern #(
            .MEM_AW(MEM_AW),
            .FRAME_W(FRAME_W),
            .FRAME_H(FRAME_H),
            .FB_BASE(24'h00_A000)
        ) writer (
            .clk(clk), .rst(rst), .start(load_start), .done(load_done),
            .mem_addr(load_addr), .mem_req(load_req), .mem_we(load_we),
            .mem_wr_data(load_wr_data), .mem_wr_data_req(load_wr_data_req),
            .mem_rd_data(load_rd_data), .mem_rd_valid(load_rd_valid), .mem_ready(load_ready)
        );
    end else if (TEST_MODE == 2) begin : g_memtest
        sdram_memtest #(
            .MEM_AW(MEM_AW),
            .FRAME_W(FRAME_W),
            .FRAME_H(FRAME_H),
            .TEST_BASE(24'h00_0000),
            .TEST_WORDS(32'h0002_8000),
            .FB_BASE(24'h00_A000)
        ) memtest (
            .clk(clk), .rst(rst), .start(load_start), .done(load_done),
            .mem_addr(load_addr), .mem_req(load_req), .mem_we(load_we),
            .mem_wr_data(load_wr_data), .mem_wr_data_req(load_wr_data_req),
            .mem_rd_data(load_rd_data), .mem_rd_valid(load_rd_valid), .mem_ready(load_ready)
        );
    end else begin : g_loader
        sdram_loader #(
            .MEM_AW(MEM_AW),
            .INPUT_WORDS(INPUT_WORDS),
            .INPUT_BASE(24'h02_0000),
            .ROM_FILE(INPUT_ROM_FILE)
        ) loader (
            .clk(clk), .rst(rst), .start(load_start), .done(load_done),
            .mem_addr(load_addr), .mem_req(load_req), .mem_we(load_we),
            .mem_wr_data(load_wr_data), .mem_wr_data_req(load_wr_data_req),
            .mem_rd_data(load_rd_data), .mem_rd_valid(load_rd_valid), .mem_ready(load_ready)
        );
    end endgenerate

    gpu_top #(
        .FRAME_W(FRAME_W),
        .FRAME_H(FRAME_H),
        .TILE_W(TILE_W),
        .TILE_H(TILE_H),
        .SUBPIXEL(SUBPIXEL),
        .MEM_AW(MEM_AW)
    ) gpu (
        .clk(clk),
        .rst(rst),
        .start(gpu_start),
        .done(gpu_done),
        .mem_addr(gpu_addr),
        .mem_req(gpu_req),
        .mem_we(gpu_we),
        .mem_wr_data(gpu_wr_data),
        .mem_wr_data_req(gpu_wr_data_req),
        .mem_rd_data(gpu_rd_data),
        .mem_rd_valid(gpu_rd_valid),
        .mem_ready(gpu_ready)
    );

    display_ctrl #(
        .MEM_AW(MEM_AW),
        .FB_BASE(24'h00_A000),
        .FB_LEN(FRAME_W * FRAME_H)
    ) display (
        .clk(clk),
        .rst(rst),
        .enable(display_enable),
        .pix_ce(display_pix_ce),
        .frame_start(display_frame_start),
        .mem_addr(disp_addr),
        .mem_req(disp_req),
        .mem_we(disp_we),
        .mem_wr_data(disp_wr_data),
        .mem_rd_data(disp_rd_data),
        .mem_rd_valid(disp_rd_valid),
        .mem_ready(disp_ready),
        .pix_data(display_pix_data),
        .pix_valid(display_pix_valid)
    );

    arbiter #(
        .MEM_AW(MEM_AW)
    ) arb (
        .clk(clk),
        .rst(rst),
        // m0 = GPU (highest priority)
        .m0_addr(gpu_addr),
        .m0_req(gpu_req),
        .m0_we(gpu_we),
        .m0_wr_data(gpu_wr_data),
        .m0_wr_data_req(gpu_wr_data_req),
        .m0_rd_data(gpu_rd_data),
        .m0_rd_valid(gpu_rd_valid),
        .m0_ready(gpu_ready),
        // m1 = geometry front-end
        .m1_addr(geom_addr),
        .m1_req(geom_req),
        .m1_we(geom_we),
        .m1_wr_data(geom_wr_data),
        .m1_wr_data_req(geom_wr_data_req),
        .m1_rd_data(geom_rd_data),
        .m1_rd_valid(geom_rd_valid),
        .m1_ready(geom_ready),
        // m2 = startup loader (above display so it isn't starved at boot; it
        // is idle during rendering, so display keeps its effective priority)
        .m2_addr(load_addr),
        .m2_req(load_req),
        .m2_we(load_we),
        .m2_wr_data(load_wr_data),
        .m2_wr_data_req(load_wr_data_req),
        .m2_rd_data(load_rd_data),
        .m2_rd_valid(load_rd_valid),
        .m2_ready(load_ready),
        // m3 = display (lowest priority)
        .m3_addr(disp_addr),
        .m3_req(disp_req),
        .m3_we(disp_we),
        .m3_wr_data(disp_wr_data),
        .m3_wr_data_req(disp_wr_data_req),
        .m3_rd_data(disp_rd_data),
        .m3_rd_valid(disp_rd_valid),
        .m3_ready(disp_ready),
        // Slave
        .s_addr(mem_addr),
        .s_req(mem_req),
        .s_we(mem_we),
        .s_wr_data(mem_wr_data),
        .s_wr_data_req(mem_wr_data_req),
        .s_rd_data(mem_rd_data),
        .s_rd_valid(mem_rd_valid),
        .s_ready(mem_ready)
    );

    assign sdram_idle = mem_ready && !mem_req;

    sdram_ctrl #(
        .CLK_FREQ(100_000_000),
        .CAS_LATENCY(2),
        .READ_LAT_ADJ(READ_LAT_ADJ)
    ) ctrl (
        .clk(clk),
        .rst(rst),
        .addr(mem_addr),
        .req(mem_req),
        .we(mem_we),
        .wr_data(mem_wr_data),
        .wr_data_req(mem_wr_data_req),
        .rd_data(mem_rd_data),
        .rd_valid(mem_rd_valid),
        .ready(mem_ready),
        .sdram_clk(sdram_clk),
        .sdram_cke(sdram_cke),
        .sdram_cs_n(sdram_cs_n),
        .sdram_ras_n(sdram_ras_n),
        .sdram_cas_n(sdram_cas_n),
        .sdram_we_n(sdram_we_n),
        .sdram_ba(sdram_ba),
        .sdram_addr(sdram_addr),
        .sdram_dqm(sdram_dqm),
        .sdram_dq_out(sdram_dq_out),
        .sdram_dq_oe(sdram_dq_oe),
        .sdram_dq_in(sdram_dq_to_ctrl)
    );

    // Behavioural SDRAM chip model for simulation only. Starts empty (like the
    // real volatile chip); the startup loader populates it. On FPGA
    // (SIM_MODEL=0) it is omitted and the controller drives the real pins.
    generate if (SIM_MODEL) begin : g_model
        sdram_model #(
            .CAS_LATENCY(2),
            .INIT_FILE("")
        ) sdram (
            .sdram_clk(sdram_clk),
            .sdram_cke(sdram_cke),
            .sdram_cs_n(sdram_cs_n),
            .sdram_ras_n(sdram_ras_n),
            .sdram_cas_n(sdram_cas_n),
            .sdram_we_n(sdram_we_n),
            .sdram_ba(sdram_ba),
            .sdram_addr(sdram_addr),
            .sdram_dqm(sdram_dqm),
            .sdram_dq_out(sdram_dq_model_out),
            .sdram_dq_oe(sdram_dq_model_oe),
            .sdram_dq_in(sdram_dq_out)   // controller write data
        );
    end else begin : g_no_model
        assign sdram_dq_model_out = 16'b0;
        assign sdram_dq_model_oe  = 1'b0;
    end endgenerate

endmodule
