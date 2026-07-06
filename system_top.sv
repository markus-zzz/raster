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
    parameter SDRAM_INIT_FILE = ""   // optional $readmemh preload of SDRAM
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
    output logic        sdram_idle
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
    logic              sdram_clk;
    logic              sdram_cke;
    logic              sdram_cs_n;
    logic              sdram_ras_n;
    logic              sdram_cas_n;
    logic              sdram_we_n;
    logic [1:0]        sdram_ba;
    logic [12:0]       sdram_addr;
    logic              sdram_dqm;
    logic [15:0]       sdram_dq_ctrl_out;
    logic              sdram_dq_ctrl_oe;
    logic [15:0]       sdram_dq_model_out;
    logic              sdram_dq_model_oe;

    // DQ bus mux: controller drives on write, model drives on read
    wire [15:0] sdram_dq_to_ctrl  = sdram_dq_model_out;
    wire [15:0] sdram_dq_to_model = sdram_dq_ctrl_out;

    // Frame sequencing: geometry pass first, then rasterisation.
    typedef enum logic [1:0] { SYS_IDLE, SYS_GEOM, SYS_RASTER } sys_t;
    sys_t  sys_state;
    logic  geom_start, gpu_start, geom_done, gpu_done;
    logic [15:0] frame_ctr;   // selects the per-frame matrix (0..NUM_MATRICES-1)

    always_ff @(posedge clk) begin
        if (rst) begin
            sys_state <= SYS_IDLE; geom_start <= 0; gpu_start <= 0; done <= 0;
            frame_ctr <= 0;
        end else begin
            geom_start <= 0; gpu_start <= 0; done <= 0;
            case (sys_state)
                SYS_IDLE:   if (start) begin geom_start <= 1; sys_state <= SYS_GEOM; end
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
        // m2 = display (lowest priority)
        .m2_addr(disp_addr),
        .m2_req(disp_req),
        .m2_we(disp_we),
        .m2_wr_data(disp_wr_data),
        .m2_wr_data_req(disp_wr_data_req),
        .m2_rd_data(disp_rd_data),
        .m2_rd_valid(disp_rd_valid),
        .m2_ready(disp_ready),
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
        .CAS_LATENCY(2)
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
        .sdram_dq_out(sdram_dq_ctrl_out),
        .sdram_dq_oe(sdram_dq_ctrl_oe),
        .sdram_dq_in(sdram_dq_to_ctrl)
    );

    sdram_model #(
        .CAS_LATENCY(2),
        .INIT_FILE(SDRAM_INIT_FILE)
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
        .sdram_dq_in(sdram_dq_to_model)
    );

endmodule
