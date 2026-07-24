`default_nettype none

// System top: gpu_top + display_ctrl -> arbiter -> sdram_ctrl -> sdram_model
// TB uses backdoor access to sdram_model.mem_inst.mem[] for pre-loading
// and readback.

module system_top #(
    parameter FRAME_W   = 320,
    parameter FRAME_H   = 480,   // full 320x480 panel
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
    logic              load_done;

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

    // Frame sequencing driven by the CPU handshake: the CPU writes the matrix
    // for the next frame into SDRAM and pulses "matrix ready" (an MMIO write);
    // the sequencer runs the geometry + raster passes for that matrix and bumps
    // frame_count, which the CPU polls to learn the frame was consumed+rendered.
    typedef enum logic [1:0] { SYS_IDLE, SYS_GEOM, SYS_RASTER } sys_t;
    sys_t  sys_state;
    logic  geom_start, gpu_start, geom_done, gpu_done;
    logic         matrix_ready;   // CPU has a fresh matrix waiting in SDRAM
    logic [31:0]  frame_count;    // completed frames (polled by the CPU)
    // CPU MMIO write to the control region (0x2xxx_xxxx) raises matrix_ready.
    wire cpu_mmio_wr = cpu_mem_valid && (cpu_mem_addr[31:28] == 4'h2)
                                     && (|cpu_mem_wstrb);

    always_ff @(posedge clk) begin
        if (rst) begin
            sys_state <= SYS_IDLE; geom_start <= 0; gpu_start <= 0; done <= 0;
            matrix_ready <= 0; frame_count <= 0;
        end else begin
            geom_start <= 0; gpu_start <= 0; done <= 0;
            if (cpu_mmio_wr) matrix_ready <= 1;   // set (clear below wins on collision)
            case (sys_state)
                SYS_IDLE:   if (start && matrix_ready) begin
                                matrix_ready <= 0;         // consume the matrix
                                geom_start   <= 1;
                                sys_state    <= SYS_GEOM;
                            end
                SYS_GEOM:   if (geom_done) begin gpu_start <= 1; sys_state <= SYS_RASTER; end
                SYS_RASTER: if (gpu_done) begin
                                done        <= 1;
                                frame_count <= frame_count + 32'd1;
                                sys_state   <= SYS_IDLE;
                            end
                default:    sys_state <= SYS_IDLE;
            endcase
        end
    end

    geom_front #(
        .MEM_AW(MEM_AW),
        .W(FRAME_W), .H(FRAME_H),
        .NTX((FRAME_W + TILE_W - 1) / TILE_W),
        .NTY((FRAME_H + TILE_H - 1) / TILE_H),
        .TILE_W(TILE_W), .TILE_H(TILE_H),
        .MAX_FACES_PER_TILE(1024)
    ) geom (
        .clk(clk), .rst(rst), .start(geom_start),
        .desc_head(24'h02_0040), .done(geom_done),   // CPU builds the descriptor list here
        .mem_addr(geom_addr), .mem_req(geom_req), .mem_we(geom_we),
        .mem_wr_data(geom_wr_data), .mem_wr_data_req(geom_wr_data_req),
        .mem_rd_data(geom_rd_data), .mem_rd_valid(geom_rd_valid), .mem_ready(geom_ready)
    );

  /*
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
  */

  // XXX: Put the entire CPU subsystem in its own module
  logic cpu_mem_valid;
  logic cpu_mem_instr;
  logic cpu_mem_ready;
  logic [31:0] cpu_mem_addr;
  logic [31:0] cpu_mem_wdata;
  logic [3:0]  cpu_mem_wstrb;
  logic [31:0] cpu_mem_rdata;
  logic [31:0] ram_rdata;
  logic [31:0] rom_rdata;

  logic        cpu_pcpi_valid;
  logic [31:0] cpu_pcpi_insn;
  logic [31:0] cpu_pcpi_rs1;
  logic [31:0] cpu_pcpi_rs2;
  logic        cpu_pcpi_wait;
  logic        cpu_pcpi_ready;

  logic cpu_pcpi_insn_wr_sdram;
  assign cpu_pcpi_insn_wr_sdram = cpu_pcpi_valid && (cpu_pcpi_insn == 32'h44b5100b);

  assign load_done = 1;

  // CPU ROM
  spram2 #(
      .ADDR_WIDTH(15),
      .DATA_WIDTH(32),
      .INIT_FILE("bios.vh")
  ) u_rom (
      .clk (clk),
      .addr(cpu_mem_addr[31:2]),
      .rd_data(rom_rdata),
      .wr_en(1'b0)
  );

  // CPU RAM
  genvar gi;
  generate
    for (gi = 0; gi < 4; gi = gi + 1) begin : ram
      spram2 #(
          .ADDR_WIDTH(10),
          .DATA_WIDTH(8)
      ) u_ram (
          .clk (clk),
          .addr(mstate == M_REQ || mstate == M_WR ? {cpu_pcpi_rs1[31:4], beat[2:1]} : cpu_mem_addr[31:2]),
          .rd_data(ram_rdata[(gi+1)*8-1:gi*8]),
          .wr_data(cpu_mem_wdata[(gi+1)*8-1:gi*8]),
          .wr_en  (cpu_mem_wstrb[gi] && (cpu_mem_valid && cpu_mem_addr[31:28] == 4'h1))
      );
    end
  endgenerate

  // CPU
  picorv32 #(
      .COMPRESSED_ISA(1),
      .ENABLE_PCPI(1),
      .ENABLE_IRQ(1),
      .ENABLE_MUL(1),
      .ENABLE_DIV(1)
  ) u_cpu (
      .clk(clk),
      .resetn(~rst),
      // Pico Co-Processor Interface (PCPI)
      .pcpi_valid(cpu_pcpi_valid),
      .pcpi_insn (cpu_pcpi_insn),
      .pcpi_rs1  (cpu_pcpi_rs1),
      .pcpi_rs2  (cpu_pcpi_rs2),
      .pcpi_wr   (1'b0),
      .pcpi_rd   (32'h0),
      .pcpi_wait (cpu_pcpi_wait),
      .pcpi_ready(cpu_pcpi_ready),
      // PicoRV32 Native Memory Interface
      .mem_valid(cpu_mem_valid),
      .mem_instr(cpu_mem_instr),
      .mem_ready(cpu_mem_ready),
      .mem_addr (cpu_mem_addr),
      .mem_wdata(cpu_mem_wdata),
      .mem_wstrb(cpu_mem_wstrb),
      .mem_rdata(cpu_mem_rdata)
  );

  always_comb begin
    casex (cpu_mem_addr)
      32'h0xxx_xxxx: cpu_mem_rdata = rom_rdata;
      32'h1xxx_xxxx: cpu_mem_rdata = ram_rdata;
      32'h2xxx_xxxx: cpu_mem_rdata = frame_count;   // MMIO: completed-frame count
      default: cpu_mem_rdata = 0;
    endcase
  end

  always_ff @(posedge clk) begin
    if (rst) cpu_mem_ready <= 0;
    else begin
      casex (cpu_mem_addr)
        32'h0xxx_xxxx: cpu_mem_ready <= ~cpu_mem_ready & cpu_mem_valid;
        32'h1xxx_xxxx: cpu_mem_ready <= ~cpu_mem_ready & cpu_mem_valid;
        32'h2xxx_xxxx: cpu_mem_ready <= ~cpu_mem_ready & cpu_mem_valid;
        default:       cpu_mem_ready <= 0;
      endcase
    end
  end

    // ---- burst write engine (mirrors geom_front / gpu_top) ----
    typedef enum logic [1:0] { M_IDLE, M_REQ, M_WR, M_RDY } mst_t;
    mst_t              mstate;
    logic [MEM_AW-1:0] burst_addr;
    logic [2:0]   beat;

    assign load_addr    = cpu_pcpi_rs2;
    assign load_we      = 1'b1;               // loader only ever writes
    assign load_req     = (mstate == M_REQ);
    assign load_wr_data = ~beat[0] ? ram_rdata[31:16] : ram_rdata[15:0];


    always_ff @(posedge clk) begin
        if (rst) begin
            mstate <= M_IDLE; beat <= 0;
        end else begin
            case (mstate)
                M_IDLE: if (cpu_pcpi_insn_wr_sdram) begin beat <= 0; mstate <= M_REQ; end
                M_REQ:  if (load_ready) begin beat <= 0; mstate <= M_WR; end
                M_WR:   if (load_wr_data_req) begin
                            beat <= beat + 1'b1;
                            if (beat == 7) begin mstate <= M_RDY; end
                        end
                M_RDY: mstate <= M_IDLE;
                default: mstate <= M_IDLE;
            endcase
        end
    end

    assign cpu_pcpi_wait = (mstate != M_IDLE);
    assign cpu_pcpi_ready = (mstate == M_RDY);


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
        .FB_BASE(24'h03_0000),
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
        // m0 = display (highest priority: must not starve or the screen tears)
        .m0_addr(disp_addr),
        .m0_req(disp_req),
        .m0_we(disp_we),
        .m0_wr_data(disp_wr_data),
        .m0_wr_data_req(disp_wr_data_req),
        .m0_rd_data(disp_rd_data),
        .m0_rd_valid(disp_rd_valid),
        .m0_ready(disp_ready),
        // m1 = GPU rasteriser
        .m1_addr(gpu_addr),
        .m1_req(gpu_req),
        .m1_we(gpu_we),
        .m1_wr_data(gpu_wr_data),
        .m1_wr_data_req(gpu_wr_data_req),
        .m1_rd_data(gpu_rd_data),
        .m1_rd_valid(gpu_rd_valid),
        .m1_ready(gpu_ready),
        // m2 = geometry front-end
        .m2_addr(geom_addr),
        .m2_req(geom_req),
        .m2_we(geom_we),
        .m2_wr_data(geom_wr_data),
        .m2_wr_data_req(geom_wr_data_req),
        .m2_rd_data(geom_rd_data),
        .m2_rd_valid(geom_rd_valid),
        .m2_ready(geom_ready),
        // m3 = CPU DMA (lowest priority: matrix writes are small, and the
        // CPU handshake tolerates delay -- it spins on frame_count anyway)
        .m3_addr(load_addr),
        .m3_req(load_req),
        .m3_we(load_we),
        .m3_wr_data(load_wr_data),
        .m3_wr_data_req(load_wr_data_req),
        .m3_rd_data(load_rd_data),
        .m3_rd_valid(load_rd_valid),
        .m3_ready(load_ready),
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
