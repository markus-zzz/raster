`default_nettype none

// System top: rast_top + display_ctrl -> arbiter -> sdram_ctrl -> sdram_model
// TB uses backdoor access to sdram_model.mem_inst.mem[] for pre-loading
// and readback.

module gpu_top #(
    parameter FRAME_W   = 320,
    parameter FRAME_H   = 480,   // full 320x480 panel
    parameter TILE_W    = 64,
    parameter TILE_H    = 64,
    parameter SUBPIXEL  = 4,
    parameter MEM_AW    = 24,
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
    logic [MEM_AW-1:0] rast_addr;
    logic              rast_req;
    logic              rast_we;
    logic [15:0]       rast_wr_data;
    logic              rast_wr_data_req;
    logic [15:0]       rast_rd_data;
    logic              rast_rd_valid;
    logic              rast_ready;

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
    logic [MEM_AW-1:0] cpu_addr;
    logic              cpu_req;
    logic              cpu_we;
    logic [15:0]       cpu_wr_data;
    logic              cpu_wr_data_req;
    logic [15:0]       cpu_rd_data;
    logic              cpu_rd_valid;
    logic              cpu_ready;

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

    logic  geom_start, rast_start, geom_done, rast_done;



    //
    //
    //

    logic              reg_geom_ctrl_stat;
    logic [MEM_AW-1:0] reg_geom_desc_head;

    logic              reg_rast_ctrl_stat;
    logic [MEM_AW-1:0] reg_rast_tri_base;
    logic [MEM_AW-1:0] reg_rast_bin_base;
    logic [MEM_AW-1:0] reg_rast_binlist_base;
    logic [MEM_AW-1:0] reg_rast_fb_base;

    logic [MEM_AW-1:0] reg_disp_fb_base;

    always_ff @(posedge clk) begin
      if (rst | geom_done) reg_geom_ctrl_stat <= 1'b0;
      if (rst | rast_done) reg_rast_ctrl_stat <= 1'b0;

      if (cpu_mem_valid && cpu_mem_wstrb == 4'b1111) begin
        // Don't care to address decode all bits - register windows repeat
        casex (cpu_mem_addr)
          32'h2xxx_xx10: reg_geom_desc_head <= cpu_mem_wdata;
          32'h2xxx_xx14: reg_geom_ctrl_stat <= reg_geom_ctrl_stat | cpu_mem_wdata[0];
          32'h2xxx_xx20: reg_rast_tri_base <= cpu_mem_wdata;
          32'h2xxx_xx24: reg_rast_bin_base <= cpu_mem_wdata;
          32'h2xxx_xx28: reg_rast_binlist_base <= cpu_mem_wdata;
          32'h2xxx_xx2c: reg_rast_fb_base <= cpu_mem_wdata;
          32'h2xxx_xx30: reg_disp_fb_base <= cpu_mem_wdata;
          32'h2xxx_xx40: reg_rast_ctrl_stat <= reg_rast_ctrl_stat | cpu_mem_wdata[0];
        endcase
      end
    end

    logic disp_fb_base_written;
    always_comb begin
      disp_fb_base_written = 0;
      if (cpu_mem_valid && cpu_mem_wstrb == 4'b1111) begin
        // Don't care to address decode all bits - register windows repeat
        casex (cpu_mem_addr)
          32'h2xxx_xx30: disp_fb_base_written = 1;
        endcase
      end
    end

    assign geom_start = reg_geom_ctrl_stat;
    assign rast_start = reg_rast_ctrl_stat;

    geom_front #(
        .MEM_AW(MEM_AW),
        .W(FRAME_W), .H(FRAME_H),
        .NTX((FRAME_W + TILE_W - 1) / TILE_W),
        .NTY((FRAME_H + TILE_H - 1) / TILE_H),
        .TILE_W(TILE_W), .TILE_H(TILE_H),
        .MAX_FACES_PER_TILE(1024)
    ) u_geom_front (
        .clk(clk),
        .rst(rst),
        .start(geom_start),
        .desc_head(reg_geom_desc_head),
        .done(geom_done),
        .mem_addr(geom_addr),
        .mem_req(geom_req),
        .mem_we(geom_we),
        .mem_wr_data(geom_wr_data),
        .mem_wr_data_req(geom_wr_data_req),
        .mem_rd_data(geom_rd_data),
        .mem_rd_valid(geom_rd_valid),
        .mem_ready(geom_ready)
    );

    rast_front #(
        .FRAME_W(FRAME_W),
        .FRAME_H(FRAME_H),
        .TILE_W(TILE_W),
        .TILE_H(TILE_H),
        .SUBPIXEL(SUBPIXEL),
        .MEM_AW(MEM_AW)
    ) u_rast_front (
        .clk(clk),
        .rst(rst),
        .start(rast_start),
        .done(rast_done),
        .mem_addr(rast_addr),
        .mem_req(rast_req),
        .mem_we(rast_we),
        .mem_wr_data(rast_wr_data),
        .mem_wr_data_req(rast_wr_data_req),
        .mem_rd_data(rast_rd_data),
        .mem_rd_valid(rast_rd_valid),
        .mem_ready(rast_ready),
        .tri_base(reg_rast_tri_base),
        .bin_base(reg_rast_bin_base),
        .binlist_base(reg_rast_binlist_base),
        .fb_base(reg_rast_fb_base)
    );

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
      // Interrupts
      .irq({rast_done, geom_done, display_frame_start}),
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
      32'h2xxx_xx14: cpu_mem_rdata = reg_geom_ctrl_stat;
      32'h2xxx_xx40: cpu_mem_rdata = reg_rast_ctrl_stat;
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

    // ---- burst write engine (mirrors geom_front / rast_top) ----
    typedef enum logic [1:0] { M_IDLE, M_REQ, M_WR, M_RDY } mst_t;
    mst_t              mstate;
    logic [MEM_AW-1:0] burst_addr;
    logic [2:0]   beat;

    assign cpu_addr    = cpu_pcpi_rs2;
    assign cpu_we      = 1'b1;               // loader only ever writes
    assign cpu_req     = (mstate == M_REQ);
    assign cpu_wr_data = ~beat[0] ? ram_rdata[31:16] : ram_rdata[15:0];


    always_ff @(posedge clk) begin
        if (rst) begin
            mstate <= M_IDLE; beat <= 0;
        end else begin
            case (mstate)
                M_IDLE: if (cpu_pcpi_insn_wr_sdram) begin beat <= 0; mstate <= M_REQ; end
                M_REQ:  if (cpu_ready) begin beat <= 0; mstate <= M_WR; end
                M_WR:   if (cpu_wr_data_req) begin
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


    display_ctrl #(
        .MEM_AW(MEM_AW),
        .FB_LEN(FRAME_W * FRAME_H)
    ) display (
        .clk(clk),
        .rst(rst),
        .enable(display_enable),
        .pix_ce(display_pix_ce),
        .frame_start(display_frame_start | disp_fb_base_written),
        .mem_addr(disp_addr),
        .mem_req(disp_req),
        .mem_we(disp_we),
        .mem_wr_data(disp_wr_data),
        .mem_rd_data(disp_rd_data),
        .mem_rd_valid(disp_rd_valid),
        .mem_ready(disp_ready),
        .pix_data(display_pix_data),
        .pix_valid(display_pix_valid),
        .fb_base(reg_disp_fb_base)
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
        .m1_addr(rast_addr),
        .m1_req(rast_req),
        .m1_we(rast_we),
        .m1_wr_data(rast_wr_data),
        .m1_wr_data_req(rast_wr_data_req),
        .m1_rd_data(rast_rd_data),
        .m1_rd_valid(rast_rd_valid),
        .m1_ready(rast_ready),
        // m2 = GPU geometry
        .m2_addr(geom_addr),
        .m2_req(geom_req),
        .m2_we(geom_we),
        .m2_wr_data(geom_wr_data),
        .m2_wr_data_req(geom_wr_data_req),
        .m2_rd_data(geom_rd_data),
        .m2_rd_valid(geom_rd_valid),
        .m2_ready(geom_ready),
        // m3 = CPU DMA (lowest priority)
        .m3_addr(cpu_addr),
        .m3_req(cpu_req),
        .m3_we(cpu_we),
        .m3_wr_data(cpu_wr_data),
        .m3_wr_data_req(cpu_wr_data_req),
        .m3_rd_data(cpu_rd_data),
        .m3_rd_valid(cpu_rd_valid),
        .m3_ready(cpu_ready),
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
