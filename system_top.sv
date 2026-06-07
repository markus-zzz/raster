// System top: gpu_top -> sdram_ctrl -> sdram_model
// TB uses backdoor access to sdram_model.mem[] for pre-loading and readback

module system_top #(
    parameter FRAME_W   = 320,
    parameter FRAME_H   = 200,
    parameter TILE_W    = 64,
    parameter TILE_H    = 64,
    parameter SUBPIXEL  = 4,
    parameter MEM_AW    = 24
) (
    input  logic clk,
    input  logic rst,
    // GPU control
    input  logic start,
    output logic done
);

    // GPU <-> SDRAM controller
    logic [MEM_AW-1:0] mem_addr;
    logic              mem_req;
    logic              mem_we;
    logic [15:0]       mem_wr_data;
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
        .start(start),
        .done(done),
        .mem_addr(mem_addr),
        .mem_req(mem_req),
        .mem_we(mem_we),
        .mem_wr_data(mem_wr_data),
        .mem_rd_data(mem_rd_data),
        .mem_rd_valid(mem_rd_valid),
        .mem_ready(mem_ready)
    );

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
        .CAS_LATENCY(2)
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
