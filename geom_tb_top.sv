`default_nettype none

// Test wrapper: geom_front (sole master) -> sdram_ctrl -> sdram_model.
// Backdoor access to sdram.mem_inst.mem[] for preload/readback.
module geom_tb_top #(
    parameter int MEM_AW = 24,
    parameter int MATRIX_BASE = 0,
    parameter int LIGHT_BASE  = 0,
    parameter int VTX_BASE    = 0,
    parameter int FACE_BASE   = 0,
    parameter int TRI_BASE    = 0,
    parameter int BIN_BASE    = 0,
    parameter int BINLIST_BASE= 0
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        start,
    input  wire [15:0] nfaces,
    output wire        done
);
    wire [MEM_AW-1:0] gaddr;
    wire              greq, gwe, gwdreq, grdvalid, gready;
    wire [15:0]       gwdata, grdata;

    // SDRAM pins
    wire        s_clk, s_cke, s_cs_n, s_ras_n, s_cas_n, s_we_n, s_dqm;
    wire [1:0]  s_ba; wire [12:0] s_addr;
    wire [15:0] dq_ctrl_out, dq_model_out; wire dq_ctrl_oe, dq_model_oe;

    geom_front #(
        .MEM_AW(MEM_AW), .MATRIX_BASE(MATRIX_BASE), .LIGHT_BASE(LIGHT_BASE),
        .VTX_BASE(VTX_BASE), .FACE_BASE(FACE_BASE), .TRI_BASE(TRI_BASE),
        .BIN_BASE(BIN_BASE), .BINLIST_BASE(BINLIST_BASE)
    ) geom (
        .clk(clk), .rst(rst), .start(start), .nfaces(nfaces), .mat_index(16'd0), .done(done),
        .mem_addr(gaddr), .mem_req(greq), .mem_we(gwe), .mem_wr_data(gwdata),
        .mem_wr_data_req(gwdreq), .mem_rd_data(grdata), .mem_rd_valid(grdvalid),
        .mem_ready(gready)
    );

    sdram_ctrl #(.CLK_FREQ(100_000_000), .CAS_LATENCY(2)) ctrl (
        .clk(clk), .rst(rst),
        .addr(gaddr), .req(greq), .we(gwe), .wr_data(gwdata),
        .wr_data_req(gwdreq), .rd_data(grdata), .rd_valid(grdvalid), .ready(gready),
        .sdram_clk(s_clk), .sdram_cke(s_cke), .sdram_cs_n(s_cs_n), .sdram_ras_n(s_ras_n),
        .sdram_cas_n(s_cas_n), .sdram_we_n(s_we_n), .sdram_ba(s_ba), .sdram_addr(s_addr),
        .sdram_dqm(s_dqm), .sdram_dq_out(dq_ctrl_out), .sdram_dq_oe(dq_ctrl_oe),
        .sdram_dq_in(dq_model_out)
    );

    sdram_model #(.CAS_LATENCY(2)) sdram (
        .sdram_clk(s_clk), .sdram_cke(s_cke), .sdram_cs_n(s_cs_n), .sdram_ras_n(s_ras_n),
        .sdram_cas_n(s_cas_n), .sdram_we_n(s_we_n), .sdram_ba(s_ba), .sdram_addr(s_addr),
        .sdram_dqm(s_dqm), .sdram_dq_out(dq_model_out), .sdram_dq_oe(dq_model_oe),
        .sdram_dq_in(dq_ctrl_out)
    );
endmodule
