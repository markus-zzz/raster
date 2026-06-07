// 2-port memory bus arbiter.
//   m0 = high priority (GPU)
//   m1 = low priority (display, etc.)
//
// The downstream sdram_ctrl only allows one outstanding read at a time
// (its `ready` deasserts while a read is in flight), so the arbiter only
// needs a single-bit owner tag to route rd_valid back to the correct master.

module arbiter #(
    parameter MEM_AW = 24
) (
    input  logic clk,
    input  logic rst,

    // Master 0 (high priority)
    input  logic [MEM_AW-1:0] m0_addr,
    input  logic              m0_req,
    input  logic              m0_we,
    input  logic [15:0]       m0_wr_data,
    output logic [15:0]       m0_rd_data,
    output logic              m0_rd_valid,
    output logic              m0_ready,

    // Master 1 (low priority)
    input  logic [MEM_AW-1:0] m1_addr,
    input  logic              m1_req,
    input  logic              m1_we,
    input  logic [15:0]       m1_wr_data,
    output logic [15:0]       m1_rd_data,
    output logic              m1_rd_valid,
    output logic              m1_ready,

    // Slave (sdram_ctrl)
    output logic [MEM_AW-1:0] s_addr,
    output logic              s_req,
    output logic              s_we,
    output logic [15:0]       s_wr_data,
    input  logic [15:0]       s_rd_data,
    input  logic              s_rd_valid,
    input  logic              s_ready
);

    // Combinational selection. When both have a request, m0 wins.
    // When neither has a request, default to m1 (doesn't matter; s_req=0).
    logic select; // 0 = m0, 1 = m1
    assign select = !m0_req;

    // Forward selected master to slave
    assign s_addr    = select ? m1_addr    : m0_addr;
    assign s_req     = select ? m1_req     : m0_req;
    assign s_we      = select ? m1_we      : m0_we;
    assign s_wr_data = select ? m1_wr_data : m0_wr_data;

    // Each master's `ready`. m0 has strict priority so it always gets the
    // bus when its req is asserted; we can pass s_ready straight through.
    // m1 is gated on m0 not requesting (otherwise it loses arbitration).
    assign m0_ready = s_ready;
    assign m1_ready = !m0_req && s_ready;

    // Track owner of the (at most one) outstanding read
    logic owner;
    always_ff @(posedge clk) begin
        if (rst) owner <= 0;
        else if (s_req && s_ready && !s_we)
            owner <= select;
    end

    // Steer rd_valid + rd_data back to the originator
    assign m0_rd_data  = s_rd_data;
    assign m1_rd_data  = s_rd_data;
    assign m0_rd_valid = s_rd_valid && !owner;
    assign m1_rd_valid = s_rd_valid &&  owner;

endmodule
