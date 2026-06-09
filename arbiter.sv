`default_nettype none

// 2-port memory bus arbiter (burst-aware).
//   m0 = high priority (GPU)
//   m1 = low priority (display, etc.)
//
// Each access is a fixed-length burst handled end-to-end by sdram_ctrl. The
// arbiter latches the winning master at accept time and holds the selection
// for the whole burst (until the controller is `ready` again), so read beats
// and write-data requests route to the correct master throughout.

module arbiter #(
    parameter MEM_AW = 24
) (
    input  wire  clk,
    input  wire  rst,

    // Master 0 (high priority)
    input  wire  [MEM_AW-1:0] m0_addr,
    input  wire               m0_req,
    input  wire               m0_we,
    input  wire  [15:0]       m0_wr_data,
    output logic              m0_wr_data_req,
    output logic [15:0]       m0_rd_data,
    output logic              m0_rd_valid,
    output logic              m0_ready,

    // Master 1 (low priority)
    input  wire  [MEM_AW-1:0] m1_addr,
    input  wire               m1_req,
    input  wire               m1_we,
    input  wire  [15:0]       m1_wr_data,
    output logic              m1_wr_data_req,
    output logic [15:0]       m1_rd_data,
    output logic              m1_rd_valid,
    output logic              m1_ready,

    // Slave (sdram_ctrl)
    output logic [MEM_AW-1:0] s_addr,
    output logic              s_req,
    output logic              s_we,
    output logic [15:0]       s_wr_data,
    input  wire               s_wr_data_req,
    input  wire  [15:0]       s_rd_data,
    input  wire               s_rd_valid,
    input  wire               s_ready
);

    // Burst ownership latch. While a burst is in flight (locked), all routing
    // follows the locked master regardless of req. A new burst is accepted
    // when the controller is ready and some master requests.
    logic       locked;
    logic       lock_owner;   // 0 = m0, 1 = m1
    wire        accept = s_req && s_ready;

    // Combinational selection for a fresh request: m0 wins when both ask.
    wire        new_sel = !m0_req;            // 0 = m0, 1 = m1
    wire        sel = locked ? lock_owner : new_sel;

    always_ff @(posedge clk) begin
        if (rst) begin
            locked     <= 1'b0;
            lock_owner <= 1'b0;
        end else if (!locked) begin
            if (accept) begin
                locked     <= 1'b1;
                lock_owner <= new_sel;
            end
        end else begin
            // Release the lock when the controller becomes ready again
            // (burst complete) and no back-to-back accept is happening.
            if (s_ready)
                locked <= 1'b0;
        end
    end

    // Forward the selected master to the slave. Suppress req while locked so
    // the controller sees a single request per burst.
    assign s_addr    = sel ? m1_addr    : m0_addr;
    assign s_req     = locked ? 1'b0 : (sel ? m1_req : m0_req);
    assign s_we      = sel ? m1_we      : m0_we;
    assign s_wr_data = sel ? m1_wr_data : m0_wr_data;

    // Each master's `ready`: only when the bus is free (not locked) and the
    // controller can accept. m0 has strict priority.
    assign m0_ready = !locked && s_ready;
    assign m1_ready = !locked && !m0_req && s_ready;

    // Steer read data and write-data requests to the burst owner.
    assign m0_rd_data     = s_rd_data;
    assign m1_rd_data     = s_rd_data;
    assign m0_rd_valid    = s_rd_valid    && (sel == 1'b0);
    assign m1_rd_valid    = s_rd_valid    && (sel == 1'b1);
    assign m0_wr_data_req = s_wr_data_req && (sel == 1'b0);
    assign m1_wr_data_req = s_wr_data_req && (sel == 1'b1);

endmodule
