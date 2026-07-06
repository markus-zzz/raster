`default_nettype none

// 3-port memory bus arbiter (burst-aware).
//   m0 = highest priority (GPU rasteriser)
//   m1 = geometry front-end
//   m2 = lowest priority (display)
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

    // Master 0 (highest priority)
    input  wire  [MEM_AW-1:0] m0_addr,
    input  wire               m0_req,
    input  wire               m0_we,
    input  wire  [15:0]       m0_wr_data,
    output logic              m0_wr_data_req,
    output logic [15:0]       m0_rd_data,
    output logic              m0_rd_valid,
    output logic              m0_ready,

    // Master 1
    input  wire  [MEM_AW-1:0] m1_addr,
    input  wire               m1_req,
    input  wire               m1_we,
    input  wire  [15:0]       m1_wr_data,
    output logic              m1_wr_data_req,
    output logic [15:0]       m1_rd_data,
    output logic              m1_rd_valid,
    output logic              m1_ready,

    // Master 2 (lowest priority)
    input  wire  [MEM_AW-1:0] m2_addr,
    input  wire               m2_req,
    input  wire               m2_we,
    input  wire  [15:0]       m2_wr_data,
    output logic              m2_wr_data_req,
    output logic [15:0]       m2_rd_data,
    output logic              m2_rd_valid,
    output logic              m2_ready,

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
    // follows the locked master regardless of req.
    logic        locked;
    logic [1:0]  lock_owner;   // 0=m0, 1=m1, 2=m2
    wire         accept = s_req && s_ready;

    // Fresh-request selection: strict priority m0 > m1 > m2.
    wire [1:0] new_sel = m0_req ? 2'd0 : m1_req ? 2'd1 : 2'd2;
    wire [1:0] sel     = locked ? lock_owner : new_sel;

    always_ff @(posedge clk) begin
        if (rst) begin
            locked     <= 1'b0;
            lock_owner <= 2'd0;
        end else if (!locked) begin
            if (accept) begin
                locked     <= 1'b1;
                lock_owner <= new_sel;
            end
        end else begin
            if (s_ready)
                locked <= 1'b0;
        end
    end

    // Forward the selected master to the slave. Suppress req while locked so
    // the controller sees a single request per burst.
    always_comb begin
        case (sel)
            2'd0:    begin s_addr = m0_addr; s_we = m0_we; s_wr_data = m0_wr_data; end
            2'd1:    begin s_addr = m1_addr; s_we = m1_we; s_wr_data = m1_wr_data; end
            default: begin s_addr = m2_addr; s_we = m2_we; s_wr_data = m2_wr_data; end
        endcase
    end
    assign s_req = locked ? 1'b0 : (m0_req | m1_req | m2_req);

    // Per-master ready: bus free and no higher-priority master requesting.
    assign m0_ready = !locked && s_ready;
    assign m1_ready = !locked && !m0_req && s_ready;
    assign m2_ready = !locked && !m0_req && !m1_req && s_ready;

    // Steer read data / write-data requests to the burst owner.
    assign m0_rd_data     = s_rd_data;
    assign m1_rd_data     = s_rd_data;
    assign m2_rd_data     = s_rd_data;
    assign m0_rd_valid    = s_rd_valid    && (sel == 2'd0);
    assign m1_rd_valid    = s_rd_valid    && (sel == 2'd1);
    assign m2_rd_valid    = s_rd_valid    && (sel == 2'd2);
    assign m0_wr_data_req = s_wr_data_req && (sel == 2'd0);
    assign m1_wr_data_req = s_wr_data_req && (sel == 2'd1);
    assign m2_wr_data_req = s_wr_data_req && (sel == 2'd2);

endmodule
