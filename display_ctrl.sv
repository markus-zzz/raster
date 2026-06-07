// Mocked display controller with pixel FIFO between SDRAM reads (producer)
// and pixel-clock consumption (consumer).
//
//   Producer (reads from SDRAM):
//     Issues a read whenever the FIFO is non-full and no read is in flight.
//     Pushes returned data into the FIFO.
//
//   Consumer (mocked display output):
//     Pops one pixel every 8 system cycles (12.5 MHz from 100 MHz).
//     Data is discarded; only used to maintain steady drain rate.
//
// FIFO_DEPTH should be large enough to absorb SDRAM access latency and any
// arbitration jitter. With CAS_LATENCY=2 + small overhead, ~16 entries is
// plenty for filling-the-pipeline latency, though it won't prevent underrun
// during long GPU bursts (the display would just stall, equivalent to the
// underrun visual on real hardware).

module display_ctrl #(
    parameter MEM_AW     = 24,
    parameter FB_BASE    = 24'h00_A000,
    parameter FB_LEN     = 64000,   // 320 * 200
    parameter FIFO_DEPTH = 16
) (
    input  logic clk,
    input  logic rst,
    input  logic enable,
    // Pulse high to restart fetching at FB_BASE+0. Caller must hold enable=0
    // long enough beforehand that no read is in flight (pending == 0).
    input  logic frame_start,

    // Memory bus master interface
    output logic [MEM_AW-1:0] mem_addr,
    output logic              mem_req,
    output logic              mem_we,
    output logic [15:0]       mem_wr_data,
    input  logic [15:0]       mem_rd_data,
    input  logic              mem_rd_valid,
    input  logic              mem_ready,

    // Pixel output (one valid pulse per popped pixel)
    output logic [15:0]       pix_data,
    output logic              pix_valid
);

    localparam FIFO_AW = $clog2(FIFO_DEPTH);

    // ---- Pixel-rate clock enable (12.5 MHz from 100 MHz) ----
    logic [2:0] div_cnt;
    logic       pix_tick;
    assign pix_tick = (div_cnt == 3'd0);
    always_ff @(posedge clk) begin
        if (rst) div_cnt <= 0;
        else     div_cnt <= div_cnt + 3'd1;
    end

    // ---- FIFO storage ----
    logic [15:0] fifo [0:FIFO_DEPTH-1];
    logic [FIFO_AW-1:0] wr_ptr, rd_ptr;
    logic [FIFO_AW:0]   count; // 0..FIFO_DEPTH

    // ---- Outstanding-read tracking ----
    // Counts reads that have been accepted but not yet returned. Robust even
    // if the memory system allows more than one read in flight.
    logic [FIFO_AW:0] outstanding;

    // ---- Read pointer into the framebuffer ----
    logic [$clog2(FB_LEN)-1:0] fb_ptr;

    // ---- Memory bus driver ----
    // Only issue a read when buffered + outstanding pixels leave FIFO room.
    assign mem_we      = 1'b0;
    assign mem_wr_data = 16'h0;
    assign mem_addr    = FB_BASE[MEM_AW-1:0] + MEM_AW'(fb_ptr);
    assign mem_req     = enable && ((count + outstanding) < FIFO_DEPTH);

    // ---- Pop logic (consumer) ----
    wire pop = pix_tick && enable && (count != 0);

    // ---- Pixel output (registered so it is stable for sampling) ----
    logic [15:0] pix_data_r;
    logic        pix_valid_r;
    assign pix_data  = pix_data_r;
    assign pix_valid = pix_valid_r;

    // ---- FIFO + pointer + counter update ----
    always_ff @(posedge clk) begin
        if (rst || frame_start) begin
            wr_ptr <= 0;
            rd_ptr <= 0;
            count  <= 0;
            fb_ptr <= 0;
            pix_valid_r <= 0;
            outstanding <= 0;
        end else begin
            // Default: no pixel out this cycle.
            pix_valid_r <= 0;

            // Outstanding-read counter: +1 on accepted request, -1 on return.
            case ({(mem_req && mem_ready), mem_rd_valid})
                2'b10: outstanding <= outstanding + 1'b1;
                2'b01: outstanding <= outstanding - 1'b1;
                default: outstanding <= outstanding;
            endcase

            // FB read pointer advances on accepted read
            if (mem_req && mem_ready)
                fb_ptr <= (fb_ptr == FB_LEN-1) ? '0 : fb_ptr + 1'b1;

            // Push on rd_valid
            if (mem_rd_valid) begin
                fifo[wr_ptr] <= mem_rd_data;
                wr_ptr <= (wr_ptr == FIFO_AW'(FIFO_DEPTH-1)) ? '0 : wr_ptr + 1'b1;
            end

            // Pop on pixel tick: emit the popped pixel and advance read pointer.
            if (pop) begin
                pix_data_r  <= fifo[rd_ptr];
                pix_valid_r <= 1'b1;
                rd_ptr <= (rd_ptr == FIFO_AW'(FIFO_DEPTH-1)) ? '0 : rd_ptr + 1'b1;
            end

            // Count update: handles simultaneous push and pop
            case ({mem_rd_valid, pop})
                2'b10: count <= count + 1'b1;
                2'b01: count <= count - 1'b1;
                default: count <= count;  // 00: idle, 11: net change zero
            endcase
        end
    end

endmodule
