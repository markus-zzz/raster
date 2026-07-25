`default_nettype none

// Display controller with pixel FIFO between SDRAM reads (producer) and
// pixel-clock consumption (consumer).
//
//   Producer (reads from SDRAM):
//     Issues a read whenever the FIFO is non-full and has room for the reads
//     already in flight. Pushes returned data into the FIFO.
//
//   Consumer (display output):
//     Pops one pixel each cycle that pix_ce is asserted. The caller drives
//     pix_ce at the pixel rate (e.g. the LCD's 12.5 MHz clock enable) and only
//     during active display pixels.
//
// FIFO_DEPTH should be large enough to absorb SDRAM access latency and any
// arbitration jitter. With CAS_LATENCY=2 + small overhead, ~16 entries is
// plenty for filling-the-pipeline latency, though it won't prevent underrun
// during long GPU bursts (the display would just stall, equivalent to the
// underrun visual on real hardware).

module display_ctrl #(
    parameter MEM_AW     = 24,
    parameter FB_LEN     = 64000,   // 320 * 200
    parameter BURST      = 8,
    parameter FIFO_DEPTH = 32
) (
    input  wire  clk,
    input  wire  rst,
    input  wire  enable,
    // Pixel-rate clock enable. Each asserted cycle pops one pixel (if the FIFO
    // is non-empty) and pulses pix_valid the following cycle.
    input  wire  pix_ce,
    // Pulse high to restart fetching at FB_BASE+0. Caller must hold enable=0
    // long enough beforehand that no read is in flight (pending == 0).
    input  wire  frame_start,

    // Memory bus master interface
    output logic [MEM_AW-1:0] mem_addr,
    output logic              mem_req,
    output logic              mem_we,
    output logic [15:0]       mem_wr_data,
    input  wire  [15:0]       mem_rd_data,
    input  wire               mem_rd_valid,
    input  wire               mem_ready,

    // Pixel output (one valid pulse per popped pixel)
    output logic [15:0]       pix_data,
    output logic              pix_valid,

    input wire [MEM_AW-1:0]   fb_base
);

    localparam FIFO_AW = $clog2(FIFO_DEPTH);

    // ---- Pixel-rate tick comes from the caller ----
    wire pix_tick = pix_ce;

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
    // Issue burst reads of BURST words. Request when there is room for a full
    // burst on top of what is buffered + already in flight, and no request is
    // currently being accepted. mem_req is held until accepted (mem_ready).
    logic req_pending;  // we want to start a burst
    assign mem_we      = 1'b0;
    assign mem_wr_data = 16'h0;
    assign mem_addr    = fb_base + MEM_AW'(fb_ptr);
    assign mem_req     = req_pending;

    wire have_room = (count + outstanding + (FIFO_AW+1)'(BURST)) <= FIFO_DEPTH;

    // ---- Pop logic (consumer) ----
    wire pop = pix_tick && enable && (count != 0);

    // ---- Pixel output (registered so it is stable for sampling) ----
    logic [15:0] pix_data_r;
    logic        pix_valid_r;
    assign pix_data  = pix_data_r;
    assign pix_valid = pix_valid_r;

    // ---- FIFO + pointer + counter update ----
    always_ff @(posedge clk) begin
        if (rst | frame_start) begin
            wr_ptr <= 0;
            rd_ptr <= 0;
            count  <= 0;
            fb_ptr <= 0;
            pix_valid_r <= 0;
            outstanding <= 0;
            req_pending <= 0;
        end else begin
            // Default: no pixel out this cycle.
            pix_valid_r <= 0;

            // Request control: raise when there is room for a full burst and no
            // request is already waiting; drop once accepted. Outstanding-read
            // accounting: +BURST when a burst is accepted, -1 per returned beat.
            // (Accept and a returned beat never coincide: the controller is busy
            // for the whole burst and only becomes ready again after the last
            // beat has drained.)
            if (req_pending && mem_ready) begin
                outstanding <= outstanding + (FIFO_AW+1)'(BURST);
                fb_ptr <= (fb_ptr >= FB_LEN - BURST) ? '0 : fb_ptr + MEM_AW'(BURST);
                req_pending <= 0;
            end else begin
                if (!req_pending && enable && have_room)
                    req_pending <= 1'b1;
                if (mem_rd_valid)
                    outstanding <= outstanding - 1'b1;
            end

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
