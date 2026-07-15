`default_nettype none

// Simplified SDR SDRAM behavioral model for simulation
// Models W9825G6KH: 4 banks, 8192 rows, 512 cols, 16-bit
// Only models basic command timing, not full protocol checking
// Storage: MEM_DEPTH halfwords (default 384 KB = 192 K x 16), addresses
// wrapped to ADDR_BITS. Every address used must be < MEM_DEPTH.

module sdram_model #(
    parameter ROW_BITS  = 13,
    parameter COL_BITS  = 9,
    parameter BANK_BITS = 2,
    parameter DATA_BITS = 16,
    parameter CAS_LATENCY = 2,
    parameter BURST_LEN = 8,
    // Backing-store depth in halfwords, independent of ADDR_BITS. ADDR_BITS
    // sets the address-wrap width (must cover the highest address used);
    // MEM_DEPTH sets how much storage is actually allocated. Every address
    // used must be < MEM_DEPTH. Default: 384 KB = 192 K halfwords.
    parameter MEM_DEPTH = 512 * 1024,
    parameter INIT_FILE = ""     // optional $readmemh preload of the storage
) (
    input  wire                   sdram_clk,
    input  wire                   sdram_cke,
    input  wire                   sdram_cs_n,
    input  wire                   sdram_ras_n,
    input  wire                   sdram_cas_n,
    input  wire                   sdram_we_n,
    input  wire  [BANK_BITS-1:0]  sdram_ba,
    input  wire  [ROW_BITS-1:0]   sdram_addr,
    input  wire                   sdram_dqm,
    output logic [DATA_BITS-1:0]  sdram_dq_out,
    output logic                  sdram_dq_oe,
    input  wire  [DATA_BITS-1:0]  sdram_dq_in
);

    // Commands
    localparam CMD_NOP       = 4'b0111;
    localparam CMD_ACTIVATE  = 4'b0011;
    localparam CMD_READ      = 4'b0101;
    localparam CMD_WRITE     = 4'b0100;
    localparam CMD_PRECHARGE = 4'b0010;
    localparam CMD_REFRESH   = 4'b0001;
    localparam CMD_MRS       = 4'b0000;

    // Storage size covers the full memory map incl. the 320x480 framebuffer at
    // 0x30000..0x55800. Addresses wrap to 19 bits (up to 0x7FFFF).
    localparam ADDR_BITS = 19;   // address-wrap width (covers up to 0x7FFFF)
    localparam BEAT_BITS = $clog2(BURST_LEN+1);

    // Active row per bank
    logic [ROW_BITS-1:0] active_row [0:3];
    logic [3:0] bank_active;

    // DQ output
    logic dq_oe;
    logic [DATA_BITS-1:0] dq_out;
    assign sdram_dq_out = dq_out;
    assign sdram_dq_oe = dq_oe;

    // Decode command
    wire [3:0] cmd = {sdram_cs_n, sdram_ras_n, sdram_cas_n, sdram_we_n};

    // Burst tracking: when a READ/WRITE command arrives the column auto-
    // increments for BURST_LEN beats.
    logic [BANK_BITS-1:0] burst_bank;
    logic [COL_BITS-1:0]  burst_col;
    logic [BEAT_BITS-1:0] rd_beat;   // read beats remaining to fetch
    logic [BEAT_BITS-1:0] wr_beat;   // write beats remaining to accept

    // Combinational dpram address (shared between read and write).
    // During a write burst we address by burst_col; otherwise by the incoming
    // command's column.
    logic [ADDR_BITS-1:0] dpram_addr;
    logic                 dpram_wr_en;
    logic [DATA_BITS-1:0] dpram_rd_data;
    logic [COL_BITS-1:0]  cur_col;
    logic [BANK_BITS-1:0] cur_bank;

    always_comb begin
        logic [BANK_BITS+ROW_BITS+COL_BITS-1:0] full_addr;
        // Beat 0 of a read/write uses the incoming command address; beats 1..N
        // use the auto-incrementing burst column.
        if (cmd == CMD_READ || cmd == CMD_WRITE) begin
            cur_bank = sdram_ba;
            cur_col  = sdram_addr[COL_BITS-1:0];
        end else begin
            cur_bank = burst_bank;
            cur_col  = burst_col;
        end
        full_addr   = {cur_bank, active_row[cur_bank], cur_col};
        dpram_addr  = full_addr[ADDR_BITS-1:0];
        dpram_wr_en = sdram_cke && bank_active[cur_bank] && !sdram_dqm
                   && ((cmd == CMD_WRITE) || (wr_beat != 0));
    end

    dpram #(
        .ADDR_WIDTH(ADDR_BITS),
        .DATA_WIDTH(DATA_BITS),
        .DEPTH(MEM_DEPTH),
        .INIT_FILE(INIT_FILE)
    ) mem_inst (
        .clk     (sdram_clk),
        .wr_en   (dpram_wr_en),
        .wr_addr (dpram_addr),
        .wr_data (sdram_dq_in),
        .rd_addr (dpram_addr),
        .rd_data (dpram_rd_data)
    );

    // Read pipeline. dpram registered read contributes 1 cycle of latency,
    // so the explicit pipeline carries the remaining CAS_LATENCY-1 stages.
    logic [DATA_BITS-1:0]   rd_pipe [0:CAS_LATENCY-2];
    logic [CAS_LATENCY-1:0] rd_valid_pipe;

    initial begin
        bank_active = 0;
        rd_valid_pipe = 0;
        dq_oe = 0;
        rd_beat = 0;
        wr_beat = 0;
    end

    always @(posedge sdram_clk) begin
        logic inject;  // a read address is presented this cycle
        dq_oe <= 0;
        inject = 1'b0;

        if (sdram_cke) begin
            case (cmd)
                CMD_ACTIVATE: begin
                    active_row[sdram_ba] <= sdram_addr;
                    bank_active[sdram_ba] <= 1;
                end

                CMD_READ: begin
                    if (bank_active[sdram_ba]) begin
                        // Beat 0 presented this cycle; set up beats 1..N-1.
                        inject     = 1'b1;
                        burst_bank <= sdram_ba;
                        burst_col  <= sdram_addr[COL_BITS-1:0] + 1'b1;
                        rd_beat    <= BEAT_BITS'(BURST_LEN - 1);
                    end
                end

                CMD_WRITE: begin
                    // Beat 0 written via dpram_wr_en this cycle; set up 1..N-1.
                    burst_bank <= sdram_ba;
                    burst_col  <= sdram_addr[COL_BITS-1:0] + 1'b1;
                    wr_beat    <= BEAT_BITS'(BURST_LEN - 1);
                end

                CMD_PRECHARGE: begin
                    if (sdram_addr[10])
                        bank_active <= 0;  // all banks
                    else
                        bank_active[sdram_ba] <= 0;
                end

                CMD_MRS: begin
                    // Just acknowledge, don't need to store
                end

                CMD_REFRESH: begin
                    // Nothing to model
                end

                default: ; // NOP
            endcase

            // Continue an in-progress read burst (beats 1..N-1).
            if (rd_beat != 0) begin
                inject    = 1'b1;
                burst_col <= burst_col + 1'b1;
                rd_beat   <= rd_beat - 1'b1;
            end

            // Continue an in-progress write burst (beats 1..N-1).
            if (wr_beat != 0) begin
                burst_col <= burst_col + 1'b1;
                wr_beat   <= wr_beat - 1'b1;
            end
        end

        // Read data pipeline (matches single-word timing, one beat per cycle).
        rd_valid_pipe <= {rd_valid_pipe[CAS_LATENCY-2:0], inject};
        rd_pipe[0] <= dpram_rd_data;
        for (int i = 1; i <= CAS_LATENCY-2; i++)
            rd_pipe[i] <= rd_pipe[i-1];

        if (rd_valid_pipe[CAS_LATENCY-1]) begin
            dq_oe <= 1;
            dq_out <= rd_pipe[CAS_LATENCY-2];
        end
    end

endmodule
