// Simplified SDR SDRAM behavioral model for simulation
// Models W9825G6KH: 4 banks, 8192 rows, 512 cols, 16-bit
// Only models basic command timing, not full protocol checking
// Storage: 256 KB (128 K x 16 bits), addresses wrapped to fit.

module sdram_model #(
    parameter ROW_BITS  = 13,
    parameter COL_BITS  = 9,
    parameter BANK_BITS = 2,
    parameter DATA_BITS = 16,
    parameter CAS_LATENCY = 2,
    parameter INIT_FILE = ""     // optional $readmemh preload of the storage
) (
    input  logic                  sdram_clk,
    input  logic                  sdram_cke,
    input  logic                  sdram_cs_n,
    input  logic                  sdram_ras_n,
    input  logic                  sdram_cas_n,
    input  logic                  sdram_we_n,
    input  logic [BANK_BITS-1:0]  sdram_ba,
    input  logic [ROW_BITS-1:0]   sdram_addr,
    input  logic                  sdram_dqm,
    output logic [DATA_BITS-1:0]  sdram_dq_out,
    output logic                  sdram_dq_oe,
    input  logic [DATA_BITS-1:0]  sdram_dq_in
);

    // Commands
    localparam CMD_NOP       = 4'b0111;
    localparam CMD_ACTIVATE  = 4'b0011;
    localparam CMD_READ      = 4'b0101;
    localparam CMD_WRITE     = 4'b0100;
    localparam CMD_PRECHARGE = 4'b0010;
    localparam CMD_REFRESH   = 4'b0001;
    localparam CMD_MRS       = 4'b0000;

    // Storage size: 256 KB = 128 K halfwords. Addresses wrap to 17 bits.
    localparam ADDR_BITS = 17;

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

    // Combinational dpram address (shared between read and write).
    // The write port is gated by dpram_wr_en, so giving the read port the same
    // address is harmless when only a write is happening.
    logic [ADDR_BITS-1:0] dpram_addr;
    logic                 dpram_wr_en;
    logic [DATA_BITS-1:0] dpram_rd_data;

    always_comb begin
        logic [BANK_BITS+ROW_BITS+COL_BITS-1:0] full_addr;
        full_addr   = {sdram_ba, active_row[sdram_ba], sdram_addr[COL_BITS-1:0]};
        dpram_addr  = full_addr[ADDR_BITS-1:0];
        dpram_wr_en = sdram_cke && bank_active[sdram_ba]
                   && cmd == CMD_WRITE && !sdram_dqm;
    end

    dpram #(
        .ADDR_WIDTH(ADDR_BITS),
        .DATA_WIDTH(DATA_BITS),
        .DEPTH(1 << ADDR_BITS),
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
    end

    always @(posedge sdram_clk) begin
        dq_oe <= 0;

        // Shift valid pipe (oldest at MSB)
        rd_valid_pipe <= {rd_valid_pipe[CAS_LATENCY-2:0], 1'b0};

        // Stage 0 captures dpram output one cycle after CMD_READ.
        rd_pipe[0] <= dpram_rd_data;
        for (int i = 1; i <= CAS_LATENCY-2; i++)
            rd_pipe[i] <= rd_pipe[i-1];

        // Output at end of pipeline
        if (rd_valid_pipe[CAS_LATENCY-1]) begin
            dq_oe <= 1;
            dq_out <= rd_pipe[CAS_LATENCY-2];
        end

        if (sdram_cke) begin
            case (cmd)
                CMD_ACTIVATE: begin
                    active_row[sdram_ba] <= sdram_addr;
                    bank_active[sdram_ba] <= 1;
                end

                CMD_READ: begin
                    if (bank_active[sdram_ba])
                        rd_valid_pipe[0] <= 1;
                end

                CMD_WRITE: begin
                    // Write is handled by dpram via combinational dpram_wr_en.
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
        end
    end

endmodule
