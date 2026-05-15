// Simplified SDR SDRAM behavioral model for simulation
// Models W9825G6KH: 4 banks, 8192 rows, 512 cols, 16-bit
// Only models basic command timing, not full protocol checking

module sdram_model #(
    parameter ROW_BITS  = 13,
    parameter COL_BITS  = 9,
    parameter BANK_BITS = 2,
    parameter DATA_BITS = 16,
    parameter CAS_LATENCY = 2
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

    // Storage: reduced for simulation (only need ~2MB of the 32MB address space)
    localparam SIM_DEPTH = 1 << 21; // 2M entries (covers FB_BASE + framebuffer)
    logic [DATA_BITS-1:0] mem [0:SIM_DEPTH-1] /* verilator public */;

    // Active row per bank
    logic [ROW_BITS-1:0] active_row [0:3];
    logic [3:0] bank_active;

    // Read pipeline
    logic [DATA_BITS-1:0] rd_pipe [0:CAS_LATENCY-1];
    logic [CAS_LATENCY-1:0] rd_valid_pipe;

    // DQ output
    logic dq_oe;
    logic [DATA_BITS-1:0] dq_out;
    assign sdram_dq_out = dq_out;
    assign sdram_dq_oe = dq_oe;

    // Decode command
    wire [3:0] cmd = {sdram_cs_n, sdram_ras_n, sdram_cas_n, sdram_we_n};

    // Address computation
    function automatic int flat_addr(input logic [BANK_BITS-1:0] bank,
                                     input logic [ROW_BITS-1:0] row,
                                     input logic [COL_BITS-1:0] col);
        return {bank, row, col};
    endfunction

    initial begin
        bank_active = 0;
        rd_valid_pipe = 0;
        dq_oe = 0;
    end

    always @(posedge sdram_clk) begin
        dq_oe <= 0;

        // Shift read pipeline
        rd_valid_pipe <= {rd_valid_pipe[CAS_LATENCY-2:0], 1'b0};
        for (int i = CAS_LATENCY-1; i > 0; i--)
            rd_pipe[i] <= rd_pipe[i-1];

        // Output read data at end of pipeline
        if (rd_valid_pipe[CAS_LATENCY-1]) begin
            dq_oe <= 1;
            dq_out <= rd_pipe[CAS_LATENCY-1];
        end

        if (sdram_cke) begin
            case (cmd)
                CMD_ACTIVATE: begin
                    active_row[sdram_ba] <= sdram_addr;
                    bank_active[sdram_ba] <= 1;
                end

                CMD_READ: begin
                    if (bank_active[sdram_ba]) begin
                        logic [COL_BITS-1:0] col;
                        col = sdram_addr[COL_BITS-1:0];
                        rd_pipe[0] <= mem[flat_addr(sdram_ba, active_row[sdram_ba], col) & (SIM_DEPTH-1)];
                        rd_valid_pipe[0] <= 1;
                    end
                end

                CMD_WRITE: begin
                    if (bank_active[sdram_ba]) begin
                        logic [COL_BITS-1:0] col;
                        col = sdram_addr[COL_BITS-1:0];
                        if (!sdram_dqm)
                            mem[flat_addr(sdram_ba, active_row[sdram_ba], col) & (SIM_DEPTH-1)] <= sdram_dq_in;
                    end
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
