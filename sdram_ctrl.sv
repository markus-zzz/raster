// SDR SDRAM Controller for W9825G6KH-6
// 16Mx16, 4 banks, 13-bit row, 9-bit column, CAS=2, 100 MHz
//
// Simple interface: address, read/write request, data in/out, ready
// Address mapping: {bank[1:0], row[12:0], col[8:0]} = 24 bits = 16M words

module sdram_ctrl #(
    parameter CLK_FREQ    = 100_000_000,
    parameter ROW_BITS    = 13,
    parameter COL_BITS    = 9,
    parameter BANK_BITS   = 2,
    parameter DATA_BITS   = 16,
    parameter CAS_LATENCY = 2,
    // Timing in clock cycles (for 100 MHz)
    parameter tRP         = 2,   // precharge to activate
    parameter tRCD        = 2,   // activate to read/write
    parameter tRC         = 7,   // activate to activate (same bank)
    parameter tMRD        = 2,   // mode register set
    parameter REFRESH_INTERVAL = 780  // 7.8us / 10ns
) (
    input  logic clk,
    input  logic rst,

    // User interface
    input  logic [ROW_BITS+COL_BITS+BANK_BITS-1:0] addr,  // {bank, row, col}
    input  logic                  req,       // request strobe
    input  logic                  we,        // 1=write, 0=read
    input  logic [DATA_BITS-1:0]  wr_data,
    output logic [DATA_BITS-1:0]  rd_data,
    output logic                  rd_valid,
    output logic                  ready,     // combinational: can accept new request

    // SDRAM pins
    output logic                  sdram_clk,
    output logic                  sdram_cke,
    output logic                  sdram_cs_n,
    output logic                  sdram_ras_n,
    output logic                  sdram_cas_n,
    output logic                  sdram_we_n,
    output logic [BANK_BITS-1:0]  sdram_ba,
    output logic [ROW_BITS-1:0]   sdram_addr,
    output logic                  sdram_dqm,
    output logic [DATA_BITS-1:0]  sdram_dq_out,
    output logic                  sdram_dq_oe,
    input  logic [DATA_BITS-1:0]  sdram_dq_in
);

    // SDRAM commands: {CS_N, RAS_N, CAS_N, WE_N}
    localparam CMD_NOP       = 4'b0111;
    localparam CMD_ACTIVATE  = 4'b0011;
    localparam CMD_READ      = 4'b0101;
    localparam CMD_WRITE     = 4'b0100;
    localparam CMD_PRECHARGE = 4'b0010;
    localparam CMD_REFRESH   = 4'b0001;
    localparam CMD_MRS       = 4'b0000;

    // State machine
    typedef enum logic [3:0] {
        S_INIT_WAIT,
        S_INIT_PRECHARGE,
        S_INIT_REFRESH1,
        S_INIT_REFRESH2,
        S_INIT_MRS,
        S_IDLE,
        S_ACTIVATE,
        S_READ,
        S_WRITE,
        S_PRECHARGE,
        S_REFRESH
    } state_t;

    state_t state;
    logic [15:0] init_counter;
    logic [3:0]  wait_counter;
    logic [9:0]  refresh_counter;

    // ready is combinational: high when controller can accept a new request
    assign ready = (state == S_IDLE)
                && (wait_counter == 0)
                && !(|rd_pipe)
                && (refresh_counter < REFRESH_INTERVAL);

    // Command output
    logic [3:0] cmd;
    assign {sdram_cs_n, sdram_ras_n, sdram_cas_n, sdram_we_n} = cmd;

    // Clock output (directly from clk)
    assign sdram_clk = clk;
    assign sdram_cke = 1'b1;
    assign sdram_dqm = 1'b0;  // no masking

    // DQ output
    logic dq_oe;
    logic [DATA_BITS-1:0] dq_out;
    assign sdram_dq_out = dq_out;
    assign sdram_dq_oe = dq_oe;

    // Address decomposition
    logic [BANK_BITS-1:0] req_bank;
    logic [ROW_BITS-1:0]  req_row;
    logic [COL_BITS-1:0]  req_col;
    assign req_bank = addr[ROW_BITS+COL_BITS+BANK_BITS-1 : ROW_BITS+COL_BITS];
    assign req_row  = addr[ROW_BITS+COL_BITS-1 : COL_BITS];
    assign req_col  = addr[COL_BITS-1 : 0];

    // Track open row per bank
    logic [ROW_BITS-1:0] open_row [4];
    logic [3:0]          row_open;  // one bit per bank

    // Latched request
    logic [BANK_BITS-1:0] lat_bank;
    logic [ROW_BITS-1:0]  lat_row;
    logic [COL_BITS-1:0]  lat_col;
    logic                  lat_we;
    logic [DATA_BITS-1:0]  lat_wr_data;

    // Read pipeline: CAS_LATENCY + 2 cycles total (1 for our READ→model, 1 for model→our DQ_in)
    logic [CAS_LATENCY+1:0] rd_pipe;

    always_ff @(posedge clk) begin
        if (rst) begin
            state <= S_INIT_WAIT;
            init_counter <= 16'd10000; // 100us at 100MHz
            wait_counter <= 0;
            refresh_counter <= 0;
            cmd <= CMD_NOP;
            rd_valid <= 0;
            dq_oe <= 0;
            row_open <= 0;
            rd_pipe <= 0;
        end else begin
            cmd <= CMD_NOP;
            rd_valid <= 0;
            dq_oe <= 0;

            // Read data pipeline
            rd_pipe <= {rd_pipe[CAS_LATENCY:0], 1'b0};
            if (rd_pipe[CAS_LATENCY+1]) begin
                rd_data <= sdram_dq_in;
                rd_valid <= 1;
            end

            // Refresh counter
            if (state == S_IDLE || state == S_REFRESH)
                refresh_counter <= (state == S_REFRESH) ? 0 : refresh_counter + 1;

            case (state)
                S_INIT_WAIT: begin
                    if (init_counter == 0)
                        state <= S_INIT_PRECHARGE;
                    else
                        init_counter <= init_counter - 1;
                end

                S_INIT_PRECHARGE: begin
                    cmd <= CMD_PRECHARGE;
                    sdram_addr[10] <= 1'b1; // all banks
                    wait_counter <= tRP;
                    state <= S_INIT_REFRESH1;
                end

                S_INIT_REFRESH1: begin
                    if (wait_counter > 0) begin
                        wait_counter <= wait_counter - 1;
                    end else begin
                        cmd <= CMD_REFRESH;
                        wait_counter <= tRC;
                        state <= S_INIT_REFRESH2;
                    end
                end

                S_INIT_REFRESH2: begin
                    if (wait_counter > 0) begin
                        wait_counter <= wait_counter - 1;
                    end else begin
                        cmd <= CMD_REFRESH;
                        wait_counter <= tRC;
                        state <= S_INIT_MRS;
                    end
                end

                S_INIT_MRS: begin
                    if (wait_counter > 0) begin
                        wait_counter <= wait_counter - 1;
                    end else begin
                        cmd <= CMD_MRS;
                        sdram_ba <= 0;
                        // Mode register: burst=1, sequential, CAS=2, standard
                        sdram_addr <= 13'b000_0_10_0_000_0_000;
                        wait_counter <= tMRD;
                        state <= S_IDLE;
                    end
                end

                S_IDLE: begin
                    if (wait_counter > 0) begin
                        wait_counter <= wait_counter - 1;
                    end else if (refresh_counter >= REFRESH_INTERVAL && !(|rd_pipe)) begin
                        // Refresh: only when no outstanding read in flight
                        cmd <= CMD_PRECHARGE;
                        sdram_addr[10] <= 1'b1;
                        row_open <= 0;
                        wait_counter <= tRP;
                        state <= S_REFRESH;
                    end else if (req && ready) begin
                        // ready is combinational; safe to use here
                        lat_bank <= req_bank;
                        lat_row <= req_row;
                        lat_col <= req_col;
                        lat_we <= we;
                        lat_wr_data <= wr_data;

                        if (row_open[req_bank] && open_row[req_bank] == req_row) begin
                            // Row hit: go directly to read/write
                            state <= we ? S_WRITE : S_READ;
                        end else if (row_open[req_bank]) begin
                            // Row miss: precharge then activate
                            cmd <= CMD_PRECHARGE;
                            sdram_ba <= req_bank;
                            sdram_addr[10] <= 1'b0;
                            row_open[req_bank] <= 0;
                            wait_counter <= tRP;
                            state <= S_ACTIVATE;
                        end else begin
                            // Bank idle: activate directly
                            state <= S_ACTIVATE;
                        end
                    end
                end

                S_ACTIVATE: begin
                    if (wait_counter > 0) begin
                        wait_counter <= wait_counter - 1;
                    end else begin
                        cmd <= CMD_ACTIVATE;
                        sdram_ba <= lat_bank;
                        sdram_addr <= lat_row;
                        open_row[lat_bank] <= lat_row;
                        row_open[lat_bank] <= 1;
                        wait_counter <= tRCD;
                        state <= lat_we ? S_WRITE : S_READ;
                    end
                end

                S_READ: begin
                    if (wait_counter > 0) begin
                        wait_counter <= wait_counter - 1;
                    end else begin
                        cmd <= CMD_READ;
                        sdram_ba <= lat_bank;
                        sdram_addr <= {{(ROW_BITS-COL_BITS-1){1'b0}}, 1'b0, lat_col}; // A10=0 (no auto-precharge)
                        rd_pipe[0] <= 1;
                        state <= S_IDLE;
                    end
                end

                S_WRITE: begin
                    if (wait_counter > 0) begin
                        wait_counter <= wait_counter - 1;
                    end else begin
                        cmd <= CMD_WRITE;
                        sdram_ba <= lat_bank;
                        sdram_addr <= {{(ROW_BITS-COL_BITS-1){1'b0}}, 1'b0, lat_col}; // A10=0
                        dq_oe <= 1;
                        dq_out <= lat_wr_data;
                        state <= S_IDLE;
                    end
                end

                S_PRECHARGE: begin
                    if (wait_counter > 0) begin
                        wait_counter <= wait_counter - 1;
                    end else begin
                        state <= S_IDLE;
                    end
                end

                S_REFRESH: begin
                    if (wait_counter > 0) begin
                        wait_counter <= wait_counter - 1;
                    end else begin
                        cmd <= CMD_REFRESH;
                        wait_counter <= tRC;
                        refresh_counter <= 0;
                        state <= S_IDLE;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
