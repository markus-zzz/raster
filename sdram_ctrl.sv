`default_nettype none

// SDR SDRAM Controller for W9825G6KH-6
// 16Mx16, 4 banks, 13-bit row, 9-bit column, CAS=2, 100 MHz
//
// Burst interface: every access transfers BURST_LEN (=8) words.
//   Read : one `req` (we=0) -> BURST_LEN `rd_valid`+`rd_data` beats.
//   Write: one `req` (we=1) -> BURST_LEN `wr_data_req` pulses; the master
//          presents the corresponding word on `wr_data` each pulse.
// `addr` is the burst base address (must be BURST_LEN-aligned).
// Address mapping: {bank[1:0], row[12:0], col[8:0]} = 24 bits = 16M words

module sdram_ctrl #(
    parameter CLK_FREQ    = 100_000_000,
    parameter ROW_BITS    = 13,
    parameter COL_BITS    = 9,
    parameter BANK_BITS   = 2,
    parameter DATA_BITS   = 16,
    parameter CAS_LATENCY = 2,
    parameter BURST_LEN   = 8,   // fixed burst length (words per access)
    // Timing in clock cycles (for 100 MHz)
    parameter tRP         = 2,   // precharge to activate
    parameter tRCD        = 2,   // activate to read/write
    parameter tRC         = 7,   // activate to activate (same bank)
    parameter tMRD        = 2,   // mode register set
    parameter REFRESH_INTERVAL = 760, // 7.8us / 10ns
    // Read-capture latency: total clocks from a READ command to rd_data valid.
    // Default CAS_LATENCY+3 accounts for command launch, the chip's CAS access
    // and the registered DQ input capture. On real hardware the round-trip
    // (clock-to-out + board delay) may shift this by a whole cycle; sweep this
    // together with the SDRAM clock phase (clkgen SDRAM_PHASE) to find the
    // working point. In simulation (zero-delay model) the default is correct.
    parameter READ_LAT_ADJ = 3
) (
    input  wire  clk,
    input  wire  rst,

    // User interface. Every access transfers BURST_LEN words. `addr` is the
    // burst base address (must be BURST_LEN-aligned).
    input  wire  [ROW_BITS+COL_BITS+BANK_BITS-1:0] addr,  // {bank, row, col}
    input  wire                   req,       // request strobe
    input  wire                   we,        // 1=write, 0=read
    input  wire  [DATA_BITS-1:0]  wr_data,   // streamed write data (see wr_data_req)
    output logic                  wr_data_req, // pulses each beat the controller consumes wr_data
    output logic [DATA_BITS-1:0]  rd_data,
    output logic                  rd_valid,  // pulses each returned read beat
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
    input  wire  [DATA_BITS-1:0]  sdram_dq_in
);

    localparam BEAT_BITS = $clog2(BURST_LEN+1);

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
        S_READ_BURST,
        S_WRITE,
        S_WRITE_CMD,
        S_WRITE_BURST,
        S_PRECHARGE,
        S_REFRESH
    } state_t;

    state_t state;
    logic [15:0] init_counter;
    logic [3:0]  wait_counter;
    logic [9:0]  refresh_counter;

    // Read pipeline: a marker is injected at each READ beat and shifted for
    // RD_LAT cycles; when it reaches the end the (IOB-registered) DQ input is
    // captured. RD_LAT is tunable for real-hardware round-trip latency.
    localparam int RD_LAT = CAS_LATENCY + READ_LAT_ADJ;
    logic [RD_LAT-1:0]      rd_pipe;
    logic [DATA_BITS-1:0]   dq_in_r;     // registered DQ input (packs into IOB)
    logic [BEAT_BITS-1:0]   rd_inject;  // read beats still to inject
    logic [BEAT_BITS-1:0]   wr_beat;    // write beats still to drive

    // Burst busy: high from accept until the access fully completes. Blocks new
    // requests so each burst owns the bus end to end.
    logic busy;

    // ready is combinational: high when controller can accept a new request
    assign ready = (state == S_IDLE)
                && !busy
                && (wait_counter == 0)
                && !(|rd_pipe)
                && (rd_inject == 0)
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

    // wr_data_req leads the dq_out latch by one cycle so a master whose data
    // source has 1-cycle (registered) latency can stream directly without a
    // prefetch buffer. The master advances its read address on each
    // wr_data_req; the controller latches the resulting data the next cycle.
    assign wr_data_req = (state == S_WRITE && wait_counter == 0)
                      || (state == S_WRITE_CMD)
                      || (state == S_WRITE_BURST && wr_beat > 1);

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
            rd_inject <= 0;
            wr_beat <= 0;
            busy <= 0;
        end else begin
            cmd <= CMD_NOP;
            rd_valid <= 0;
            dq_oe <= 0;

            // Read data pipeline: a marker reaching the end produces a rd_valid.
            // DQ is captured through a registered input (dq_in_r) for a clean,
            // IOB-packed sample point.
            dq_in_r <= sdram_dq_in;
            rd_pipe <= {rd_pipe[RD_LAT-2:0], (rd_inject != 0) ? 1'b1 : 1'b0};
            if (rd_inject != 0)
                rd_inject <= rd_inject - 1'b1;
            if (rd_pipe[RD_LAT-1]) begin
                rd_data <= dq_in_r;
                rd_valid <= 1;
            end

            // Free-running wall-clock refresh timer: count every cycle (not
            // only in S_IDLE) so refresh isn't starved during sustained bus
            // activity. Hold at the interval once due; reset when serviced.
            if (state == S_REFRESH)
                refresh_counter <= 0;
            else if (refresh_counter < REFRESH_INTERVAL)
                refresh_counter <= refresh_counter + 1;

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
                        // Mode register: burst length 8, sequential, CAS=2.
                        // [2:0]=011 (BL=8), [3]=0 seq, [6:4]=CAS=010
                        sdram_addr <= 13'b000_0_010_0_011;
                        wait_counter <= tMRD;
                        state <= S_IDLE;
                    end
                end

                S_IDLE: begin
                    if (wait_counter > 0) begin
                        wait_counter <= wait_counter - 1;
                    end else if (refresh_counter >= REFRESH_INTERVAL && !(|rd_pipe) && rd_inject == 0) begin
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
                        busy <= 1'b1;

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
                        // One CMD_READ; the SDRAM bursts BURST_LEN words.
                        cmd <= CMD_READ;
                        sdram_ba <= lat_bank;
                        sdram_addr <= {{(ROW_BITS-COL_BITS-1){1'b0}}, 1'b0, lat_col};
                        // Inject marker 0 on the CMD_READ cycle (matches the
                        // original single-word timing); markers 1..N-1 follow on
                        // the next cycles via rd_inject.
                        rd_inject <= BEAT_BITS'(BURST_LEN - 1);
                        rd_pipe[0] <= 1'b1;
                        state <= S_READ_BURST;
                    end
                end

                S_READ_BURST: begin
                    // Wait until all injected markers have drained out as rd_valid.
                    if (rd_inject == 0 && !(|rd_pipe)) begin
                        busy <= 1'b0;
                        state <= S_IDLE;
                    end
                end

                S_WRITE: begin
                    if (wait_counter > 0) begin
                        wait_counter <= wait_counter - 1;
                    end else begin
                        // Prime cycle: wr_data_req is asserted (combinationally)
                        // so the master presents beat-0's address now; its data
                        // arrives next cycle when we issue CMD_WRITE.
                        state <= S_WRITE_CMD;
                    end
                end

                S_WRITE_CMD: begin
                    // Issue CMD_WRITE and latch beat 0 (valid now). Request
                    // beat 1's address (wr_data_req high this cycle).
                    cmd <= CMD_WRITE;
                    sdram_ba <= lat_bank;
                    sdram_addr <= {{(ROW_BITS-COL_BITS-1){1'b0}}, 1'b0, lat_col};
                    dq_oe <= 1;
                    dq_out <= wr_data;                      // beat 0
                    wr_beat <= BEAT_BITS'(BURST_LEN - 1);   // beats 1..7 remain
                    state <= S_WRITE_BURST;
                end

                S_WRITE_BURST: begin
                    dq_oe <= 1;
                    dq_out <= wr_data;                      // beats 1..7
                    if (wr_beat == 1) begin
                        // Last beat latched; burst done.
                        busy <= 1'b0;
                        state <= S_IDLE;
                    end
                    wr_beat <= wr_beat - 1'b1;
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
