`default_nettype none

module dpram #(
    parameter ADDR_WIDTH = 17,
    parameter DATA_WIDTH = 16,
    parameter DEPTH = 76800,
    parameter INIT_FILE = ""     // optional $readmemh preload
) (
    input  wire  clk,
    // Write port
    input  wire                     wr_en,
    input  wire  [ADDR_WIDTH-1:0]   wr_addr,
    input  wire  [DATA_WIDTH-1:0]   wr_data,
    // Read port
    input  wire  [ADDR_WIDTH-1:0]   rd_addr,
    output logic [DATA_WIDTH-1:0]   rd_data
);
    
    (* ram_style = "block" *) logic [DATA_WIDTH-1:0] mem [0:DEPTH-1] /* verilator public */;

    initial begin
        if (INIT_FILE != "")
            $readmemh(INIT_FILE, mem);
    end

    always_ff @(posedge clk) begin
        if (wr_en)
            mem[wr_addr] <= wr_data;
        rd_data <= mem[rd_addr];
    end

endmodule
