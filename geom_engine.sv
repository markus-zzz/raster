`default_nettype none

//===========================================================================
// geom_engine: hardware port of the C++ process_face() geometry stage,
// Q12.20 fixed point. Produces the 11-word triangle record consumed by
// gpu_top (identical layout to write_tri_record):
//   rec[0..5] : v0x,v0y,v1x,v1y,v2x,v2y (sub-pixel, after 1<->2 winding swap)
//   rec[6..8] : iz_at_00, diz_dx, diz_dy   (Q11.5)
//   rec[9]    : colour low16 ((g<<8)|b)
//   rec[10]   : colour high8 (r)
//
// Datapath: the multiplies are time-multiplexed onto TWO pipelined resources
// so the design uses few DSPs and meets timing:
//   * dot4  - a 4-lane dot-product unit (4 parallel multipliers -> registered
//             products -> registered adder tree). Used for matrix-row .
//             [x,y,z,w] (transform + normal) and the lighting dot.
//   * smul  - one pipelined 32x32->64 multiplier for the remaining scalar
//             products (lighting scale, projection, plane setup, 1/z origin).
// A sequential FSM issues one op at a time and waits for its result, so the
// arithmetic stays bit-identical to the reference model.
//===========================================================================
module geom_engine #(
    parameter int W = 320,
    parameter int H = 200
) (
    input  wire                     clk,
    input  wire                     rst,
    input  wire                     start,
    input  wire signed [31:0]       mat   [0:11],
    input  wire signed [31:0]       vtx   [0:8],
    input  wire signed [31:0]       nrm   [0:2],
    input  wire signed [31:0]       col   [0:2],
    input  wire signed [31:0]       light [0:2],
    output logic                    done,
    output logic                    valid,
    output logic signed [31:0]      bbminx, bbminy, bbmaxx, bbmaxy,
    output logic [15:0]             rec   [0:10]
);
    localparam int SH = 12;
    localparam signed [31:0] FX_ONE = 32'sd1 <<< SH;
    localparam signed [31:0] C_020  = 32'sd819;      // fxf(0.2) in Q20.12
    localparam signed [31:0] C_080  = 32'sd3277;     // fxf(0.8) in Q20.12
    localparam signed [31:0] OFF_X  = (W/2) <<< SH;  // fxi(W/2)
    localparam signed [31:0] OFF_Y  = (H/2) <<< SH;  // fxi(H/2)
    localparam signed [31:0] FXF275 = 32'sd275 <<< SH;

    // Q20.12 -> Q11.5 (real*32), round half away from zero.
    function automatic signed [15:0] to_q11_5(input signed [31:0] a);
        logic signed [63:0] v;
        v = $signed({{32{a[31]}}, a}) * 64'sd32;
        v = v + (a >= 0 ? 64'sd2048 : -64'sd2048);
        return v[SH+15:SH];
    endfunction

    //--------------------------------------------------------------------
    // Shared pipelined resources
    //--------------------------------------------------------------------
    // 4-lane dot product (transform / lighting)
    logic               dot_go, dot_done;
    logic signed [31:0] da0,da1,da2,da3, db0,db1,db2,db3;
    logic signed [31:0] dot_res;
    dot4 #(.SH(SH)) u_dot (.clk(clk), .start(dot_go),
                .a0(da0),.a1(da1),.a2(da2),.a3(da3),
                .b0(db0),.b1(db1),.b2(db2),.b3(db3),
                .result(dot_res), .done(dot_done));

    // scalar 32x32 -> 64 multiply
    logic               smul_go, smul_done;
    logic signed [31:0] smul_a, smul_b;
    logic signed [63:0] smul_p;
    smul u_smul (.clk(clk), .start(smul_go), .a(smul_a), .b(smul_b),
                 .p(smul_p), .done(smul_done));

    // sequential signed divider (64/64 -> low 32, trunc toward zero)
    logic               div_go, div_done;
    logic signed [63:0] div_num, div_den;
    logic signed [31:0] div_quo;
    divs u_div (.clk(clk), .rst(rst), .start(div_go),
                .num(div_num), .den(div_den), .done(div_done), .quo(div_quo));

    //--------------------------------------------------------------------
    // State
    //--------------------------------------------------------------------
    typedef enum logic [5:0] {
        S_IDLE, S_SMUL, S_DOT,
        S_XF_ISS, S_CULL,
        S_LDOT_STO, S_INT_STO,
        S_COL_A, S_COL_B, S_COL_C,
        S_PROJ_X, S_PROJ_XS, S_PROJ_YS, S_PROJ_ZS,
        S_SWAP, S_AREA_A, S_AREA_B, S_AREA_C,
        S_NX_A, S_NX_B, S_NX_C, S_NY_A, S_NY_B, S_NY_C,
        S_DIVX, S_DIVX_W, S_DIVY, S_DIVY_W, S_IZ00_A, S_IZ00_B, S_IZ00_C, S_FIN
    } st_t;
    st_t st, ret_st;

    // latched inputs
    logic signed [31:0] m [0:11], v [0:8], nn [0:2], cc [0:2], L [0:2];
    // results
    logic signed [31:0] vv [0:8], nvec [0:2];
    logic signed [31:0] diffuse, intensity;
    logic [7:0]         rr, gg, bb;
    logic signed [31:0] s0x,s0y,s1x,s1y,s2x,s2y, iz0,iz1,iz2;
    logic signed [31:0] p0x,p0y,p1x,p1y,p2x,p2y, izp0,izp1,izp2;
    logic signed [31:0] e1x,e1y,e2x,e2y, diz1,diz2, diz_dx,diz_dy, iz00;
    logic signed [63:0] area, nxg, nyg, pa, mres;
    logic signed [31:0] dres;
    logic               cull;
    logic [3:0]         xfi;      // transform issue index 0..12
    logic [3:0]         xf_col;   // transform collect index 0..12
    logic [1:0]         ci, vi;   // colour channel / vertex index

    function automatic signed [31:0] clamp01(input signed [31:0] a);
        if (a < 0)           return 0;
        else if (a > FX_ONE) return FX_ONE;
        else                 return a;
    endfunction

    // fmul result of the scalar multiply (Q12.20)
    wire signed [31:0] smul_fm = mres[SH+31:SH];   // (a*b) >>> SH

    integer k;
    always_ff @(posedge clk) begin
        if (rst) begin
            st <= S_IDLE; done <= 0; valid <= 0;
            dot_go <= 0; smul_go <= 0; div_go <= 0;
        end else begin
            done <= 0; dot_go <= 0; smul_go <= 0; div_go <= 0;
            case (st)
                S_IDLE: if (start) begin
                    for (k=0;k<12;k++) m[k] <= mat[k];
                    for (k=0;k<9;k++)  v[k] <= vtx[k];
                    for (k=0;k<3;k++) begin nn[k]<=nrm[k]; cc[k]<=col[k]; L[k]<=light[k]; end
                    xfi <= 0; xf_col <= 0; st <= S_XF_ISS;
                end

                // generic waits -------------------------------------------
                S_SMUL: if (smul_done) begin mres <= smul_p; st <= ret_st; end
                S_DOT:  if (dot_done)  begin dres <= dot_res; st <= ret_st; end

                // transform: 12 pipelined dot4 ops -> vv[0..8], nvec[0..2].
                // dot4 accepts a new op every cycle and returns results in
                // order 3 cycles later, so we issue all 12 back-to-back and
                // collect them as they stream out (~15 cycles vs ~60 serial).
                S_XF_ISS: begin
                    logic [1:0] vecsel; logic [1:0] row;
                    // ---- issue one op per cycle ----
                    if (xfi < 4'd12) begin
                        vecsel = xfi / 3;        // 0,1,2 verts ; 3 normal
                        row    = xfi % 3;
                        da0 <= m[row*4+0]; da1 <= m[row*4+1];
                        da2 <= m[row*4+2]; da3 <= m[row*4+3];
                        if (vecsel == 3) begin       // normal (w=0)
                            db0 <= nn[0]; db1 <= nn[1]; db2 <= nn[2]; db3 <= 0;
                        end else begin               // vertex (w=1)
                            db0 <= v[vecsel*3+0]; db1 <= v[vecsel*3+1];
                            db2 <= v[vecsel*3+2]; db3 <= FX_ONE;
                        end
                        dot_go <= 1;
                        xfi <= xfi + 1'b1;
                    end
                    // ---- collect results in issue order ----
                    if (dot_done) begin
                        if (xf_col < 4'd9) vv[xf_col]        <= dot_res; // vv[0..8]
                        else               nvec[xf_col - 4'd9] <= dot_res; // nvec[0..2]
                        if (xf_col == 4'd11) st <= S_CULL;
                        else xf_col <= xf_col + 1'b1;
                    end
                end

                // back-face cull -----------------------------------------
                S_CULL: begin
                    if (nvec[2] >= 0) begin valid <= 0; done <= 1; st <= S_IDLE; end
                    else begin
                        // lighting dot: [nx,ny,nz,0] . [L0,L1,L2,0]
                        da0<=nvec[0]; da1<=nvec[1]; da2<=nvec[2]; da3<=0;
                        db0<=L[0]; db1<=L[1]; db2<=L[2]; db3<=0;
                        dot_go<=1; ret_st<=S_LDOT_STO; st<=S_DOT;
                    end
                end
                S_LDOT_STO: begin
                    diffuse <= (dres < 0) ? 0 : dres;
                    // intensity = C020 + fmul(C080, diffuse)
                    smul_a <= C_080; smul_b <= (dres < 0) ? 0 : dres;
                    smul_go <= 1; ret_st <= S_INT_STO; st <= S_SMUL;
                end
                S_INT_STO: begin
                    intensity <= C_020 + smul_fm;
                    ci <= 0;
                    // first colour channel: intensity * col[0]
                    smul_a <= C_020 + smul_fm; smul_b <= cc[0];
                    smul_go <= 1; ret_st <= S_COL_B; st <= S_SMUL;
                end
                // colour channel ci: cr = clamp01(intensity*col); rgb=(cr*255)>>20
                S_COL_A: begin
                    smul_a <= intensity; smul_b <= cc[ci];
                    smul_go <= 1; ret_st <= S_COL_B; st <= S_SMUL;
                end
                S_COL_B: begin
                    logic signed [31:0] cr;
                    cr = clamp01(smul_fm);
                    smul_a <= cr; smul_b <= 32'sd255;
                    smul_go <= 1; ret_st <= S_COL_C; st <= S_SMUL;
                end
                S_COL_C: begin
                    logic [7:0] chan;
                    chan = mres[SH+7:SH];        // (cr*255) >>> SH
                    if (ci == 0) rr <= chan;
                    else if (ci == 1) gg <= chan;
                    else bb <= chan;
                    if (ci == 2) begin vi <= 0; st <= S_PROJ_X; end
                    else begin ci <= ci + 1'b1; st <= S_COL_A; end
                end

                // projection ---------------------------------------------
                S_PROJ_X: begin
                    smul_a <= vv[vi*3+0]; smul_b <= 32'sd80;
                    smul_go <= 1; ret_st <= S_PROJ_XS; st <= S_SMUL;
                end
                S_PROJ_XS: begin
                    logic signed [31:0] sx;
                    sx = ($signed(mres[31:0]) + OFF_X) >>> SH;
                    case (vi) 0: s0x<=sx; 1: s1x<=sx; default: s2x<=sx; endcase
                    smul_a <= vv[vi*3+1]; smul_b <= 32'sd80;
                    smul_go <= 1; ret_st <= S_PROJ_YS; st <= S_SMUL;
                end
                S_PROJ_YS: begin
                    logic signed [31:0] sy;
                    sy = ($signed(mres[31:0]) + OFF_Y) >>> SH;
                    case (vi) 0: s0y<=sy; 1: s1y<=sy; default: s2y<=sy; endcase
                    smul_a <= vv[vi*3+2]; smul_b <= 32'sd225;
                    smul_go <= 1; ret_st <= S_PROJ_ZS; st <= S_SMUL;
                end
                S_PROJ_ZS: begin
                    logic signed [31:0] z;
                    z = FXF275 + $signed(mres[31:0]);
                    case (vi)
                        0: iz0 <= (z > FX_ONE) ? z : FX_ONE;
                        1: iz1 <= (z > FX_ONE) ? z : FX_ONE;
                        default: iz2 <= (z > FX_ONE) ? z : FX_ONE;
                    endcase
                    if (vi == 2) st <= S_SWAP;
                    else begin vi <= vi + 1'b1; st <= S_PROJ_X; end
                end

                // winding swap (1<->2), bbox, edges ----------------------
                S_SWAP: begin
                    p0x<=s0x; p0y<=s0y; p1x<=s2x; p1y<=s2y; p2x<=s1x; p2y<=s1y;
                    izp0<=iz0; izp1<=iz2; izp2<=iz1;
                    // edges use post-swap coords
                    e1x <= s2x - s0x; e1y <= s2y - s0y;
                    e2x <= s1x - s0x; e2y <= s1y - s0y;
                    // bbox over the three screen verts
                    bbminx <= (s0x<s1x?(s0x<s2x?s0x:s2x):(s1x<s2x?s1x:s2x));
                    bbmaxx <= (s0x>s1x?(s0x>s2x?s0x:s2x):(s1x>s2x?s1x:s2x));
                    bbminy <= (s0y<s1y?(s0y<s2y?s0y:s2y):(s1y<s2y?s1y:s2y));
                    bbmaxy <= (s0y>s1y?(s0y>s2y?s0y:s2y):(s1y>s2y?s1y:s2y));
                    st <= S_AREA_A;
                end
                // area = e1x*e2y - e1y*e2x
                S_AREA_A: begin
                    smul_a <= e1x; smul_b <= e2y;
                    smul_go <= 1; ret_st <= S_AREA_B; st <= S_SMUL;
                end
                S_AREA_B: begin
                    pa <= mres;
                    smul_a <= e1y; smul_b <= e2x;
                    smul_go <= 1; ret_st <= S_AREA_C; st <= S_SMUL;
                end
                S_AREA_C: begin
                    area <= pa - mres;
                    diz1 <= izp1 - izp0; diz2 <= izp2 - izp0;
                    if ((pa - mres) == 0) begin
                        diz_dx <= 0; diz_dy <= 0; st <= S_IZ00_A;
                    end else st <= S_NX_A;
                end
                // nxg = diz1*e2y - diz2*e1y
                S_NX_A: begin
                    smul_a <= diz1; smul_b <= e2y;
                    smul_go <= 1; ret_st <= S_NX_B; st <= S_SMUL;
                end
                S_NX_B: begin
                    pa <= mres;
                    smul_a <= diz2; smul_b <= e1y;
                    smul_go <= 1; ret_st <= S_NX_C; st <= S_SMUL;
                end
                S_NX_C: begin nxg <= pa - mres; st <= S_NY_A; end
                // nyg = diz2*e1x - diz1*e2x
                S_NY_A: begin
                    smul_a <= diz2; smul_b <= e1x;
                    smul_go <= 1; ret_st <= S_NY_B; st <= S_SMUL;
                end
                S_NY_B: begin
                    pa <= mres;
                    smul_a <= diz1; smul_b <= e2x;
                    smul_go <= 1; ret_st <= S_NY_C; st <= S_SMUL;
                end
                S_NY_C: begin nyg <= pa - mres; st <= S_DIVX; end

                // divides -------------------------------------------------
                S_DIVX:   begin div_num <= nxg; div_den <= area; div_go <= 1; st <= S_DIVX_W; end
                S_DIVX_W: if (div_done) begin diz_dx <= div_quo; st <= S_DIVY; end
                S_DIVY:   begin div_num <= nyg; div_den <= area; div_go <= 1; st <= S_DIVY_W; end
                S_DIVY_W: if (div_done) begin diz_dy <= div_quo; st <= S_IZ00_A; end

                // iz_at_00 = izp0 - diz_dx*p0x - diz_dy*p0y ---------------
                S_IZ00_A: begin
                    smul_a <= diz_dx; smul_b <= p0x;
                    smul_go <= 1; ret_st <= S_IZ00_B; st <= S_SMUL;
                end
                S_IZ00_B: begin
                    pa <= mres;
                    smul_a <= diz_dy; smul_b <= p0y;
                    smul_go <= 1; ret_st <= S_IZ00_C; st <= S_SMUL;
                end
                S_IZ00_C: begin
                    logic signed [63:0] acc64;
                    acc64 = $signed({{32{izp0[31]}}, izp0}) - pa - mres;
                    iz00 <= acc64[31:0];
                    st <= S_FIN;
                end

                // pack the record ----------------------------------------
                S_FIN: begin
                    rec[0] <= 16'($signed(p0x)*16 + 8);
                    rec[1] <= 16'($signed(p0y)*16 + 8);
                    rec[2] <= 16'($signed(p1x)*16 + 8);
                    rec[3] <= 16'($signed(p1y)*16 + 8);
                    rec[4] <= 16'($signed(p2x)*16 + 8);
                    rec[5] <= 16'($signed(p2y)*16 + 8);
                    rec[6] <= to_q11_5(iz00);
                    rec[7] <= to_q11_5(diz_dx);
                    rec[8] <= to_q11_5(diz_dy);
                    rec[9] <= {gg, bb};
                    rec[10] <= {8'b0, rr};
                    valid <= 1; done <= 1; st <= S_IDLE;
                end
                default: st <= S_IDLE;
            endcase
        end
    end
endmodule


//===========================================================================
// dot4: 4-lane Q12.20 dot product. Four parallel multipliers, registered
// products, then a registered 2-level adder tree. done pulses 3 cycles after
// start; result = sum_i (a_i*b_i)>>20 (each product truncated then summed,
// matching per-term fmul in the reference).
//===========================================================================
module dot4 #(parameter int SH = 20) (
    input  wire                clk,
    input  wire                start,
    input  wire signed [31:0]  a0,a1,a2,a3,
    input  wire signed [31:0]  b0,b1,b2,b3,
    output logic signed [31:0] result,
    output logic               done
);
    function automatic signed [31:0] fm(input signed [31:0] a, input signed [31:0] b);
        logic signed [63:0] p; p = a * b; return p >>> SH;
    endfunction
    logic signed [31:0] t0,t1,t2,t3, s01,s23, sum;
    logic d1,d2,d3;
    always_ff @(posedge clk) begin
        // stage 1: parallel multipliers + truncate, register products
        t0 <= fm(a0,b0); t1 <= fm(a1,b1); t2 <= fm(a2,b2); t3 <= fm(a3,b3); d1 <= start;
        // stage 2: adder tree level 1 (registered)
        s01 <= t0 + t1; s23 <= t2 + t3; d2 <= d1;
        // stage 3: adder tree level 2 (registered)
        sum <= s01 + s23; d3 <= d2;
    end
    assign result = sum;
    assign done   = d3;
endmodule


//===========================================================================
// smul: pipelined signed 32x32 -> 64 multiply. done 2 cycles after start.
//===========================================================================
module smul (
    input  wire                clk,
    input  wire                start,
    input  wire signed [31:0]  a, b,
    output logic signed [63:0] p,
    output logic               done
);
    logic signed [63:0] p1;
    logic d1, d2;
    always_ff @(posedge clk) begin
        p1 <= a * b; d1 <= start;
        p  <= p1;    d2 <= d1;
    end
    assign done = d2;
endmodule


//===========================================================================
// Sequential signed divider (trunc toward zero). 64-bit long division; the
// caller uses the low 32 bits of the quotient.
//===========================================================================
module divs (
    input  wire                clk,
    input  wire                rst,
    input  wire                start,
    input  wire signed [63:0]  num,
    input  wire signed [63:0]  den,
    output logic               done,
    output logic signed [31:0] quo
);
    logic busy;
    logic [6:0] cnt;
    logic [63:0] an, ad, rem, q;
    logic        neg;

    always_ff @(posedge clk) begin
        if (rst) begin
            busy <= 0; done <= 0;
        end else begin
            done <= 0;
            if (start && !busy) begin
                an   <= num[63] ? (~num + 1'b1) : num;
                ad   <= den[63] ? (~den + 1'b1) : den;
                neg  <= num[63] ^ den[63];
                rem  <= 0; q <= 0; cnt <= 0; busy <= 1;
            end else if (busy) begin
                logic [63:0] r2, qn;
                r2 = (rem <<< 1) | {63'b0, an[63]};
                an <= an <<< 1;
                if (r2 >= ad) begin rem <= r2 - ad; qn = (q <<< 1) | 64'd1; end
                else          begin rem <= r2;      qn = q <<< 1;          end
                q   <= qn;
                cnt <= cnt + 1'b1;
                if (cnt == 7'd63) begin
                    busy <= 0; done <= 1;
                    quo  <= neg ? (~qn[31:0] + 1'b1) : qn[31:0];
                end
            end
        end
    end
endmodule
