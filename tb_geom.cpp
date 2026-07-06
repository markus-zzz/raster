// Unit test: geom_engine (HW) vs a C++ reference of process_face's Q12.20 math.
// Feeds identical inputs to both and checks the 11-word triangle record +
// cull decision are bit-exact.
#include <verilated.h>
#include "Vgeom_engine.h"
#include <cstdio>
#include <cstdint>
#include <cstdlib>

typedef int32_t fx;
static const int SH = 20;
static const fx  FX_ONE = 1 << SH;
static inline fx  fmul(fx a, fx b) { return (fx)(((int64_t)a * b) >> SH); }
static inline int fxfloor(fx a) { return a >> SH; }
static inline fx  clamp01(fx a) { return a < 0 ? 0 : (a > FX_ONE ? FX_ONE : a); }
static inline int16_t to_q11_5(fx a) {
    int64_t v = (int64_t)a * 32;
    v += (v >= 0) ? (1 << (SH - 1)) : -(1 << (SH - 1));
    return (int16_t)(v >> SH);
}
static inline fx fxi(int i)   { return (fx)(i << SH); }
static inline fx fxf(double d){ return (fx)llround(d * (double)(1 << SH)); }

// Reference: mirrors process_face()+write_tri_record() exactly.
struct Rec { bool valid; uint16_t w[11]; int32_t iz00, dzx, dzy; int64_t area, nxg; int e1x,e1y,e2x,e2y; };
static Rec ref_face(const int32_t mat[12], const int32_t v[9],
                    const int32_t nn[3], const int32_t col[3], const int32_t L[3]) {
    Rec o{}; 
    auto xf_pt = [&](int r, int b) {
        return fmul(mat[r*4+0],v[b+0]) + fmul(mat[r*4+1],v[b+1])
             + fmul(mat[r*4+2],v[b+2]) + mat[r*4+3];
    };
    auto xf_dir = [&](int r) {
        return fmul(mat[r*4+0],nn[0]) + fmul(mat[r*4+1],nn[1]) + fmul(mat[r*4+2],nn[2]);
    };
    fx vv[3][3];
    for (int i=0;i<3;i++){ vv[i][0]=xf_pt(0,i*3); vv[i][1]=xf_pt(1,i*3); vv[i][2]=xf_pt(2,i*3); }
    fx nx=xf_dir(0), ny=xf_dir(1), nz=xf_dir(2);
    if (nz >= 0) { o.valid=false; return o; }
    fx diffuse = fmul(nx,L[0])+fmul(ny,L[1])+fmul(nz,L[2]); if (diffuse<0) diffuse=0;
    fx intensity = fxf(0.2) + fmul(fxf(0.8), diffuse);
    uint8_t r = (uint8_t)fxfloor(fmul(clamp01(fmul(intensity,col[0])), fxi(255)));
    uint8_t g = (uint8_t)fxfloor(fmul(clamp01(fmul(intensity,col[1])), fxi(255)));
    uint8_t b = (uint8_t)fxfloor(fmul(clamp01(fmul(intensity,col[2])), fxi(255)));
    int sp[3][2]; fx iz[3];
    for (int i=0;i<3;i++){
        sp[i][0]=fxfloor(fmul(vv[i][0],fxi(80))+fxi(160));
        sp[i][1]=fxfloor(fmul(vv[i][1],fxi(80))+fxi(100));
        fx z=fxf(275.0)+fmul(vv[i][2],fxi(225)); iz[i]=(z>FX_ONE)?z:FX_ONE;
    }
    int p[3][2]; p[0][0]=sp[0][0];p[0][1]=sp[0][1];
    p[1][0]=sp[2][0];p[1][1]=sp[2][1]; p[2][0]=sp[1][0];p[2][1]=sp[1][1];
    fx izp[3]={iz[0],iz[2],iz[1]};
    int e1x=p[1][0]-p[0][0],e1y=p[1][1]-p[0][1],e2x=p[2][0]-p[0][0],e2y=p[2][1]-p[0][1];
    o.e1x=e1x; o.e1y=e1y; o.e2x=e2x; o.e2y=e2y;
    int64_t area=(int64_t)e1x*e2y-(int64_t)e1y*e2x;
    o.area=area; o.nxg=0;
    fx diz_dx=0,diz_dy=0;
    if (area!=0){
        fx diz1=izp[1]-izp[0], diz2=izp[2]-izp[0];
        int64_t nxg=(int64_t)diz1*e2y-(int64_t)diz2*e1y;
        int64_t nyg=(int64_t)diz2*e1x-(int64_t)diz1*e2x;
        o.nxg=nxg;
        diz_dx=(fx)(nxg/area); diz_dy=(fx)(nyg/area);
    }
    o.dzx=diz_dx; o.dzy=diz_dy;
    fx iz00=izp[0]-(fx)((int64_t)diz_dx*p[0][0])-(fx)((int64_t)diz_dy*p[0][1]);
    o.iz00 = iz00;
    o.w[0]=(uint16_t)(p[0][0]*16+8); o.w[1]=(uint16_t)(p[0][1]*16+8);
    o.w[2]=(uint16_t)(p[1][0]*16+8); o.w[3]=(uint16_t)(p[1][1]*16+8);
    o.w[4]=(uint16_t)(p[2][0]*16+8); o.w[5]=(uint16_t)(p[2][1]*16+8);
    o.w[6]=(uint16_t)to_q11_5(iz00); o.w[7]=(uint16_t)to_q11_5(diz_dx); o.w[8]=(uint16_t)to_q11_5(diz_dy);
    uint32_t color=(r<<16)|(g<<8)|b; o.w[9]=color&0xFFFF; o.w[10]=(color>>16)&0xFF;
    o.valid=true; return o;
}

static Vgeom_engine *dut;
static void tick(){ dut->clk=0; dut->eval(); dut->clk=1; dut->eval(); }

static bool run_hw(const int32_t mat[12], const int32_t v[9], const int32_t nn[3],
                   const int32_t col[3], const int32_t L[3], Rec &out) {
    for (int i=0;i<12;i++) dut->mat[i]=mat[i];
    for (int i=0;i<9;i++)  dut->vtx[i]=v[i];
    for (int i=0;i<3;i++){ dut->nrm[i]=nn[i]; dut->col[i]=col[i]; dut->light[i]=L[i]; }
    dut->start=1; tick(); dut->start=0;
    for (int c=0;c<400;c++){ tick(); if (dut->done){
        out.valid=dut->valid;
        for (int i=0;i<11;i++) out.w[i]=dut->rec[i];
        return true;
    }}
    return false; // timeout
}

static int32_t rnd_q(int lo_num, int hi_num) { // random Q12.20 in [lo,hi] (integers scaled)
    double r = lo_num + (double)rand()/RAND_MAX * (hi_num - lo_num);
    return fxf(r);
}

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vgeom_engine;
    dut->rst=1; tick(); tick(); dut->rst=0;

    // A representative rotation-ish matrix + light (arbitrary; the datapath is
    // input-agnostic for bit-exactness).
    int32_t mat[12] = {
        fxf( 0.87), fxf(0.0),  fxf(0.5),  fxf(0.0),
        fxf( 0.15), fxf(-0.9), fxf(-0.26),fxf(0.0),
        fxf(-0.45), fxf(-0.3), fxf( 0.78),fxf(0.0)
    };
    int32_t L[3] = { fxf(-0.188144), fxf(-0.282216), fxf(-0.940719) };

    srand(1234);
    int n=4000, fails=0, culled=0, drawn=0;
    for (int t=0;t<n;t++){
        int32_t v[9]; for (int i=0;i<9;i++) v[i]=rnd_q(-2,2);
        int32_t nn[3]={ rnd_q(-1,1), rnd_q(-1,1), rnd_q(-1,1) };
        int32_t col[3]={ (int32_t)(rand()%(FX_ONE+1)), (int32_t)(rand()%(FX_ONE+1)),
                         (int32_t)(rand()%(FX_ONE+1)) };
        Rec ref=ref_face(mat,v,nn,col,L), hw{};
        if (!run_hw(mat,v,nn,col,L,hw)){ printf("timeout at %d\n",t); fails++; continue; }
        if (ref.valid!=hw.valid){ if(fails<10)printf("t%d valid ref=%d hw=%d\n",t,ref.valid,hw.valid); fails++; continue; }
        if (!ref.valid){ culled++; continue; }
        drawn++;
        bool bad=false;
        for (int i=0;i<11;i++) if (ref.w[i]!=hw.w[i]){
            if (fails<20) printf("t%d rec[%d] ref=%u hw=%u\n", t,i,ref.w[i],hw.w[i]);
            bad=true;
        }
        if (bad) fails++;
    }
    printf("tested=%d drawn=%d culled=%d fails=%d -> %s\n",
           n, drawn, culled, fails, fails? "FAIL":"PASS");
    delete dut;
    return fails ? 1 : 0;
}
