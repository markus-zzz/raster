// Unit test: burst geom_front (via real sdram_ctrl + sdram_model) vs C++ ref.
// Loads Suzanne frame 0, backdoor-preloads the geometry inputs, runs the HW
// geometry pass, and compares TRI / BIN / BINLIST regions bit-exact.
#include <verilated.h>
#include "Vgeom_tb_top.h"
#include "Vgeom_tb_top_geom_tb_top.h"
#include "Vgeom_tb_top_sdram_model.h"
#include "Vgeom_tb_top_dpram__A12_DB30000.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <vector>
#include <algorithm>

typedef int32_t fx; static const int SH=20; static const fx FX_ONE=1<<SH;
static inline fx fmul(fx a,fx b){return (fx)(((int64_t)a*b)>>SH);}
static inline int fxfloor(fx a){return a>>SH;}
static inline fx clamp01(fx a){return a<0?0:(a>FX_ONE?FX_ONE:a);}
static inline fx fxi(int i){return (fx)(i<<SH);}
static inline fx fxf(double d){return (fx)llround(d*(double)(1<<SH));}
static inline int16_t to_q11_5(fx a){int64_t v=(int64_t)a*32; v+=(v>=0)?(1<<(SH-1)):-(1<<(SH-1)); return (int16_t)(v>>SH);}

// memory map (halfword addresses), all within 0x20000 (128K model)
static const int TRI_BASE=0x00000, BIN_BASE=0x04000, BINLIST_BASE=0x05000;
static const int MATRIX_BASE=0x0A000, LIGHT_BASE=0x0A020, VTX_BASE=0x0A100, FACE_BASE=0x0C000;
static const int MAXF=1024, NTX=5, NTY=4, NUM_TILES=NTX*NTY, TW=64, TH=64;

struct Face { int v[3]; fx nrm[3]; fx col[3]; };
static std::vector<glm::vec3> gv;
static std::vector<Face> faces;

static void load_obj(const char*path){
    FILE*f=fopen(path,"r"); if(!f){printf("no %s\n",path);exit(1);}
    char line[256];
    while(fgets(line,sizeof line,f)){
        if(line[0]=='v'&&line[1]==' '){glm::vec3 v;sscanf(line+2,"%f %f %f",&v.x,&v.y,&v.z);gv.push_back(v);}
        else if(line[0]=='f'&&line[1]==' '){
            int vi[4]={0},c=0;char*p=line+2;
            while(*p&&*p!='\n'&&c<4){ if(*p>='0'&&*p<='9'){vi[c++]=atoi(p)-1;while(*p&&*p!=' '&&*p!='\n')p++;} else p++; }
            if(c>=3){ faces.push_back({{vi[0],vi[1],vi[2]},{0,0,0},{0,0,0}});
                      if(c==4) faces.push_back({{vi[0],vi[2],vi[3]},{0,0,0},{0,0,0}}); }
        }
    }
    fclose(f);
    for(auto&fc:faces){
        glm::vec3 n=glm::normalize(glm::cross(gv[fc.v[1]]-gv[fc.v[0]],gv[fc.v[2]]-gv[fc.v[0]]));
        fc.nrm[0]=fxf(-n.x); fc.nrm[1]=fxf(-n.y); fc.nrm[2]=fxf(-n.z);
        fc.col[0]=fxf(0.5f+0.5f*n.x); fc.col[1]=fxf(0.5f+0.5f*n.y); fc.col[2]=fxf(0.5f+0.5f*n.z);
    }
}
static void build_matrix(float ay,float ax, fx mat[12]){
    glm::mat4 g(1.0f);
    g=glm::rotate(g,ax,glm::vec3(1,0,0)); g=glm::rotate(g,ay,glm::vec3(0,1,0));
    g=glm::scale(g,glm::vec3(1,-1,1)); glm::mat3 R(g);
    for(int i=0;i<3;i++){ for(int j=0;j<3;j++) mat[i*4+j]=fxf(R[j][i]); mat[i*4+3]=0; }
}
struct Ref { bool valid; uint16_t w[11]; int bbminx,bbminy,bbmaxx,bbmaxy; };
static Ref ref_face(const fx mat[12], const fx v[9], const fx nn[3], const fx col[3], const fx L[3]){
    Ref o{};
    auto xp=[&](int r,int b){return fmul(mat[r*4+0],v[b])+fmul(mat[r*4+1],v[b+1])+fmul(mat[r*4+2],v[b+2])+mat[r*4+3];};
    auto xd=[&](int r){return fmul(mat[r*4+0],nn[0])+fmul(mat[r*4+1],nn[1])+fmul(mat[r*4+2],nn[2]);};
    fx vv[3][3]; for(int i=0;i<3;i++){vv[i][0]=xp(0,i*3);vv[i][1]=xp(1,i*3);vv[i][2]=xp(2,i*3);}
    fx nz=xd(2); if(nz>=0){o.valid=false;return o;}
    fx nx=xd(0),ny=xd(1);
    fx diffuse=fmul(nx,L[0])+fmul(ny,L[1])+fmul(nz,L[2]); if(diffuse<0)diffuse=0;
    fx intensity=fxf(0.2)+fmul(fxf(0.8),diffuse);
    uint8_t r=(uint8_t)fxfloor(fmul(clamp01(fmul(intensity,col[0])),fxi(255)));
    uint8_t g=(uint8_t)fxfloor(fmul(clamp01(fmul(intensity,col[1])),fxi(255)));
    uint8_t b=(uint8_t)fxfloor(fmul(clamp01(fmul(intensity,col[2])),fxi(255)));
    int sp[3][2]; fx iz[3];
    for(int i=0;i<3;i++){ sp[i][0]=fxfloor(fmul(vv[i][0],fxi(80))+fxi(160));
                          sp[i][1]=fxfloor(fmul(vv[i][1],fxi(80))+fxi(100));
                          fx z=fxf(275.0)+fmul(vv[i][2],fxi(225)); iz[i]=(z>FX_ONE)?z:FX_ONE; }
    int p[3][2]; p[0][0]=sp[0][0];p[0][1]=sp[0][1];p[1][0]=sp[2][0];p[1][1]=sp[2][1];p[2][0]=sp[1][0];p[2][1]=sp[1][1];
    fx izp[3]={iz[0],iz[2],iz[1]};
    int e1x=p[1][0]-p[0][0],e1y=p[1][1]-p[0][1],e2x=p[2][0]-p[0][0],e2y=p[2][1]-p[0][1];
    int64_t area=(int64_t)e1x*e2y-(int64_t)e1y*e2x; fx dzx=0,dzy=0;
    if(area!=0){ fx d1=izp[1]-izp[0],d2=izp[2]-izp[0];
        dzx=(fx)(((int64_t)d1*e2y-(int64_t)d2*e1y)/area);
        dzy=(fx)(((int64_t)d2*e1x-(int64_t)d1*e2x)/area); }
    fx iz00=izp[0]-(fx)((int64_t)dzx*p[0][0])-(fx)((int64_t)dzy*p[0][1]);
    o.w[0]=(uint16_t)(p[0][0]*16+8);o.w[1]=(uint16_t)(p[0][1]*16+8);
    o.w[2]=(uint16_t)(p[1][0]*16+8);o.w[3]=(uint16_t)(p[1][1]*16+8);
    o.w[4]=(uint16_t)(p[2][0]*16+8);o.w[5]=(uint16_t)(p[2][1]*16+8);
    o.w[6]=(uint16_t)to_q11_5(iz00);o.w[7]=(uint16_t)to_q11_5(dzx);o.w[8]=(uint16_t)to_q11_5(dzy);
    uint32_t color=(r<<16)|(g<<8)|b; o.w[9]=color&0xFFFF; o.w[10]=(color>>16)&0xFF;
    o.bbminx=std::min({p[0][0],p[1][0],p[2][0]}); o.bbmaxx=std::max({p[0][0],p[1][0],p[2][0]});
    o.bbminy=std::min({p[0][1],p[1][1],p[2][1]}); o.bbmaxy=std::max({p[0][1],p[1][1],p[2][1]});
    o.valid=true; return o;
}

static Vgeom_tb_top *dut;
static void tick(){ dut->clk=0; dut->eval(); dut->clk=1; dut->eval(); }

int main(int argc,char**argv){
    Verilated::commandArgs(argc,argv);
    load_obj("suzanne.obj");
    printf("loaded %zu verts, %zu faces\n", gv.size(), faces.size());
    fx mat[12]; build_matrix(0.0f,-0.3f,mat);
    glm::vec3 nl=-glm::normalize(glm::vec3(0.2f,0.3f,1.0f));
    fx light[3]={fxf(nl.x),fxf(nl.y),fxf(nl.z)};

    dut=new Vgeom_tb_top;
    dut->rst=1; dut->start=0; dut->nfaces=0;
    tick(); tick(); dut->rst=0;
    auto &MEM = dut->geom_tb_top->sdram->mem_inst->mem;
    auto w32 = [&](int a, uint32_t v){ MEM[a]=v&0xFFFF; MEM[a+1]=(v>>16)&0xFFFF; };

    // SDRAM init (~100us)
    for(int i=0;i<11000;i++) tick();

    // preload inputs
    for(int k=0;k<12;k++) w32(MATRIX_BASE+k*2,mat[k]);
    for(int k=0;k<3;k++)  w32(LIGHT_BASE+k*2,light[k]);
    for(size_t i=0;i<gv.size();i++){ w32(VTX_BASE+i*8+0,fxf(gv[i].x)); w32(VTX_BASE+i*8+2,fxf(gv[i].y)); w32(VTX_BASE+i*8+4,fxf(gv[i].z)); }
    for(size_t f=0;f<faces.size();f++){ int b=FACE_BASE+f*16;
        MEM[b+0]=faces[f].v[0]; MEM[b+1]=faces[f].v[1]; MEM[b+2]=faces[f].v[2];
        w32(b+3,faces[f].nrm[0]); w32(b+5,faces[f].nrm[1]); w32(b+7,faces[f].nrm[2]);
        w32(b+9,faces[f].col[0]); w32(b+11,faces[f].col[1]); w32(b+13,faces[f].col[2]); }

    // reference
    std::vector<uint16_t> rtri(faces.size()*16+16,0), rcount(NUM_TILES,0);
    std::vector<std::vector<uint16_t>> rbucket(NUM_TILES);
    int ntri=0;
    for(size_t f=0;f<faces.size();f++){
        fx v[9]; for(int i=0;i<3;i++){int vi=faces[f].v[i]; v[i*3+0]=fxf(gv[vi].x);v[i*3+1]=fxf(gv[vi].y);v[i*3+2]=fxf(gv[vi].z);}
        Ref r=ref_face(mat,v,faces[f].nrm,faces[f].col,light);
        if(!r.valid) continue;
        for(int i=0;i<11;i++) rtri[ntri*16+i]=r.w[i];
        int txmin=std::max(0,r.bbminx/TW), txmax=std::min(NTX-1,r.bbmaxx/TW);
        int tymin=std::max(0,r.bbminy/TH), tymax=std::min(NTY-1,r.bbmaxy/TH);
        for(int ty=tymin;ty<=tymax;ty++) for(int tx=txmin;tx<=txmax;tx++){ int tile=ty*NTX+tx; rbucket[tile].push_back(ntri); rcount[tile]++; }
        ntri++;
    }

    // run HW
    dut->nfaces=faces.size();
    dut->start=1; tick(); dut->start=0;
    long timeout=faces.size()*4000L+200000; bool done=false;
    for(long c=0;c<timeout;c++){ tick(); if(dut->done){done=true;break;} }
    if(!done){ printf("TIMEOUT\n"); return 1; }

    int fails=0;
    for(int t=0;t<ntri && fails<20;t++)
        for(int i=0;i<11;i++) if(MEM[TRI_BASE+t*16+i]!=rtri[t*16+i]){
            printf("tri %d rec[%d] hw=%u ref=%u\n",t,i,MEM[TRI_BASE+t*16+i],rtri[t*16+i]); fails++; break; }
    for(int t=0;t<NUM_TILES;t++){
        if(MEM[BIN_BASE+t]!=rcount[t]){ printf("count tile %d hw=%u ref=%u\n",t,MEM[BIN_BASE+t],rcount[t]); fails++; }
        for(size_t j=0;j<rbucket[t].size() && fails<40;j++)
            if(MEM[BINLIST_BASE+t*MAXF+j]!=rbucket[t][j]){
                printf("bucket tile %d [%zu] hw=%u ref=%u\n",t,j,MEM[BINLIST_BASE+t*MAXF+j],rbucket[t][j]); fails++; break; }
    }
    printf("ntri=%d fails=%d -> %s\n",ntri,fails,fails?"FAIL":"PASS");
    delete dut; return fails?1:0;
}
