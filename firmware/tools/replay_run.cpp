// Host tool: replay a logged run (tools/read_run.py --csv) through the Navigator's random-block detection.
//   build:  g++ -std=gnu++17 -O1 -DNAV_LOW_MEMORY -I lib/course_nav/include -I lib/base/include -I include \
//               tools/replay_run.cpp lib/course_nav/src/planner.cpp lib/course_nav/src/navigator.cpp \
//               lib/course_nav/src/geometry.cpp -o /tmp/replay_run
//   usage:  /tmp/replay_run run.csv 1            report the random blocks it would find (use a run with NONE: any is a phantom)
//           /tmp/replay_run run.csv 0 d          print the unexplained hit points (HIT,x,y,sensor) for tools/make_background.py
//           /tmp/replay_run run.csv 1 b bg.bin [wallMargin mapMargin [confirmSteps [pathReach [minX [rangeTol [baseline.bin]]]]]]   as the first, with a background bit grid
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <vector>
#include "navigator.h"
#include "config.h"
#include "map_data.h"
static unsigned char gBits[200]; static std::vector<nav::Navigator::BaselineEntry> gBase;
int main(int argc,char**argv){
  FILE*f=fopen(argv[1],"r"); char line[256]; fgets(line,sizeof line,f);
  struct R{int ms,x,y,th,al,tl,tr,ml,mr,t0,t1,t2;}; std::vector<R> rows; R r;
  while(fgets(line,sizeof line,f)){ if(sscanf(line,"%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",&r.ms,&r.x,&r.y,&r.th,&r.al,&r.tl,&r.tr,&r.ml,&r.mr,&r.t0,&r.t1,&r.t2)==12) rows.push_back(r); }
  nav::NavParams p; p.vFull=cfg::V_FULL_MM_S; p.track=cfg::WHEEL_TRACK_MM;
  p.unmappedMinX=1150; p.unmappedMaxX=1800; p.unmappedMinY=60; p.maxExtras=argc>2?atoi(argv[2]):1; if(argc>6){ p.wallMargin=atof(argv[5]); p.mapMargin=atof(argv[6]); } if(argc>7) p.confirmSteps=atoi(argv[7]); if(argc>8) p.pathReach=atof(argv[8]); if(argc>9) p.unmappedMinX=atof(argv[9]); if(argc>10) p.rangeTol=atof(argv[10]);
  nav::Navigator* gN=new nav::Navigator(cfg::TOF_LAYOUT,p); gN->init(MAP_BLOCKS,MAP_BLOCK_COUNT);
  bool dump = argc>3 && argv[3][0]=='d';
  if(argc>4 && argv[3][0]=='b'){ FILE*b=fopen(argv[4],"rb"); fread(gBits,1,200,b); fclose(b); nav::Navigator::Background bg; bg.bits=gBits; bg.x0=1100; bg.y0=-200; bg.cell=40; bg.nx=18; bg.ny=35; gN->setBackground(bg); }
  if(argc>11){ FILE*b=fopen(argv[11],"rb"); if(b){ nav::Navigator::BaselineEntry q; while(fread(&q,sizeof q,1,b)==1) gBase.push_back(q); fclose(b);
    nav::Navigator::Baseline bl; bl.e=gBase.data(); bl.n=(int)gBase.size(); gN->setBaseline(bl); } }
  gN->start({300,-80,1.5708f},0.0f);
  int last=0;
  for(auto&q:rows){
    math::Pose e={(float)q.x,(float)q.y,q.th/1000.0f}; float tof[3]={(float)q.t0,(float)q.t1,(float)q.t2}; float duty[2];
    gN->step(e,(q.ml+q.mr)/2.0f,tof,q.ms/1000.0f,duty);
    if(dump){ for(int k=0;k<3;k++) if(gN->unexplainedStreak(k)>=1){ const auto&m=cfg::TOF_LAYOUT.sensors[k]; float c=cosf(e.th),sn=sinf(e.th); float ox=e.x+c*m.x-sn*m.y, oy=e.y+sn*m.x+c*m.y; float a=e.th+m.angle; float z=tof[k]; if(z<800) printf("HIT,%.0f,%.0f,%d\n",ox+z*cosf(a),oy+z*sinf(a),k);} }
    if(gN->unmappedBlocks()!=last){ last=gN->unmappedBlocks(); auto c=gN->planner().extra(last-1);
      printf("  t=%.2fs pose(%d,%d,%.2f) tof R%d F%d L%d -> extra #%d at (%.0f,%.0f)\n",q.ms/1000.0,q.x,q.y,q.th/1000.0,q.t0,q.t1,q.t2,last,c.x,c.y); }
  }
  if(!dump) printf("  total extras: %d, replans %d\n",gN->unmappedBlocks(),gN->replans());
}
