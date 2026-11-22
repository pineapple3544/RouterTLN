#include "el/teacher.hpp"
#include <png.h>
#include <fstream>
#include <iostream>
using namespace el;
int main(int argc,char** argv){if(argc!=2)return 2;TeacherProblem p;std::ifstream in(argv[1],std::ios::binary);in.read(reinterpret_cast<char*>(&p),sizeof(p));if(!in)return 2;
  std::cout<<"speed="<<p.state.z[3]<<" steer="<<p.state.z[2]<<" beta="<<p.state.z[6]<<" footprint="<<teacher_footprint(p,p.state)<<'\n';
  if(p.stable_grid){auto base=p;Memory memory("/home/misys/forza_ws/race_stack/stack_master/maps/hyhall12/hyhall12_orig.yaml");std::vector<uint8_t> cells;
    memory.copy_occupancy(cells,base.collision.width,base.collision.height,base.collision.resolution,base.collision.x0,base.collision.y0);std::copy(cells.begin(),cells.end(),base.collision.cells.begin());
    std::cout<<"base_map_footprint="<<teacher_footprint(base,base.state)<<" (difference indicates observed occupancy rather than a fixed wall)\n";}
  for(float steer:{p.state.z[2],0.f,-.2f,.2f}){auto s=p.state;int hit=-1;for(int t=0;t<200;++t){s=physics_step(s,{steer,0},p.vehicle);if(teacher_footprint(p,s)>.5f){hit=t;break;}if(s.z[3]<.05)break;}std::cout<<"brake steer="<<steer<<" collision_step="<<hit<<'\n';}
  constexpr int scale=5,n=grid_n*scale;std::vector<unsigned char> pixels(n*n*3,255);
  auto paint=[&](float x,float y,int red,int green,int blue,int radius){int ix=lroundf((x-p.grid.x0)/p.grid.resolution*scale),iy=n-1-lroundf((y-p.grid.y0)/p.grid.resolution*scale);
    for(int dx=-radius;dx<=radius;++dx)for(int dy=-radius;dy<=radius;++dy)if(ix+dx>=0&&ix+dx<n&&iy+dy>=0&&iy+dy<n){int k=((iy+dy)*n+ix+dx)*3;pixels[k]=red;pixels[k+1]=green;pixels[k+2]=blue;}};
  for(int y=0;y<grid_n;++y)for(int x=0;x<grid_n;++x)if(p.grid.cost[y*grid_n+x]>.5f)paint(p.grid.x0+(x+.5f)*p.grid.resolution,p.grid.y0+(y+.5f)*p.grid.resolution,0,0,0,2);
  for(auto r:p.ref)paint(r.x,r.y,0,80,255,2);paint(0,0,255,0,0,5);
  png_image img{};img.version=PNG_IMAGE_VERSION;img.width=img.height=n;img.format=PNG_FORMAT_RGB;std::string path=std::string(argv[1])+".png";png_image_write_to_file(&img,path.c_str(),0,pixels.data(),0,nullptr);
}
