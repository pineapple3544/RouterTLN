#include "el/teacher.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <set>
using namespace el;using json=nlohmann::json;
int main(int argc,char** argv){try{
 if(argc!=5)throw std::runtime_error("Usage: el_memory_trace SCANS MAP_YAML FAILURE_PROBLEM OUTPUT.json");
 TeacherProblem problem;{std::ifstream f(argv[3],std::ios::binary);f.read(reinterpret_cast<char*>(&problem),sizeof(problem));if(!f)throw std::runtime_error("bad problem");}
 Memory memory(argv[2]);std::vector<uint8_t> base;int w,h;float res,x0,y0;memory.copy_occupancy(base,w,h,res,x0,y0);
 auto index=[&](float x,float y){int ix=floorf((x-x0)/res),iy=floorf((y-y0)/res);return ix<0||iy<0||ix>=w||iy>=h?-1:iy*w+ix;};
 std::set<int> targets;auto grid=problem.collision;
 for(int i=0;i<7;++i)for(int j=0;j<5;++j){float x=-.33f+i*.66f/6,y=-.1416f+j*.2832f/4;int k=index(grid.ego_x+grid.c*x-grid.s*y,grid.ego_y+grid.s*x+grid.c*y);if(k>=0&&!base[k]&&grid.cells[k])targets.insert(k);}
 json report={{"odds_field_definition","Legacy endpoint-only update reconstructed for diagnosis; final occupied uses the linked Memory implementation"},{"targets",json::array()},{"hits",json::array()},{"updates",json::array()}};std::map<int,float> odds;
 for(int k:targets)report["targets"].push_back({{"index",k},{"x0",x0+(k%w)*res},{"y0",y0+(k/w)*res},{"resolution",res}});
 std::ifstream scans(argv[1],std::ios::binary);uint64_t size;scans.read(reinterpret_cast<char*>(&size),sizeof(size));if(size!=sizeof(Scan))throw std::runtime_error("scan schema mismatch");Scan scan;int count=0;
 while(scans.read(reinterpret_cast<char*>(&scan),sizeof(scan))){++count;memory.update(scan);std::set<int> hits,free;
  float sx=scan.pose.x+.27f*cosf(scan.pose.yaw),sy=scan.pose.y+.27f*sinf(scan.pose.yaw);
  for(int b=0;b<beams;++b){if(scan.valid[b]<.5f)continue;float r=clamp(scan.ranges[b],0,scan.range_max),a=scan.pose.yaw+(scan.explicit_angles?scan.angles[b]:scan.angle_min+(scan.angle_max-scan.angle_min)*b/(beams-1)),c=cosf(a),s=sinf(a);
   for(float d=0;d<r-res;d+=res*.7f){int k=index(sx+c*d,sy+s*d);if(targets.count(k))free.insert(k);}
   int k=index(sx+c*r,sy+s*r);if(r>=scan.range_max-.02||!targets.count(k))continue;hits.insert(k);
   double wall=-1;for(double d=0;d<r+1;d+=res*.05){int cell=index(sx+c*d,sy+s*d);if(cell<0||base[cell]){wall=d;break;}}
   double distance=1e9;float x=sx+c*r,y=sy+s*r;int cx=floorf((x-x0)/res),cy=floorf((y-y0)/res);
   for(int yy=cy-5;yy<=cy+5;++yy)for(int xx=cx-5;xx<=cx+5;++xx)if(xx>=0&&yy>=0&&xx<w&&yy<h&&base[yy*w+xx]){double dx=std::max({double(x0+xx*res-x),0.,double(x-(x0+(xx+1)*res))}),dy=std::max({double(y0+yy*res-y),0.,double(y-(y0+(yy+1)*res))});distance=std::min(distance,hypot(dx,dy));}
   report["hits"].push_back({{"stamp",scan.stamp},{"beam",b},{"cell",k},{"range",r},{"wall_range",wall},{"range_residual",wall>=0?json(r-wall):json(nullptr)},{"endpoint_base_wall_distance",distance},{"endpoint",{x,y}}});
  }
  for(int k:targets){if(hits.count(k)){odds[k]=fminf(4,odds[k]+1);report["updates"].push_back({{"stamp",scan.stamp},{"cell",k},{"type","hit"},{"odds",odds[k]}});}else if(free.count(k)){odds[k]=fmaxf(-3,odds[k]-.7f);report["updates"].push_back({{"stamp",scan.stamp},{"cell",k},{"type","free"},{"odds",odds[k]}});}}
 }
 std::vector<uint8_t> end;memory.copy_occupancy(end,w,h,res,x0,y0);report["scans"]=count;report["final"]=json::array();for(int k:targets)report["final"].push_back({{"cell",k},{"odds",odds[k]},{"occupied",end[k]}});
 std::ofstream(argv[4])<<report.dump(2);std::cout<<"scans="<<count<<" targets="<<targets.size()<<" hits="<<report["hits"].size()<<'\n';return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
