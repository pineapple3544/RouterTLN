#include "el/core.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <algorithm>
using namespace el;
// Frozen pre-fix inverse sensor model, independently replayed to prevent train/serve skew.
struct LegacyFeatures {
 std::vector<uint8_t> base;std::vector<float> odds;std::vector<double> seen;int w,h;float resolution,x0,y0;
 explicit LegacyFeatures(Memory& memory){memory.copy_occupancy(base,w,h,resolution,x0,y0);odds.assign(base.size(),0);seen.assign(base.size(),-1);}
 int index(float x,float y)const{int c=(int)floor((x-x0)/resolution),r=(int)floor((y-y0)/resolution);return c<0||r<0||c>=w||r>=h?-1:r*w+c;}
 void update(const Scan& scan){std::vector<int> hits,free_cells;float sx=scan.pose.x+.27f*cosf(scan.pose.yaw),sy=scan.pose.y+.27f*sinf(scan.pose.yaw);
  for(int b=0;b<beams;++b){if(scan.valid[b]<.5f)continue;float r=clamp(scan.ranges[b],0,scan.range_max),a=scan.pose.yaw+(scan.explicit_angles?scan.angles[b]:scan.angle_min+(scan.angle_max-scan.angle_min)*b/(beams-1));float c=cosf(a),q=sinf(a);bool hit=r<scan.range_max-.02f;
   for(float d=0;d<r-resolution;d+=resolution*.7f){int k=index(sx+c*d,sy+q*d);if(k>=0)free_cells.push_back(k);}if(hit){int k=index(sx+c*r,sy+q*r);if(k>=0)hits.push_back(k);}}
  auto unique=[](std::vector<int>& a){std::sort(a.begin(),a.end());a.erase(std::unique(a.begin(),a.end()),a.end());};unique(hits);unique(free_cells);
  for(int k:free_cells)if(!std::binary_search(hits.begin(),hits.end(),k)){odds[k]=fmaxf(-3.f,odds[k]-.7f);seen[k]=scan.stamp;}for(int k:hits){odds[k]=fminf(4.f,odds[k]+1.f);seen[k]=scan.stamp;}
 }
 Observation features(State s,double now)const{Observation o;float c=cosf(s.yaw),q=sinf(s.yaw);
  for(int y=0;y<crop;++y)for(int x=0;x<crop;++x){float lx=-2.f+(x+.5f)*.35f,ly=-5.6f+(y+.5f)*.35f;int k=index(s.x+c*lx-q*ly,s.y+q*lx+c*ly),i=y*crop+x;
   o.map[i]=k<0||base[k]||odds[k]>.5f?1.f:0.f;o.map[crop*crop+i]=k>=0&&seen[k]>=0?1.f:0.f;o.map[2*crop*crop+i]=k>=0&&seen[k]>=0?clamp((now-seen[k])/60.,0,1):1.f;}
  return o;
 }
};
int main(int argc,char** argv){try{
 if(argc!=3)throw std::runtime_error("Usage: el_memory_replay_test SCANS MAP_YAML");Memory memory(argv[2]);LegacyFeatures legacy(memory);
 std::ifstream f(argv[1],std::ios::binary);uint64_t size;f.read(reinterpret_cast<char*>(&size),sizeof(size));if(size!=sizeof(Scan))throw std::runtime_error("bad archive");
 Scan scan;size_t count=0,values=0;while(f.read(reinterpret_cast<char*>(&scan),sizeof(scan))){legacy.update(scan);memory.update(scan);Observation actual;memory.features(scan.pose,scan.stamp,actual);auto expected=legacy.features(scan.pose,scan.stamp);
  for(size_t i=0;i<actual.map.size();++i){++values;if(actual.map[i]!=expected.map[i])throw std::runtime_error("schema-3 feature mismatch at scan "+std::to_string(count)+" feature "+std::to_string(i));}++count;}
 if(count==0)throw std::runtime_error("empty replay");std::cout<<nlohmann::json{{"scans",count},{"feature_values_exactly_equal",values},{"wall_associated_returns",memory.associated_wall_returns()},{"schema3_feature_parity",true}}.dump(2)<<'\n';return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
