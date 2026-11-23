#include "el/core.hpp"
#include "el/dynamics.hpp"
#include "el/avoid_corridor.hpp"
#include "el/lap_plan.hpp"
#include "el/race_speed.hpp"
#include "el/mppi.hpp"
#include "el/dataset.hpp"
#include "el/scenarios.hpp"
#include "el/eval_metrics.hpp"
#include <nlohmann/json.hpp>
#include <png.h>
#include <fstream>
#include <iostream>
#include <filesystem>
#include <stdexcept>
using namespace el;
void require(bool condition,const char* msg){if(!condition)throw std::runtime_error(msg);}
void write_map(const std::string& dir){
  std::filesystem::create_directories(dir);std::vector<unsigned char> pixels(200*200,255);
  png_image img{};img.version=PNG_IMAGE_VERSION;img.width=img.height=200;img.format=PNG_FORMAT_GRAY;
  require(png_image_write_to_file(&img,(dir+"/map.png").c_str(),0,pixels.data(),0,nullptr),"write fixture PNG");
  std::ofstream f(dir+"/map.yaml");f<<"image: map.png\nresolution: 0.05\norigin: [-5,-5,0]\nnegate: 0\nfree_thresh: 0.25\n";
}
Problem straight(){Problem p;for(int i=0;i<ref_n;++i)p.ref[i]={i*.15f,0,0,2.f,i*.15f};p.max_speed=2;return p;}
int main(){try{
  {
    // A slow entry just after the lap seam must constrain the previous lap.
    std::vector<float> speed2{4,36,36,36,36,36,36,36};
    std::vector<float> ds(speed2.size(), .5f);
    el::plan_cyclic_speeds(speed2,ds,2.f,3.f);
    require(speed2[7]==7.f && speed2[6]==10.f, "brake before wrapped entry");
    require(speed2[1]==6.f && speed2[2]==8.f, "limit acceleration after entry");
    for (size_t i=0; i<speed2.size(); ++i) {
      size_t next=(i+1)%speed2.size();
      require(speed2[next]<=speed2[i]+2.f && speed2[i]<=speed2[next]+3.f,
              "cyclic longitudinal reachability");
    }
  }
  {
    Scan sight;
    sight.valid.fill(1);
    sight.ranges.fill(8);
    const float open = race_sight_speed_limit(sight, 6.f);
    sight.ranges.fill(2);
    const float narrow = race_sight_speed_limit(sight, 6.f);
    require(open > narrow && narrow > 0, "Race speed follows visible braking distance");
    require(.20f*narrow+narrow*narrow/12.f <= 2.f-.35f+.0001f, "latency and nose reserve");
    sight.ranges[beams/2] = .2f;
    require(race_sight_speed_limit(sight, 6.f) == 0, "obstruction inside nose reserve");
    sight.valid[beams/2] = 0;
    sight.ranges.fill(8);
    require(race_sight_speed_limit(sight, 6.f) == 0, "invalid forward beam");
    sight.valid.fill(1);
    sight.angle_min = .01f;
    require(race_sight_speed_limit(sight, 6.f) == 0, "missing half the forward scan");
    RouterSettings profile;
    profile.race_sight_speed = true;
    bool rejected = false;
    try { profile.validate_sim_band(5.f, true); }
    catch (const std::runtime_error&) { rejected = true; }
    require(rejected, "Race envelope requires a Race-only scout profile");
  }
  {
    // Replay the five 10 ms ticks of the accelerating wall approach.
    DynamicState state{};
    state.z[3]=3.07265f;
    const Control target{.32f,4.9f};
    state=physics_step(state,target,Vehicle{});
    state=physics_step(state,target,Vehicle{});
    require(state.z[2]==0,"two ticks of steering actuator delay");
    state=physics_step(state,target,Vehicle{});
    require(fabsf(state.z[2]-.032f)<1e-6f,"rate-limited steering after delay");
    state=physics_step(state,target,Vehicle{});
    state=physics_step(state,target,Vehicle{});
    require(fabsf(state.z[3]-3.467704f)<1e-4f,"accelerating approach speed after 50 ms");
  }
  {
    RouterSettings band;
    band.validate_sim_band(3,false);
    band.sim_avoid_min=2.4f;
    band.sim_avoid_max=2.8f;
    auto rejected_band=[&](float cap,bool sim){
      try {band.validate_sim_band(cap,sim);return false;}
      catch(const std::runtime_error&){return true;}
    };
    require(rejected_band(3,true),"speed probe requires a scout plan");
    band.lap_planning=true;
    require(rejected_band(3,false),"speed probe only in local simulation");
    require(rejected_band(2.7f,true),"speed probe cannot exceed global cap");
    band.validate_sim_band(3,true);
    band.sim_avoid_min=3;
    require(rejected_band(3,true),"speed probe rejects reversed bounds");
    // The same map can have different scout starts. Translate a remembered
    // obstacle section into map stations, including a section crossing the seam.
    std::vector<Waypoint> loop;
    float station=0;
    for(int i=0;i<=100;++i){
      float angle=2*pi*i/100;
      if(i)station+=2*10.f*sinf(pi/100);
      loop.push_back({10*cosf(angle),10*sinf(angle),wrap(angle+pi/2),6,station});
    }
    LapPlan speed_plan;
    RouterSettings profile;
    auto limit=[&](int i,float relative,float cruise){
      const auto& w=loop[i];
      return speed_plan.race_speed(loop,i,relative,State{w.x,w.y,w.yaw},cruise,2.7f);
    };
    require(limit(0,0,5)==0,"no flying plan before successful scout");
    // Starting halfway round puts the mapped start/finish inside this section.
    const float origin=loop[50].s;
    speed_plan.enter(loop[97].s-origin,1,2,2.2f,false);
    speed_plan.returned(loop[3].s+station-origin);
    for(size_t i=0;i<loop.size();++i)speed_plan.observe(i,loop.size(),0);
    speed_plan.update(station+.1f,station,INFINITY,2,5,profile);
    require(fabsf(limit(0,station+station-origin,5)-2.7f)<.001f,"wrapped scout obstacle binds");
    require(limit(50,station,5)>4.9f,"clear section can accelerate");
    require(limit(50,station,4)<=4.f,"new cruise cap rebuilds envelope");
    LapPlan plan;
    RouterSettings settings;
    auto update=[&](float pos,float ahead){return plan.update(pos,40.f,ahead,2.f,3.f,settings);};
    LapPlan interrupted;
    interrupted.update(5,40,INFINITY,0,3,settings);
    interrupted.update(40,40,INFINITY,2,3,settings);
    require(!interrupted.ready(),"stopped scout lap cannot enable a faster lap");
    plan.observe(0,4,.15f);
    require(!update(37,3) && plan.cap()==settings.scout_speed_cap,"scout speed cap");
    plan.enter(37,3,2,settings.scout_speed_cap,false);
    update(40,0);
    require(!plan.ready() && plan.sections()==0,"unfinished scout crossing the lap line");
    plan.returned(44);
    update(45,INFINITY);
    require(plan.ready() && plan.sections()==1 && plan.cap()==3,"promote successful traversal");
    require(!update(76.5,INFINITY) && !plan.avoidance_line() && plan.cap()<2,"brake without forcing a stale switch");
    require(!update(76.5,15),"unrelated obstacle cannot confirm switch");
    require(update(76.5,3.5) && plan.avoidance_line()->at(0)==.15f,"confirmed scheduled switch");
    plan.enter(76.5,3.5,2,settings.scout_speed_cap,true);
    require(plan.entries()==1 && plan.brake_steps()>0,"count executed switch and speed limits");
  }
  std::string dir="/tmp/el-native-tests";write_map(dir);
  {nlohmann::json points=nlohmann::json::array();
    for(int i=0;i<=40;++i){float a=2*pi*i/40;points.push_back({{"x_m",2*cosf(a)},{"y_m",2*sinf(a)},{"psi_rad",wrap(a+pi/2)},{"vx_mps",2},{"s_m",a*2}});}
    auto path=dir+"/circle.json";{std::ofstream f(path);f<<nlohmann::json{{"global_traj_wpnts_iqp",{{"wpnts",points}}}};}
    Track forward(path),reverse(path);orient_track(forward,false);orient_track(reverse,true);
    Track straight_track=forward;
    straight_track.points.clear();
    for(int i=0;i<70;++i)straight_track.points.push_back({i*.1f,0,0,3,i*.1f});
    AvoidCorridor corridor;
    corridor.follow_reference(straight_track,0);
    // Displaced farther from the racing line than the old 0.49 m chord.
    State merging{0,-.65f,0,2.7f};
    Observation guided{};
    corridor.condition(guided,merging,.5f,true);
    const float look=.5f*(.65f+.12f*2.7f)+.65f*(2.7f-1.8f);
    const float steering=atan2f(2*.3302f*.65f,look*look+.65f*.65f)/.4f;
    require(fabsf(guided.context[4]-steering)<1e-5f,"merge uses interpolated forward preview");
    require(forward.points.size()==40&&reverse.points.size()==40,"closed-loop endpoint must not duplicate");
    auto point=forward.points[10];State pose;pose.x=point.x;pose.y=point.y;
    auto projection=project_route(forward,pose);require(projection.distance<1e-5&&fabs(projection.s-point.s)<1e-5,"route projection on waypoint");
    require(fabs(route_delta(.1,39.9,40)-.2)<1e-6&&fabs(route_delta(39.9,.1,40)+.2)<1e-6,"signed progress across lap boundary");
    require(fabs(crossing_time(40,39.9,40.1,10,10.05)-10.025)<1e-6,"lap crossing interpolation");
    require(fabsf(wrap(reverse.points.front().yaw-forward.points.back().yaw-pi))<1e-5,"reverse tangent orientation");
    require(route_curvature(forward,10)>.49f&&route_curvature(reverse,10)<-.49f,"reverse curvature sign");
    require(road_region(route_curvature(forward,10))=="corner","curvature scenario classification");
    for(size_t i=1;i<reverse.points.size();++i)require(reverse.points[i].s>reverse.points[i-1].s,"reverse arc length must increase");
    for(auto name:{"compact","standard","wide","long","rotated"}){auto shape=obstacle_shape(name);require(shape.min_length>0&&shape.max_length>=shape.min_length&&shape.max_width>=shape.min_width,"valid obstacle bounds");}
    bool rejected=false;try{obstacle_shape("unknown");}catch(...){rejected=true;}require(rejected,"unknown obstacle profile rejected");}
  Memory memory(dir+"/map.yaml");Scan scan;scan.stamp=1;scan.ranges.fill(10);scan.valid.fill(0);
  {Memory angled(dir+"/map.yaml");Scan exact;exact.valid[beams/2]=1;exact.ranges[beams/2]=2;exact.explicit_angles=true;exact.angles[beams/2]=.3f;angled.update(exact);
    std::vector<uint8_t> cells;int w,h;float res,x0,y0;angled.copy_occupancy(cells,w,h,res,x0,y0);
    int ix=floorf((.27f+2*cosf(.3f)-x0)/res),iy=floorf((2*sinf(.3f)-y0)/res);
    require(cells[iy*w+ix]!=0,"downsampled rays must preserve their actual sensor angles");}
  require(!memory.wall_hides_disc(-3,0,3,0,.25),"open-space obstacle must not count as wall occluded");
  require(memory.base_free_disc(0,0,.3),"free candidate footprint");
  {std::vector<unsigned char> pixels(200*200,255);for(int y=0;y<200;++y)for(int x=99;x<=101;++x)pixels[y*200+x]=0;
    png_image img{};img.version=PNG_IMAGE_VERSION;img.width=img.height=200;img.format=PNG_FORMAT_GRAY;
    require(png_image_write_to_file(&img,(dir+"/map.png").c_str(),0,pixels.data(),0,nullptr),"occlusion fixture");}
  Memory wall_map(dir+"/map.yaml");require(wall_map.wall_hides_disc(-3,0,3,0,.25),"wall must occlude whole obstacle projection");
  {
    State approaching{};
    approaching.x=-.46f;
    approaching.v=1.4f;
    require(footprint(wall_map,approaching)<.5f,"physical rectangle still clear");
    require(gym_front_blocked(wall_map,approaching),"forward lidar TTC envelope reaches wall");
    approaching.yaw=pi;
    require(!gym_front_blocked(wall_map,approaching),"lidar TTC envelope follows heading");
  }
  require(!wall_map.base_free_disc(0,0,.3),"reject obstacle overlapping base wall");
  {Memory matched(dir+"/map.yaml");Scan ray;ray.pose.x=-1.27f;ray.explicit_angles=true;ray.valid[beams/2]=1;ray.angles[beams/2]=0;ray.range_max=18;
    // Sensor x=-1, mapped wall starts at -0.05: expected range 0.95 m.
    ray.ranges[beams/2]=.94f;
    for(int i=0;i<20;++i){ray.stamp=i*.05;matched.update(ray);}
    require(matched.occupied()==0&&matched.associated_wall_returns()==20,"noisy mapped wall must not create adjacent obstacle");
    State feature_pose;feature_pose.y=-.15f;Observation schema3;matched.features(feature_pose,1.,schema3);
    require(schema3.map[16*crop+5]==1&&schema3.map[crop*crop+16*crop+5]==1,"schema-3 policy features must retain their training-time wall-return encoding");
    require(!matched.base_free_disc(0,0,.02f),"wall remains blocked after association");
    // A protrusion in the SAME free cell, 4 cm before the wall, must remain visible.
    ray.ranges[beams/2]=.91f;for(int i=0;i<8;++i){ray.stamp++;matched.update(ray);}
    require(matched.occupied()>0,"near-wall real foreground return must not be suppressed by cell proximity");
    auto occupied=matched.occupied();ray.ranges[beams/2]=.94f;ray.stamp++;matched.update(ray);
    require(matched.occupied()==occupied,"ambiguous wall association cannot erase established obstacle");
    ray.valid.fill(0);ray.stamp++;matched.update(ray);require(matched.occupied()==occupied,"occlusion retains near-wall memory");
    ray.valid[beams/2]=1;ray.ranges[beams/2]=.95f;
    // A farther obstacle in free space must be registered immediately in a fresh map.
    Memory foreground(dir+"/map.yaml");ray.ranges[beams/2]=.5f;foreground.update(ray);require(foreground.occupied()==1,"new foreground obstacle remains immediate");
    ray.ranges[beams/2]=.95f;for(int i=0;i<8;++i){ray.stamp++;foreground.update(ray);}require(foreground.occupied()==0,"wall return still clears genuinely traversed foreground cells");
    Memory reverse_wall(dir+"/map.yaml");ray.pose.x=1.27f;ray.pose.yaw=pi;ray.ranges[beams/2]=.89f;ray.stamp++;
    reverse_wall.update(ray);require(reverse_wall.occupied()==0,"negative ray direction wall association");
    Memory parallel_wall(dir+"/map.yaml");ray.pose={};ray.pose.x=-.33f;ray.pose.y=-1;ray.angles[beams/2]=pi*.5f;ray.ranges[beams/2]=1;
    parallel_wall.update(ray);require(parallel_wall.occupied()==1,"obstacle beside a wall is not suppressed unless that same ray predicts a wall return");
    matched.reset();require(matched.occupied()==0&&matched.associated_wall_returns()==0,"reset clears evidence and diagnostics");
  }
  scan.ranges[beams/2]=2;scan.valid[beams/2]=1;memory.update(scan);require(memory.occupied()>0,"observed obstacle registered");
  size_t before=memory.occupied();scan.valid.fill(0);scan.stamp=2;memory.update(scan);require(memory.occupied()==before,"occlusion must retain obstacle");
  scan.valid[beams/2]=1;scan.ranges[beams/2]=10;
  for(int i=0;i<8;++i){scan.stamp++;memory.update(scan);}require(memory.occupied()==0,"free observations clear obstacle");
  scan.ranges[beams/2]=2;memory.update(scan);memory.reset();require(memory.occupied()==0,"reset clears episode memory");
  auto p=straight();p.state.v=1;require(safe_stop(p,{0,1}),"free braking trajectory");
  for(int y=0;y<grid_n;++y)for(int x=0;x<grid_n;++x){float wx=p.grid.x0+(x+.5f)*p.grid.resolution;if(wx>.5&&wx<.8)p.grid.cost[y*grid_n+x]=1;}
  p.state.v=3;require(!safe_stop(p,{0,2}),"insufficient braking distance rejected");
  require(!safe_stop(p,{NAN,1}),"NaN controls rejected");
  Router r;require(r.wants_avoid(true,0),"enter avoid immediately");require(r.wants_avoid(false,.1),"hold avoid");require(!r.wants_avoid(false,.7),"release after clear interval");
  require(r.update(true,false,false,.8)==Mode::Emergency,"emergency bypasses dwell time");
  require(!r.allow_candidate(true,.9)&&r.allow_candidate(true,1.3),"stable safe interval before returning from fallback");
  auto file=dir+"/episode.elds";std::filesystem::remove(file);
  {DatasetWriter w(file,"{}");Record rec;rec.step=7;rec.valid=1;rec.label={.1f,2};rec.observation.scans[3]=.3;w.append(rec);w.flush();}
  auto records=read_dataset(file);require(records.size()==1&&records[0].step==7&&records[0].observation.scans[3]==.3f,"dataset roundtrip");
  {std::ofstream out(file,std::ios::binary|std::ios::app);out.put('x');}bool rejected=false;try{read_dataset(file);}catch(...){rejected=true;}require(rejected,"truncated record rejected");
  MppiConfig cfg;cfg.samples=256;cfg.horizon=24;Mppi mppi(cfg);auto free=straight();free.state.v=1;
  auto plan=mppi.solve(free,{0,2});require(plan.valid&&std::isfinite(plan.action.speed),"GPU plan valid in free space");
  require(valid_plan(free,plan.sequence,cfg.dt),"weighted output independently validated");
  std::cout<<"GPU free-space solve "<<plan.compute_ms<<" ms\n";
  auto blocked=free;blocked.grid.cost.fill(1);auto failed=mppi.solve(blocked,{0,2});require(!failed.valid&&failed.action.speed==0,"blocked MPPI returns invalid stop");
  std::cout<<"PASS memory, braking, routing, dataset integrity, CUDA planning; no training\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}}
