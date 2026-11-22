#pragma once
#include "el/teacher.hpp"
#include "el/dataset.hpp"
#include "el/scenarios.hpp"
#include <Python.h>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>
#include <algorithm>
#include <chrono>
#include <set>
using namespace el;
using json=nlohmann::json;
struct Py {
  PyObject* p=nullptr;explicit Py(PyObject* v):p(v){if(!p){PyErr_Print();throw std::runtime_error("simulator Python API failed");}}
  ~Py(){Py_XDECREF(p);}Py(const Py&)=delete;Py& operator=(const Py&)=delete;
  Py(Py&& a):p(a.p){a.p=nullptr;}
};
Py attr(PyObject* p,const char* name){return Py(PyObject_GetAttrString(p,name));}
Py item(PyObject* p,int i){return Py(PySequence_GetItem(p,i));}
double number(PyObject* p){double v=PyFloat_AsDouble(p);if(PyErr_Occurred())throw std::runtime_error("simulator numeric conversion");return v;}
struct Sim {
  Py np,env,car,carclass;std::string module_path;Scan scan;DynamicState state;bool collision=false;double stamp=0;std::vector<double> raw_scan;json active=json::array();
  Py array(PyObject* list){return Py(PyObject_CallMethod(np.p,"asarray","O",list));}
  static Py make_env(const std::string& map,int seed){
    Py mod(PyImport_ImportModule("f110_gym.envs.f110_env")),cls=attr(mod.p,"F110Env"),args(PyTuple_New(0)),kw(PyDict_New());
    Py params(Py_BuildValue("{s:d,s:d,s:d,s:d,s:d,s:d,s:d,s:d,s:d,s:d,s:d,s:d,s:d,s:d,s:d,s:d,s:d,s:d}",
      "mu",1.0489,"C_Sf",4.718,"C_Sr",5.4562,"lf",.15875,"lr",.17145,"h",.074,"m",3.47,"I",.04712,
      "s_min",-.4189,"s_max",.4189,"sv_min",-3.2,"sv_max",3.2,"v_switch",7.319,"a_max",9.51,"v_min",-5.,"v_max",20.,"width",.2032,"length",.58));
    Py mp(PyUnicode_FromString(map.c_str())),ext(PyUnicode_FromString(".png")),sd(PyLong_FromLong(seed)),one(PyLong_FromLong(1));
    PyDict_SetItemString(kw.p,"map",mp.p);PyDict_SetItemString(kw.p,"map_ext",ext.p);PyDict_SetItemString(kw.p,"seed",sd.p);
    PyDict_SetItemString(kw.p,"num_agents",one.p);PyDict_SetItemString(kw.p,"params",params.p);
    return Py(PyObject_Call(cls.p,args.p,kw.p));
  }
  Sim(const std::string& map,int seed,float range):np(PyImport_ImportModule("numpy")),env(make_env(map,seed)),
    car(item(attr(attr(env.p,"sim").p,"agents").p,0)),carclass(attr(PyImport_AddModule("f110_gym.envs.base_classes"),"RaceCar")){
    Py mod(PyImport_ImportModule("f110_gym.envs.base_classes")),file=attr(mod.p,"__file__");module_path=PyUnicode_AsUTF8(file.p);
    scan.range_max=range;Py sensor=attr(carclass.p,"scan_simulator"),maximum(PyFloat_FromDouble(range));if(PyObject_SetAttrString(sensor.p,"max_range",maximum.p))throw std::runtime_error("LiDAR range configuration failed");
  }
  void obstacles(const json& obs){Py list(PyList_New(0));for(auto& o:obs){Py v(Py_BuildValue("(ddddd)",o[0].get<double>(),o[1].get<double>(),o[2].get<double>(),o[3].get<double>(),o[4].get<double>()));PyList_Append(list.p,v.p);}if(PyObject_SetAttrString(carclass.p,"static_obstacles",list.p))throw std::runtime_error("set obstacles");active=obs;}
  int candidate_visible_beams(const json& candidate){
    auto saved=active;json trial=active;trial.push_back(candidate);obstacles(trial);
    Py values(PyList_New(raw_scan.size()));for(size_t i=0;i<raw_scan.size();++i)PyList_SetItem(values.p,i,PyFloat_FromDouble(raw_scan[i]));
    Py a=array(values.p),result(PyObject_CallMethod(car.p,"ray_cast_static","O",a.p));int changed=0;
    for(size_t i=0;i<raw_scan.size();++i)if(number(item(result.p,i).p)<raw_scan[i]-1e-4)++changed;
    obstacles(saved);return changed;
  }
  void read(PyObject* result){auto obs=PyTuple_GetItem(result,0);auto scans=PyDict_GetItemString(obs,"scans");auto collisions=PyDict_GetItemString(obs,"collisions");
    collision=number(item(collisions,0).p)>.5;Py st=attr(car.p,"state"),delay=attr(car.p,"steer_buffer");
    for(int j=0;j<7;++j)state.z[j]=number(item(st.p,j).p);state.z[4]=wrap(state.z[4]);
    for(int j=0;j<2;++j)state.delay[j]=j<PySequence_Size(delay.p)?number(item(delay.p,j).p):0;
    Py ranges=item(scans,0);auto size=PySequence_Size(ranges.p);scan.pose=basic(state);scan.stamp=stamp;
    raw_scan.resize(size);for(int j=0;j<size;++j)raw_scan[j]=number(item(ranges.p,j).p);
    scan.explicit_angles=true;
    for(int j=0;j<beams;++j){int index=lround(double(j)*(size-1)/(beams-1));float v=raw_scan[index];scan.angles[j]=scan.angle_min+(scan.angle_max-scan.angle_min)*index/(size-1);scan.valid[j]=std::isfinite(v)&&v>0;scan.ranges[j]=scan.valid[j]?clamp(v,0,scan.range_max):scan.range_max;}
  }
  void reset(const Waypoint& w){stamp=0;Py list(Py_BuildValue("[[ddd]]",double(w.x),double(w.y),double(w.yaw))),a=array(list.p),r(PyObject_CallMethod(env.p,"reset","O",a.p));read(r.p);}
  void step(Control u){Py list(Py_BuildValue("[[dd]]",double(u.steer),double(u.speed))),a=array(list.p),r(PyObject_CallMethod(env.p,"step","O",a.p));stamp+=.01;read(r.p);}
};
void speed_profile(Track& track,float cap){
  int n=track.points.size();for(int i=0;i<n;++i){auto& w=track.points[i];auto a=track.points[(i+n-2)%n],b=track.points[(i+2)%n];
    float ds=hypotf(w.x-a.x,w.y-a.y)+hypotf(b.x-w.x,b.y-w.y),k=fabsf(wrap(b.yaw-a.y))/fmaxf(ds,.01f);
    w.speed=std::min({w.speed,cap,sqrtf(5.f/fmaxf(k,.001f))});}
  for(int j=0;j<8;++j){for(int i=n-1;i>=0;--i){auto& a=track.points[i];auto& b=track.points[(i+1)%n];a.speed=std::min(a.speed,sqrtf(b.speed*b.speed+10*hypotf(b.x-a.x,b.y-a.y)));}
    for(int i=0;i<n;++i){auto& a=track.points[i];auto& b=track.points[(i+1)%n];b.speed=std::min(b.speed,sqrtf(a.speed*a.speed+6*hypotf(b.x-a.x,b.y-a.y)));}}
}
TeacherProblem problem(const Track& track,const Memory& mem,const DynamicState& actual,float cap,Control previous){
  TeacherProblem p;auto st=basic(actual);p.state=actual;p.state.z[0]=p.state.z[1]=p.state.z[4]=0;p.max_speed=cap;p.previous=previous;
  p.grid=mem.local(st,.15f);int start=track.closest(st);float s=0,c=cosf(st.yaw),q=sinf(st.yaw);
  std::vector<uint8_t> cells;auto& collision=p.collision;mem.copy_occupancy(cells,collision.width,collision.height,collision.resolution,collision.x0,collision.y0);
  if(cells.size()>WorldCollisionGrid::capacity)throw std::runtime_error("map exceeds teacher collision-grid capacity");
  std::copy(cells.begin(),cells.end(),collision.cells.begin());collision.ego_x=st.x;collision.ego_y=st.y;collision.c=c;collision.s=q;p.stable_grid=true;
  for(int i=0;i<teacher_refs;++i){auto w=track.points[(start+i)%track.points.size()];float dx=w.x-st.x,dy=w.y-st.y;
    if(i)s+=hypotf(c*dx+q*dy-p.ref[i-1].x,-q*dx+c*dy-p.ref[i-1].y);
    p.ref[i]={c*dx+q*dy,-q*dx+c*dy,wrap(w.yaw-st.yaw),w.speed,s};}return p;
}
std::vector<Control> initial(const TeacherProblem& p,int h){std::vector<Control> out;auto s=p.state;int n=0;
  for(int t=0;t<h;++t){float best=1e9;for(int i=std::max(0,n-3);i<std::min(teacher_refs,n+25);++i){float d=hypotf(p.ref[i].x-s.z[0],p.ref[i].y-s.z[1]);if(d<best){best=d;n=i;}}
    int ahead=n;float look=.6f+.25f*s.z[3];while(ahead<teacher_refs-1&&p.ref[ahead].s-p.ref[n].s<look)++ahead;
    float dx=p.ref[ahead].x-s.z[0],dy=p.ref[ahead].y-s.z[1],lat=-sinf(s.z[4])*dx+cosf(s.z[4])*dy;
    Control u{clamp(atanf(2*.3302f*lat/fmaxf(dx*dx+dy*dy,.04f)),-.4,.4),p.ref[n].speed};out.push_back(u);for(int j=0;j<5;++j)s=physics_step(s,u,p.vehicle);
  }return out;}

