#include "el/mppi.hpp"
#include <cuda_runtime.h>
#include <curand_kernel.h>
#include <stdexcept>
#include <algorithm>
#include <chrono>
#include <limits>
namespace el {
static void check(cudaError_t e){if(e!=cudaSuccess)throw std::runtime_error(cudaGetErrorString(e));}
__global__ void rollout(const Problem* p,const Control* nominal,Control* samples,float* costs,MppiConfig cfg,uint64_t iteration){
  int k=blockIdx.x*blockDim.x+threadIdx.x;if(k>=cfg.samples)return;
  curandStatePhilox4_32_10_t rng;curand_init(cfg.seed,k,iteration*cfg.horizon*4,&rng);
  State s=p->state;float total=0,ns=0,nv=0;Control prev{s.steer,s.v};
  for(int t=0;t<cfg.horizon;++t){
    ns=.65f*ns+.759934f*curand_normal(&rng)*cfg.steer_sigma;
    nv=.65f*nv+.759934f*curand_normal(&rng)*cfg.speed_sigma;
    if(k==0){ns=nv=0;}
    Control u{clamp(nominal[t].steer+ns,-.4f,.4f),clamp(nominal[t].speed+nv,0,p->max_speed)};
    // Explicit low-speed candidates help the optimizer retain a braking option.
    if(k==1)u={nominal[t].steer,0};
    samples[k*cfg.horizon+t]=u;
    float hit=0;
    for(int j=0;j<4;++j){s=advance(s,u,cfg.dt/4);hit=fmaxf(hit,footprint(p->grid,s,.04f+.04f*fminf(1.f,t*cfg.dt/.3f)));}
    if(hit>.5f)total+=100000.f;
    int near=nearest(*p,s);Ref r=p->ref[near];float dx=s.x-r.x,dy=s.y-r.y;
    float heading=wrap(s.yaw-r.yaw),ds=u.steer-prev.steer,dv=u.speed-prev.speed;
    total+=2.5f*(dx*dx+dy*dy)+.6f*heading*heading+.25f*(s.v-r.speed)*(s.v-r.speed)+3.f*ds*ds+.03f*dv*dv;
    // Nominal/noise correction from the path-integral control distribution.
    total+=cfg.temperature*.02f*(nominal[t].steer*ns/(cfg.steer_sigma*cfg.steer_sigma)+nominal[t].speed*nv/(cfg.speed_sigma*cfg.speed_sigma));
    prev=u;
  }
  total-=5.f*p->ref[nearest(*p,s)].s;
  Control brake{s.steer,0};int brake_steps=(int)ceilf(s.v/6/.02f)+30;
  for(int t=0;t<brake_steps;++t){s=advance(s,brake,.02f);if(footprint(p->grid,s)>.5f)total+=100000.f;}
  costs[k]=isfinite(total)?total:1e30f;
}
__global__ void weighted(const Control* samples,const float* costs,Control* out,MppiConfig cfg,float beta){
  int t=blockIdx.x*blockDim.x+threadIdx.x;if(t>=cfg.horizon)return;
  float sum=0,s=0,v=0;
  for(int k=0;k<cfg.samples;++k){float w=expf(-(costs[k]-beta)/cfg.temperature);auto u=samples[k*cfg.horizon+t];sum+=w;s+=w*u.steer;v+=w*u.speed;}
  out[t]={s/fmaxf(sum,1e-12f),v/fmaxf(sum,1e-12f)};
}
struct Mppi::Impl {
  MppiConfig cfg;Problem* problem=nullptr;Control *nominal=nullptr,*samples=nullptr,*output=nullptr;float* costs=nullptr;
  std::vector<Control> warm;std::vector<float> host_costs;uint64_t iteration=0;bool fresh=true;
  cudaStream_t stream=nullptr;
  explicit Impl(MppiConfig c):cfg(c),warm(c.horizon),host_costs(c.samples){
    if(c.samples<2||c.samples>32768||c.horizon<2||c.horizon>256||c.dt<=0||c.temperature<=0||c.steer_sigma<=0||c.speed_sigma<=0)throw std::invalid_argument("invalid MPPI configuration");
    check(cudaStreamCreateWithFlags(&stream,cudaStreamNonBlocking));
    check(cudaMalloc(&problem,sizeof(Problem)));check(cudaMalloc(&nominal,c.horizon*sizeof(Control)));
    check(cudaMalloc(&samples,c.samples*c.horizon*sizeof(Control)));check(cudaMalloc(&output,c.horizon*sizeof(Control)));check(cudaMalloc(&costs,c.samples*sizeof(float)));
  }
  ~Impl(){cudaFree(problem);cudaFree(nominal);cudaFree(samples);cudaFree(output);cudaFree(costs);if(stream)cudaStreamDestroy(stream);}
};
Mppi::Mppi(MppiConfig c):impl_(new Impl(c)){} Mppi::~Mppi()=default;
void Mppi::reset(){impl_->fresh=true;impl_->iteration=0;}
Plan Mppi::solve(const Problem& p,Control initial){
  auto begin=std::chrono::steady_clock::now();auto& a=*impl_;auto c=a.cfg;
  if(a.fresh){State s=p.state;for(int t=0;t<c.horizon;++t){
      int n=nearest(p,s);int target=std::min(ref_n-1,n+5);float dx=p.ref[target].x-s.x,dy=p.ref[target].y-s.y;
      float lateral=-sinf(s.yaw)*dx+cosf(s.yaw)*dy;
      auto u=Control{clamp(atan2f(2*.3302f*lateral,dx*dx+dy*dy),-.4f,.4f),fminf(initial.speed,p.ref[n].speed)};
      a.warm[t]=u;s=advance(s,u,c.dt);}
    a.fresh=false;
  }else{for(int t=0;t<c.horizon-1;++t){a.warm[t].steer=.5f*(a.warm[t].steer+a.warm[t+1].steer);a.warm[t].speed=.5f*(a.warm[t].speed+a.warm[t+1].speed);}}
  check(cudaMemcpyAsync(a.problem,&p,sizeof(p),cudaMemcpyHostToDevice,a.stream));
  check(cudaMemcpyAsync(a.nominal,a.warm.data(),c.horizon*sizeof(Control),cudaMemcpyHostToDevice,a.stream));
  rollout<<<(c.samples+127)/128,128,0,a.stream>>>(a.problem,a.nominal,a.samples,a.costs,c,a.iteration++);
  check(cudaGetLastError());check(cudaMemcpyAsync(a.host_costs.data(),a.costs,c.samples*sizeof(float),cudaMemcpyDeviceToHost,a.stream));check(cudaStreamSynchronize(a.stream));
  auto best=std::min_element(a.host_costs.begin(),a.host_costs.end());float beta=*best;
  weighted<<<(c.horizon+63)/64,64,0,a.stream>>>(a.samples,a.costs,a.output,c,beta);check(cudaGetLastError());
  check(cudaMemcpyAsync(a.warm.data(),a.output,c.horizon*sizeof(Control),cudaMemcpyDeviceToHost,a.stream));check(cudaStreamSynchronize(a.stream));
  Plan result;result.cost=beta;result.sequence=a.warm;result.valid=valid_plan(p,result.sequence,c.dt);
  // Weighted controls may cross an obstacle between two safe modes. Never trust that average blindly.
  if(!result.valid){
    std::vector<int> order(c.samples);for(int i=0;i<c.samples;++i)order[i]=i;
    std::partial_sort(order.begin(),order.begin()+std::min(16,c.samples),order.end(),[&](int i,int j){return a.host_costs[i]<a.host_costs[j];});
    for(int n=0;n<std::min(16,c.samples);++n){int i=order[n];check(cudaMemcpy(result.sequence.data(),a.samples+i*c.horizon,c.horizon*sizeof(Control),cudaMemcpyDeviceToHost));
      result.valid=valid_plan(p,result.sequence,c.dt)&&safe_stop(p,result.sequence.front());if(result.valid)break;}
  }
  result.action=result.valid?result.sequence.front():Control{p.state.steer,0};
  result.valid=result.valid&&safe_stop(p,result.action);
  if(result.valid)a.warm=result.sequence;else a.fresh=true;
  result.compute_ms=std::chrono::duration<float,std::milli>(std::chrono::steady_clock::now()-begin).count();return result;
}
}
