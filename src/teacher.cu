#include "el/teacher.hpp"
#include <cuda_runtime.h>
#include <curand_kernel.h>
#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <numeric>
namespace el {
static void ck(cudaError_t e){if(e!=cudaSuccess)throw std::runtime_error(cudaGetErrorString(e));}
EL_HD static int near_ref(const TeacherProblem& p,const DynamicState& s,int last){
  int best=last;float distance=1e20f;
  for(int i=last>3?last-3:0;i<teacher_refs&&i<last+28;++i){float dx=s.z[0]-p.ref[i].x,dy=s.z[1]-p.ref[i].y;
    float d=dx*dx+dy*dy;if(d<distance){distance=d;best=i;}}
  return best;
}
EL_HD static float path_cost(const TeacherProblem& p,const Control* sequence,int h){
  DynamicState s=p.state;Control prev=p.previous;float cost=0;int n=0;
  for(int t=0;t<h;++t){auto u=sequence[t];float hit=0;
    for(int j=0;j<5;++j){s=physics_step(s,u,p.vehicle);hit=fmaxf(hit,teacher_footprint(p,s,.045f));}
    if(!isfinite(s.z[0])||!isfinite(s.z[6])||fabsf(s.z[6])>1)return 1e8f;
    if(hit>.5f)cost+=10000;
    n=near_ref(p,s,n);auto r=p.ref[n];float dx=s.z[0]-r.x,dy=s.z[1]-r.y,heading=wrap(s.z[4]-r.yaw),dv=s.z[3]-r.speed;
    cost+=.05f*(8*(dx*dx+dy*dy)+3*heading*heading+5*dv*dv+25*s.z[6]*s.z[6]);
    float ds=u.steer-prev.steer,vs=u.speed-prev.speed;
    cost+=80*ds*ds+.2f*vs*vs;prev=u;
  }
  cost-=8*p.ref[n].s;
  // A terminal braking continuation is checked in the same dynamic model.
  Control stop{s.z[2],0};
  for(int j=0;j<220&&s.z[3]>.05f;++j){s=physics_step(s,stop,p.vehicle);if(teacher_footprint(p,s,.04f)>.5f)cost+=1000;}
  return cost;
}
bool teacher_valid(const TeacherProblem& p,const std::vector<Control>& seq){
  DynamicState s=p.state;
  for(auto u:seq){if(!std::isfinite(u.steer)||!std::isfinite(u.speed)||fabs(u.steer)>.4001f||u.speed<0||u.speed>p.max_speed+.001f)return false;
    for(int j=0;j<5;++j){s=physics_step(s,u,p.vehicle);if(!std::isfinite(s.z[6])||fabs(s.z[6])>.5||teacher_footprint(p,s,.04f)>.5)return false;}}
  Control stop{s.z[2],0};for(int j=0;j<220&&s.z[3]>.05;++j){s=physics_step(s,stop,p.vehicle);if(teacher_footprint(p,s,.04f)>.5)return false;}
  return s.z[3]<.05;
}
float teacher_cost(const TeacherProblem& p,const std::vector<Control>& seq){return path_cost(p,seq.data(),seq.size());}
float fit_svg_sigma(const std::vector<float>& x,const std::vector<float>& logp,float fallback,float lo,float hi){
  if(x.size()!=logp.size()||x.size()<3)return fallback;
  double center=std::accumulate(x.begin(),x.end(),0.)/x.size(),scale=0;
  for(float v:x)scale=std::max(scale,std::abs(v-center));if(scale<1e-6)return fallback;
  double a[3][4]{},peak=*std::max_element(logp.begin(),logp.end());
  for(size_t i=0;i<x.size();++i){double z=(x[i]-center)/scale,b=std::max(-40.,double(logp[i])-peak),w=exp(2*b),powers[5]={1,z,z*z,z*z*z,z*z*z*z};
    for(int r=0;r<3;++r){for(int c=0;c<3;++c)a[r][c]+=w*powers[r+c];a[r][3]+=w*powers[r]*b;}}
  for(int i=0;i<3;++i){int pivot=i;for(int j=i+1;j<3;++j)if(fabs(a[j][i])>fabs(a[pivot][i]))pivot=j;
    if(fabs(a[pivot][i])<1e-10)return fallback;for(int c=i;c<4;++c)std::swap(a[pivot][c],a[i][c]);
    double d=a[i][i];for(int c=i;c<4;++c)a[i][c]/=d;
    for(int j=0;j<3;++j)if(j!=i){double v=a[j][i];for(int c=i;c<4;++c)a[j][c]-=v*a[i][c];}}
  if(a[2][3]>=-1e-8)return fallback;return clamp(scale*sqrt(-.5/a[2][3]),lo,hi);
}
// SMPPI samples derivative actions V, then evaluates A + dt*V (paper Algorithm 1).
// SVG guide phase estimates Eq.11; final phase applies Eq.8 using the transported prior.
__global__ void sample_rollout(const TeacherProblem* p,const Control* mean,const Control* prior,const Control* action,
    const Control* sigma,Control* samples,Control* variables,float* costs,TeacherConfig cfg,int count,int phase,uint64_t iteration){
  int k=blockIdx.x*blockDim.x+threadIdx.x;if(k>=count)return;
  curandStatePhilox4_32_10_t rng;curand_init(cfg.seed,k,iteration*cfg.horizon*4,&rng);
  float correction=0;
  for(int t=0;t<cfg.horizon;++t){float es=curand_normal(&rng)*sigma[t].steer,ev=curand_normal(&rng)*sigma[t].speed;
    Control v{mean[t].steer+es,mean[t].speed+ev};variables[k*cfg.horizon+t]=v;Control u=v;
    if(phase==0){u={action[t].steer+.05f*v.steer,action[t].speed+.05f*v.speed};
      correction+=cfg.temperature*(mean[t].steer*es/(sigma[t].steer*sigma[t].steer)+mean[t].speed*ev/(sigma[t].speed*sigma[t].speed));}
    else if(phase==2)correction+=cfg.temperature*((mean[t].steer-prior[t].steer)*v.steer/(sigma[t].steer*sigma[t].steer)
      +(mean[t].speed-prior[t].speed)*v.speed/(sigma[t].speed*sigma[t].speed));
    samples[k*cfg.horizon+t]={clamp(u.steer,-.4f,.4f),clamp(u.speed,0,p->max_speed)};
  }
  costs[k]=path_cost(*p,samples+k*cfg.horizon,cfg.horizon)+correction;
}
__global__ void reduce_update(const Control* samples,const float* costs,const Control* mean,const Control* sigma,Control* out,
    int h,int count,float beta,float temp,int guide,float step){
  int t=blockIdx.x*blockDim.x+threadIdx.x;if(t>=h)return;float den=0,st=0,sp=0;
  for(int k=0;k<count;++k){float w=expf(-(costs[k]-beta)/temp);den+=w;st+=w*samples[k*h+t].steer;sp+=w*samples[k*h+t].speed;}
  st/=fmaxf(den,1e-20f);sp/=fmaxf(den,1e-20f);
  // Covariance-preconditioned Eq.11 ascent. The previous inverse-variance step
  // amplified a small steering variance and repeatedly hit the step limiter.
  if(guide){st=mean[t].steer+.5f*(st-mean[t].steer);sp=mean[t].speed+.5f*(sp-mean[t].speed);}
  out[t]={st,sp};
}
__global__ void score_one(const TeacherProblem* p,const Control* u,float* cost,int h){if(threadIdx.x==0&&blockIdx.x==0)*cost=path_cost(*p,u,h);}
struct Teacher::Impl {
  TeacherConfig c;TeacherProblem* p;Control *mean,*prior,*action,*sigma,*samples,*variables,*out;float* costs;
  std::vector<Control> warm,deriv;std::vector<float> host_cost;uint64_t iter=0;bool fresh=true;
  explicit Impl(TeacherConfig cfg):c(cfg),warm(c.horizon),deriv(c.horizon),host_cost(std::max(c.samples,c.guide_samples)){
    if(c.samples<16||c.samples>32768||c.horizon<4||c.horizon>100||c.iterations<1||c.guide_iterations<3||c.guide_samples<16||c.temperature<=0||c.steer_sigma<=0||c.speed_sigma<=0)throw std::runtime_error("invalid teacher config");
    ck(cudaMalloc(&p,sizeof(TeacherProblem)));for(auto dst:{&mean,&prior,&action,&sigma,&out})ck(cudaMalloc(dst,c.horizon*sizeof(Control)));
    for(auto dst:{&samples,&variables})ck(cudaMalloc(dst,host_cost.size()*c.horizon*sizeof(Control)));ck(cudaMalloc(&costs,host_cost.size()*sizeof(float)));
  }
  ~Impl(){cudaFree(p);for(auto v:{mean,prior,action,sigma,samples,variables,out})cudaFree(v);cudaFree(costs);}
  void upload(Control* dst,const std::vector<Control>& v){ck(cudaMemcpy(dst,v.data(),v.size()*sizeof(Control),cudaMemcpyHostToDevice));}
  void evaluate(int phase,int count){sample_rollout<<<(count+127)/128,128>>>(p,mean,prior,action,sigma,samples,variables,costs,c,count,phase,iter++);
    ck(cudaGetLastError());ck(cudaMemcpy(host_cost.data(),costs,count*sizeof(float),cudaMemcpyDeviceToHost));}
  std::vector<Control> update(int count,bool guide,bool derivative=false){float beta=*std::min_element(host_cost.begin(),host_cost.begin()+count);
    reduce_update<<<(c.horizon+63)/64,64>>>(derivative?variables:samples,costs,mean,sigma,out,c.horizon,count,beta,c.temperature,guide,c.guide_step);
    std::vector<Control> v(c.horizon);ck(cudaMemcpy(v.data(),out,v.size()*sizeof(Control),cudaMemcpyDeviceToHost));return v;}
  float score(const std::vector<Control>& u){upload(out,u);score_one<<<1,1>>>(p,out,costs,c.horizon);float v;ck(cudaMemcpy(&v,costs,sizeof(v),cudaMemcpyDeviceToHost));return v;}
};
Teacher::Teacher(TeacherConfig c):impl_(new Impl(c)){}Teacher::~Teacher()=default;void Teacher::reset(){impl_->fresh=true;}
Plan Teacher::solve(const TeacherProblem& p,const std::vector<Control>& initial){
  auto begin=std::chrono::steady_clock::now();auto& a=*impl_;auto& c=a.c;
  if(initial.size()!=size_t(c.horizon))throw std::runtime_error("teacher initial horizon mismatch");
  ck(cudaMemcpy(a.p,&p,sizeof(p),cudaMemcpyHostToDevice));
  if(a.fresh){a.warm=initial;std::fill(a.deriv.begin(),a.deriv.end(),Control{});a.fresh=false;}
  else {std::rotate(a.warm.begin(),a.warm.begin()+1,a.warm.end());a.warm.back()=initial.back();
    std::rotate(a.deriv.begin(),a.deriv.begin()+1,a.deriv.end());a.deriv.back()={};
    for(auto& u:a.deriv){u.steer*=.5f;u.speed*=.5f;}}
  std::vector<Control> candidate=a.warm;
  Plan best;best.cost=1e30f;
  auto retain=[&](const std::vector<Control>& u,int source){float cost=a.score(u);
    if(cost<best.cost&&teacher_valid(p,u)){best.valid=true;best.cost=cost;best.sequence=u;best.source=source;}};
  retain(a.warm,1);retain(initial,2);
  // Reinitialize the proposal if the memory-aware route has a better dynamic rollout.
  if(a.score(initial)+1.f<a.score(candidate)){candidate=initial;a.warm=initial;std::fill(a.deriv.begin(),a.deriv.end(),Control{});}
  if(c.kind==TeacherKind::Smppi){
    std::vector<Control> sd(c.horizon,{c.steer_sigma/.05f,c.speed_sigma/.05f});a.upload(a.sigma,sd);
    for(int j=0;j<c.iterations;++j){a.upload(a.action,candidate);a.upload(a.mean,a.deriv);a.evaluate(0,c.samples);
      a.deriv=a.update(c.samples,false,true);for(int t=0;t<c.horizon;++t){auto old=candidate[t];candidate[t].steer=clamp(old.steer+.05f*a.deriv[t].steer,-.4,.4);candidate[t].speed=clamp(old.speed+.05f*a.deriv[t].speed,0,p.max_speed);
        a.deriv[t]={(candidate[t].steer-old.steer)/.05f,(candidate[t].speed-old.speed)/.05f};}
      retain(candidate,0);
      if(best.valid&&a.score(candidate)>best.cost){candidate=best.sequence;std::fill(a.deriv.begin(),a.deriv.end(),Control{});}}
  }else{
    std::vector<Control> guide=a.warm,sd(c.horizon,{c.steer_sigma,c.speed_sigma});std::vector<std::vector<Control>> history;std::vector<float> logp;
    a.upload(a.sigma,sd);
    for(int j=0;j<c.guide_iterations;++j){history.push_back(guide);logp.push_back(-a.score(guide)/c.temperature);
      a.upload(a.mean,guide);a.evaluate(1,c.guide_samples);guide=a.update(c.guide_samples,true,true);
      for(auto& u:guide){u.steer=clamp(u.steer,-.4,.4);u.speed=clamp(u.speed,0,p.max_speed);}}
    history.push_back(guide);logp.push_back(-a.score(guide)/c.temperature);
    auto best=std::max_element(logp.begin(),logp.end())-logp.begin();guide=history[best];
    retain(guide,3);
    for(int t=0;t<c.horizon;++t){std::vector<float> s,v;for(auto& h:history){s.push_back(h[t].steer);v.push_back(h[t].speed);}
      sd[t]={fit_svg_sigma(s,logp,c.steer_sigma,.035f,.20f),fit_svg_sigma(v,logp,c.speed_sigma,.20f,1.5f)};}
    a.upload(a.sigma,sd);a.upload(a.prior,guide);a.upload(a.mean,a.warm);a.evaluate(2,c.samples);candidate=a.update(c.samples,false);
  }
  retain(candidate,0);Plan result=best;
  // Keep a validated incumbent when the weighted update is worse; search samples
  // only when no feasible incumbent survived, so diagnostic source remains explicit.
  if(!result.valid){std::vector<int> order(c.samples);std::iota(order.begin(),order.end(),0);
    std::partial_sort(order.begin(),order.begin()+std::min(64,c.samples),order.end(),[&](int i,int j){return a.host_cost[i]<a.host_cost[j];});
    result.sequence.resize(c.horizon);
    for(int j=0;j<std::min(64,c.samples);++j){ck(cudaMemcpy(result.sequence.data(),a.samples+order[j]*c.horizon,c.horizon*sizeof(Control),cudaMemcpyDeviceToHost));
      if(teacher_valid(p,result.sequence)){result.valid=true;result.source=4;break;}}}
  double total=0,squares=0;float low=*std::min_element(a.host_cost.begin(),a.host_cost.begin()+c.samples);
  for(int k=0;k<c.samples;++k){double w=exp(-(a.host_cost[k]-low)/c.temperature);total+=w;squares+=w*w;}
  result.effective_samples=total*total/std::max(squares,1e-30);
  result.action=result.valid?result.sequence.front():Control{p.state.z[2],0};
  result.cost=a.score(result.sequence);if(result.valid)a.warm=result.sequence;else a.fresh=true;
  if(result.source!=0)std::fill(a.deriv.begin(),a.deriv.end(),Control{});
  result.compute_ms=std::chrono::duration<float,std::milli>(std::chrono::steady_clock::now()-begin).count();return result;
}
}
