#pragma once
#include "el/dynamics.hpp"
#include <algorithm>
#include <limits>
#include <optional>
#include <eigen3/Eigen/Cholesky>

namespace el {
// Scan-to-static-map localization. No pose, obstacle annotations, simulator
// state, initial waypoint or reset pose enters the measurement model.
class ScanLocalizer {
  friend struct OfflineSnapshot;
  struct Point {float x,y;};
  std::vector<float> field_;
  int width_=0,height_=0;
  float res_=0,x0_=0,y0_=0;
  State pose_{};
  double stamp_=-1;
  bool locked_=false;
  bool used_motion_prior_=false;

  float distance(float x,float y,float* gx=nullptr,float* gy=nullptr) const {
    const float fx=(x-x0_)/res_-.5f,fy=(y-y0_)/res_-.5f;
    if(!(fx>=0 && fy>=0 && fx<width_-1 && fy<height_-1))return 1.f;
    const int j=int(fx),i=int(fy),k=i*width_+j;
    const float u=fx-j,v=fy-i;
    const float a=field_[k],b=field_[k+1],c=field_[k+width_],d=field_[k+width_+1];
    if(gx)*gx=((1-v)*(b-a)+v*(d-c))/res_;
    if(gy)*gy=((1-u)*(c-a)+u*(d-b))/res_;
    return (1-v)*(a+u*(b-a))+v*(c+u*(d-c));
  }
  float loss(const State& s,const std::vector<Point>& points,int stride,float limit)const{
    const float c=cosf(s.yaw),h=sinf(s.yaw);
    float total=0;int n=0;
    for(size_t i=0;i<points.size();i+=stride){
      const auto& p=points[i];
      const float e=distance(s.x+c*p.x-h*p.y,s.y+h*p.x+c*p.y);
      total+=std::min(fabsf(e),limit);++n;
    }
    return total/std::max(1,n);
  }
  bool refine(State& s,const std::vector<Point>& points,float window,int iterations,
              const Point* translation_axis=nullptr)const{
    for(int it=0;it<iterations;++it){
      const float c=cosf(s.yaw),h=sinf(s.yaw);
      Eigen::Matrix3f H=Eigen::Matrix3f::Identity()*.025f;
      Eigen::Vector3f g=Eigen::Vector3f::Zero();int kept=0;
      for(auto p:points){
        const float dx=c*p.x-h*p.y,dy=h*p.x+c*p.y;
        float gx=0,gy=0;
        const float e=distance(s.x+dx,s.y+dy,&gx,&gy);
        if(fabsf(e)>=window)continue;
        Eigen::Vector3f J(gx,gy,gy*dx-gx*dy);
        if(translation_axis){
          J.x()=gx*translation_axis->x+gy*translation_axis->y;
          J.y()=0;
        }
        const float weight=std::min(1.f,.04f/std::max(.001f,fabsf(e)));
        H.noalias()+=weight*J*J.transpose();g+=weight*e*J;++kept;
      }
      if(kept<int(points.size()/3))return false;
      const Eigen::Vector3f step=H.ldlt().solve(-g);
      if(!step.allFinite())return false;
      const float shift=clamp(step.x(),-.12f,.12f);
      if(translation_axis){
        s.x+=shift*translation_axis->x;
        s.y+=shift*translation_axis->y;
      }else{
        s.x+=shift;
        s.y+=clamp(step.y(),-.12f,.12f);
      }
      s.yaw=wrap(s.yaw+clamp(step.z(),-.10f,.10f));
      if(step.squaredNorm()<1e-8f)break;
    }
    return true;
  }
  bool initialize(const Track& track,const std::vector<Point>& points,State& result)const{
    struct Hypothesis {State pose;float score;};
    std::vector<Hypothesis> candidates;
    for(size_t j=0;j<track.points.size();j+=4){
      const auto& w=track.points[j];
      for(float lateral:{-.8f,-.4f,0.f,.4f,.8f}){
        State s{w.x-sinf(w.yaw)*lateral,w.y+cosf(w.yaw)*lateral};
        if(distance(s.x,s.y)<.14f)continue;
        for(int k=0;k<24;++k){
          s.yaw=wrap(w.yaw+k*pi/12);
          candidates.push_back({s,loss(s,points,5,.35f)});
        }
      }
    }
    const size_t n=std::min<size_t>(64,candidates.size());
    std::partial_sort(candidates.begin(),candidates.begin()+n,candidates.end(),
                      [](const auto& a,const auto& b){return a.score<b.score;});
    std::vector<Hypothesis> fits;
    for(size_t i=0;i<n;++i){
      auto s=candidates[i].pose;
      if(refine(s,points,.35f,10))fits.push_back({s,loss(s,points,1,.12f)});
    }
    if(fits.empty())return false;
    auto best=std::min_element(fits.begin(),fits.end(),[](auto a,auto b){return a.score<b.score;});
    if(best->score>.055f)return false;
    for(auto h:fits)
      if(h.score<best->score+.006f && (hypotf(h.pose.x-best->pose.x,h.pose.y-best->pose.y)>.7f ||
         fabsf(wrap(h.pose.yaw-best->pose.yaw))>.4f))return false;
    result=best->pose;return true;
  }
 public:
  ScanLocalizer(const Memory& empty_map){
    std::vector<uint8_t> map;
    empty_map.copy_occupancy(map,width_,height_,res_,x0_,y0_);
    // Signed chamfer distance in metres, interpolated between cell centres.
    auto transform=[&](bool occupied){
      std::vector<float> d(map.size());
      for(size_t i=0;i<map.size();++i)d[i]=bool(map[i])==occupied?0.f:1e5f;
      for(int sweep:{1,-1}){
        for(int i=0;i<height_;++i)for(int j=0;j<width_;++j){
          const int y=sweep==1?i:height_-1-i,x=sweep==1?j:width_-1-j,k=y*width_+x;
          const int px=x-sweep,py=y-sweep;
          if(px>=0 && px<width_)d[k]=std::min(d[k],d[y*width_+px]+1);
          if(py<0 || py>=height_)continue;
          d[k]=std::min(d[k],d[py*width_+x]+1);
          for(int dx:{-1,1})if(x+dx>=0 && x+dx<width_)d[k]=std::min(d[k],d[py*width_+x+dx]+1.414214f);
        }
      }
      return d;
    };
    field_=transform(true);const auto free=transform(false);
    for(size_t i=0;i<map.size();++i)field_[i]=(map[i]?(.5f-free[i]):(field_[i]-.5f))*res_;
  }
  bool locked()const{return locked_;}
  void reset(){locked_=false;used_motion_prior_=false;stamp_=-1;pose_={};}
  std::optional<State> update(const Scan& scan,const Track& track,Control previous){
    std::vector<Point> points;points.reserve(beams);
    for(int i=0;i<beams;++i){
      const float r=scan.ranges[i];
      if(scan.valid[i]<.5f || !(r>.08f && r<scan.range_max-.03f))continue;
      const float a=scan.explicit_angles?scan.angles[i]:scan.angle_min+i*(scan.angle_max-scan.angle_min)/(beams-1);
      points.push_back({.27f+r*cosf(a),r*sinf(a)});
    }
    if(points.size()<30 || !std::isfinite(scan.stamp))return {};
    if(!locked_){
      if(!initialize(track,points,pose_))return {};
      locked_=true;stamp_=scan.stamp;return pose_;
    }
    const float dt=scan.stamp-stamp_;
    if(!(dt>0 && dt<=.15f))return {};
    const State prior=predict_command_tick(pose_,previous,dt);
    State next=prior;
    if(!refine(next,points,.18f,5))return {};
    const float fit_loss=loss(next,points,1,.12f);
    if(!(fit_loss<=.065f))return {};
    auto forward_speed=[&]{
      const float mid=pose_.yaw+.5f*wrap(next.yaw-pose_.yaw);
      return ((next.x-pose_.x)*cosf(mid)+(next.y-pose_.y)*sinf(mid))/dt;
    };
    float v=forward_speed();
    float rate=wrap(next.yaw-pose_.yaw)/dt;
    if(fabsf(rate)>14.f)return {};
    const bool inconsistent=fabsf(v-next.v)>3.f;
    if(inconsistent){
      // A scan can slide along a corridor wall with little change in residual.
      // A reverse jump or a jump ahead of both the prediction and the requested
      // speed can have an equally good longitudinal fit. Test the lateral and
      // heading measurement with forward travel fixed for this one scan.
      const bool longitudinal_jump=v<0.f || v>std::max(prior.v,previous.speed);
      if(used_motion_prior_ || !longitudinal_jump || prior.v<=0 || previous.speed<=0)return {};
      if(hypotf(next.x-prior.x,next.y-prior.y)>.18f)return {};
      const float mid=pose_.yaw+.5f*wrap(prior.yaw-pose_.yaw);
      const Point lateral{-sinf(mid),cosf(mid)};
      next=prior;
      if(!refine(next,points,.18f,5,&lateral))return {};
      const float constrained_loss=loss(next,points,1,.12f);
      if(!(constrained_loss<=std::min(.065f,fit_loss+.25f*res_)) ||
         hypotf(next.x-prior.x,next.y-prior.y)>res_)return {};
      v=forward_speed();
      rate=wrap(next.yaw-pose_.yaw)/dt;
      if(fabsf(v-next.v)>3.f || fabsf(rate)>14.f)return {};
    }
    used_motion_prior_=inconsistent;
    next.v=std::max(0.f,.82f*next.v+.18f*v);
    next.yaw_rate=.40f*pose_.yaw_rate+.60f*rate;
    next.steer=next.v>.2f?clamp(atanf(.3302f*next.yaw_rate/next.v),-.4f,.4f):previous.steer;
    pose_=next;stamp_=scan.stamp;return pose_;
  }
};
}
