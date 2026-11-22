#pragma once
// Dynamics port attribution and BSD license: ../../THIRD_PARTY_DYNAMICS.txt.
#include "el/core.hpp"
namespace el {
// F1TENTH single-track model shared by the teacher and the router safety check.
struct Vehicle {float mu=1.0489f,cf=4.718f,cr=5.4562f,lf=.15875f,lr=.17145f,h=.074f,m=3.47f,inertia=.04712f;};
struct DynamicState {float z[7]{};float delay[2]{};}; // x,y,steer,v,yaw,yawrate,beta; newest steering first
EL_HD inline void derivative(const DynamicState& s,float sv,float a,const Vehicle& p,float* f){
  const float* x=s.z;float v=x[3],l=p.lf+p.lr;
  if((x[2]<=-.4189f&&sv<0)||(x[2]>=.4189f&&sv>0))sv=0;sv=clamp(sv,-3.2f,3.2f);
  float limit=v>7.319f?9.51f*7.319f/v:9.51f;
  if((v<=-5&&a<0)||(v>=20&&a>0))a=0;else a=clamp(a,-9.51f,limit);
  f[2]=sv;f[3]=a;
  if(fabsf(v)<.5f){f[0]=v*cosf(x[4]);f[1]=v*sinf(x[4]);f[4]=v/l*tanf(x[2]);
    f[5]=a/l*tanf(x[2])+v/(l*cosf(x[2])*cosf(x[2]))*sv;f[6]=0;return;}
  float front=9.81f*p.lr-a*p.h,rear=9.81f*p.lf+a*p.h;
  f[0]=v*cosf(x[6]+x[4]);f[1]=v*sinf(x[6]+x[4]);f[4]=x[5];
  f[5]=-p.mu*p.m/(v*p.inertia*l)*(p.lf*p.lf*p.cf*front+p.lr*p.lr*p.cr*rear)*x[5]
    +p.mu*p.m/(p.inertia*l)*(p.lr*p.cr*rear-p.lf*p.cf*front)*x[6]+p.mu*p.m/(p.inertia*l)*p.lf*p.cf*front*x[2];
  f[6]=(p.mu/(v*v*l)*(p.cr*rear*p.lr-p.cf*front*p.lf)-1)*x[5]
    -p.mu/(v*l)*(p.cr*rear+p.cf*front)*x[6]+p.mu/(v*l)*p.cf*front*x[2];
}
EL_HD inline float drive_acceleration(float speed,float target){
  const float error=target-speed;
  const float gain=speed>0 ? (error>0?4.755f:19.02f) : (error>0?.951f:3.804f);
  return gain*error;
}
EL_HD inline DynamicState physics_step(DynamicState s,Control u,const Vehicle& p){
  float steer=s.delay[1];s.delay[1]=s.delay[0];s.delay[0]=u.steer;
  float error=steer-s.z[2],sv=fabsf(error)>1e-4f?copysignf(3.2f,error):0;
  float a=drive_acceleration(s.z[3],u.speed),k[4][7];DynamicState tmp=s;derivative(s,sv,a,p,k[0]);
  for(int j=0;j<7;++j)tmp.z[j]=s.z[j]+.005f*k[0][j];derivative(tmp,sv,a,p,k[1]);
  for(int j=0;j<7;++j)tmp.z[j]=s.z[j]+.005f*k[1][j];derivative(tmp,sv,a,p,k[2]);
  for(int j=0;j<7;++j)tmp.z[j]=s.z[j]+.01f*k[2][j];derivative(tmp,sv,a,p,k[3]);
  for(int j=0;j<7;++j)s.z[j]+=.01f/6*(k[0][j]+2*k[1][j]+2*k[2][j]+k[3][j]);
  s.z[4]=wrap(s.z[4]);return s;
}
EL_HD inline State basic(const DynamicState& s){return {s.z[0],s.z[1],s.z[4],s.z[3],s.z[2],s.z[5]};}
}
