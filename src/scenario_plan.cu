#include "el/scenario_plan.hpp"
#include <cuda_runtime.h>
#include <algorithm>
#include <stdexcept>
namespace el::scenario {
namespace {
void check(cudaError_t x){if(x!=cudaSuccess)throw std::runtime_error(cudaGetErrorString(x));}
struct Batch {Frame frame;MapView map;Hypothesis h[batch_size];int count=0;};
__global__ void score(const Batch* b,float* costs){
  int k=blockIdx.x*blockDim.x+threadIdx.x;if(k>=b->count*candidates)return;
  Path p;costs[k]=evaluate(b->frame,b->map,b->h[k/candidates],k%candidates,p);
}
__global__ void materialize(const Batch* b,const int* best,Path* paths){
  int k=threadIdx.x;if(k>=b->count)return;
  if(best[k]<0){paths[k]=Path{};return;}
  evaluate(b->frame,b->map,b->h[k],best[k],paths[k]);
}
}
struct Optimizer::Impl{
  cudaStream_t stream=nullptr;uint8_t* cells=nullptr;size_t capacity=0;MapView map{};
  Batch* batch=nullptr;float* costs=nullptr;int* best=nullptr;Path* paths=nullptr;
  std::vector<float> host_costs=std::vector<float>(batch_size*candidates);
  void release(){if(stream)cudaStreamSynchronize(stream);cudaFree(cells);cudaFree(batch);cudaFree(costs);cudaFree(best);cudaFree(paths);if(stream)cudaStreamDestroy(stream);}
  Impl(){try{check(cudaStreamCreateWithFlags(&stream,cudaStreamNonBlocking));check(cudaMalloc(&batch,sizeof(Batch)));
    check(cudaMalloc(&costs,batch_size*candidates*sizeof(float)));check(cudaMalloc(&best,batch_size*sizeof(int)));
    check(cudaMalloc(&paths,batch_size*sizeof(Path)));}catch(...){release();throw;}}
  ~Impl(){release();}
};
Optimizer::Optimizer():impl_(new Impl){} Optimizer::~Optimizer()=default;
void Optimizer::upload(MapView m){
  auto& a=*impl_;size_t n=size_t(m.width)*m.height;
  if(!m.data||m.width<=0||m.height<=0||!(m.resolution>0)||n>16000000)throw std::invalid_argument("invalid scenario map");
  if(n>a.capacity){check(cudaFree(a.cells));a.cells=nullptr;a.capacity=0;check(cudaMalloc(&a.cells,n));a.capacity=n;}
  check(cudaMemcpyAsync(a.cells,m.data,n,cudaMemcpyHostToDevice,a.stream));check(cudaStreamSynchronize(a.stream));
  a.map=m;a.map.data=a.cells;
}
std::vector<Path> Optimizer::solve(const Frame& f,const std::vector<Hypothesis>& h){
  auto& a=*impl_;if(h.empty()||h.size()>batch_size||!a.map.data)throw std::invalid_argument("invalid scenario batch");
  Batch b{};b.frame=f;b.map=a.map;b.count=int(h.size());
  for(int i=0;i<b.count;++i){if(h[i].count<0||h[i].count>2)throw std::invalid_argument("invalid hypothesis");b.h[i]=h[i];}
  check(cudaMemcpyAsync(a.batch,&b,sizeof(b),cudaMemcpyHostToDevice,a.stream));
  score<<<(b.count*candidates+127)/128,128,0,a.stream>>>(a.batch,a.costs);
  check(cudaGetLastError());check(cudaMemcpyAsync(a.host_costs.data(),a.costs,b.count*candidates*sizeof(float),cudaMemcpyDeviceToHost,a.stream));check(cudaStreamSynchronize(a.stream));
  int best[batch_size];for(int i=0;i<b.count;++i){
    auto begin=a.host_costs.begin()+i*candidates;auto end=begin+candidates;auto it=std::min_element(begin,end);
    best[i]=*it<invalid_cost?int(it-begin):-1;
  }
  check(cudaMemcpyAsync(a.best,best,b.count*sizeof(int),cudaMemcpyHostToDevice,a.stream));
  materialize<<<1,32,0,a.stream>>>(a.batch,a.best,a.paths);check(cudaGetLastError());
  std::vector<Path> out(b.count);check(cudaMemcpyAsync(out.data(),a.paths,b.count*sizeof(Path),cudaMemcpyDeviceToHost,a.stream));check(cudaStreamSynchronize(a.stream));return out;
}
}
