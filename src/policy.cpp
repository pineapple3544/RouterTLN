#include "el/policy.hpp"
#include "el/network.hpp"
#include <filesystem>
namespace el {
struct Policy::Impl {Tln model;torch::Device device{torch::kCUDA};};
Policy::Policy(const std::string& checkpoint,InferenceDevice device):impl_(new Impl){
  if(checkpoint.empty()||!std::filesystem::is_regular_file(checkpoint))throw std::runtime_error("trained TLN checkpoint required: "+checkpoint);
  if(device==InferenceDevice::cpu){
    impl_->device=torch::kCPU;
    torch::set_num_threads(1);
  }else if(!torch::cuda::is_available())throw std::runtime_error("CUDA LibTorch unavailable");
  torch::load(impl_->model,checkpoint,torch::Device(torch::kCPU));auto schema=impl_->model->named_buffers()["schema"].to(torch::kCPU);
  auto expected=torch::tensor({3,beams,history,crop,context_size},torch::kInt32);
  auto guided=expected.clone();guided[0]=4;
  impl_->model->guided=torch::equal(schema,guided);
  if(!impl_->model->guided&&!torch::equal(schema,expected))throw std::runtime_error("checkpoint schema mismatch");
  impl_->model->to(impl_->device);impl_->model->eval();
  Observation warm;for(int i=0;i<3;++i)infer(warm);
}
bool Policy::uses_corridor()const{return impl_->model->guided;}
Policy::~Policy()=default;
Control Policy::infer(const Observation& o,ControlTiming* timing){
  torch::NoGradGuard guard;
  auto t=tensors(o,impl_->device,timing);
  if(timing)timing->mark(ControlTiming::inputs);
  auto out=impl_->model->forward(t[0],t[1],t[2]);
  if(timing)timing->mark(ControlTiming::forward);
  out=out.to(torch::kCPU).contiguous();
  if(timing)timing->mark(ControlTiming::readback);
  auto p=out.data_ptr<float>();return {p[0]*.4f,(p[1]+1)*5.f};
}
}
