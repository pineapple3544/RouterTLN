#pragma once
#include "el/core.hpp"
#include "el/control_timing.hpp"
#include <torch/torch.h>
namespace el {
// New network, initialized from scratch. Not the instructor's Keras checkpoint.
struct TlnImpl:torch::nn::Module {
  torch::nn::Sequential lidar,grid,head;
  bool guided=false;
  bool shared_guided=false;
  TlnImpl(){
    lidar=register_module("lidar",torch::nn::Sequential(
      torch::nn::Conv1d(torch::nn::Conv1dOptions(history*2,24,5).stride(2)),torch::nn::ReLU(),
      torch::nn::Conv1d(torch::nn::Conv1dOptions(24,32,5).stride(2)),torch::nn::ReLU(),
      torch::nn::Conv1d(torch::nn::Conv1dOptions(32,48,3).stride(2)),torch::nn::ReLU(),
      torch::nn::Conv1d(48,64,3),torch::nn::ReLU(),
      torch::nn::Conv1d(64,64,3),torch::nn::ReLU(),torch::nn::AdaptiveAvgPool1d(8)));
    grid=register_module("grid",torch::nn::Sequential(
      torch::nn::Conv2d(torch::nn::Conv2dOptions(channels,16,3).stride(2).padding(1)),torch::nn::ReLU(),
      torch::nn::Conv2d(torch::nn::Conv2dOptions(16,32,3).stride(2).padding(1)),torch::nn::ReLU(),
      torch::nn::AdaptiveAvgPool2d(torch::nn::AdaptiveAvgPool2dOptions({4,4}))));
    head=register_module("head",torch::nn::Sequential(torch::nn::Linear(1024+context_size,128),torch::nn::ReLU(),
      torch::nn::Linear(128,64),torch::nn::ReLU(),torch::nn::Linear(64,2),torch::nn::Tanh()));
    register_buffer("schema",torch::tensor({3,beams,history,crop,context_size},torch::kInt32));
  }
  torch::Tensor forward(torch::Tensor s,torch::Tensor m,torch::Tensor c){auto raw=head->forward(torch::cat({lidar->forward(s).flatten(1),grid->forward(m).flatten(1),c},1));
    if(!guided)return raw;
    auto steer=(c.select(1,4)+.20f*raw.select(1,0)).clamp(-1,1);
    if(shared_guided)steer=torch::where(c.select(1,3)>.5f,steer,raw.select(1,0));
    return torch::stack({steer,raw.select(1,1)},1);}
};
TORCH_MODULE(Tln);
inline std::array<torch::Tensor,3> tensors(const Observation& o,torch::Device device,
                                          ControlTiming* timing=nullptr){
  auto opts=torch::TensorOptions().dtype(torch::kFloat32);
  auto input=[&](const float* data,at::IntArrayRef shape){
    auto host=torch::from_blob(const_cast<float*>(data),shape,opts).clone();
    if(timing)timing->mark(ControlTiming::packing);
    auto placed=host.to(device);
    if(timing)timing->mark(ControlTiming::inputs);
    return placed;
  };
  return {input(o.scans.data(),{1,history*2,beams}),
          input(o.map.data(),{1,channels,crop,crop}),
          input(o.context.data(),{1,context_size})};
}
}
