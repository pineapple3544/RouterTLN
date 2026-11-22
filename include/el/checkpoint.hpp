#pragma once
#include "el/network.hpp"
#include <filesystem>
#include <limits>
namespace el {
// Snapshot weights onto CPU without moving the live model or its optimizer state.
class BestCheckpoint {
 public:
  explicit BestCheckpoint(std::string path):path_(std::move(path)){
    if(std::filesystem::exists(path_)||std::filesystem::exists(path_+".pending"))throw std::runtime_error("refuse to overwrite checkpoint from another run");
    auto parent=std::filesystem::path(path_).parent_path();if(!parent.empty())std::filesystem::create_directories(parent);
  }
  bool consider(Tln& model,double loss,int epoch){
    if(!std::isfinite(loss))throw std::runtime_error("nonfinite validation loss; checkpoint unchanged");
    if(loss>=best_loss_)return false;
    torch::NoGradGuard guard;
    auto dst=snapshot_->named_parameters();for(auto& p:model->named_parameters())dst[p.key()].copy_(p.value().to(torch::kCPU));
    auto buffers=snapshot_->named_buffers();for(auto& p:model->named_buffers())buffers[p.key()].copy_(p.value().to(torch::kCPU));
    snapshot_->eval();torch::save(snapshot_,path_+".pending");
    std::filesystem::rename(path_+".pending",path_);
    best_loss_=loss;best_epoch_=epoch;return true;
  }
  double loss()const{return best_loss_;}int epoch()const{return best_epoch_;}
 private:
  std::string path_;Tln snapshot_;double best_loss_=std::numeric_limits<double>::infinity();int best_epoch_=0;
};
}
