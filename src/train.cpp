#include "el/network.hpp"
#include "el/checkpoint.hpp"
#include "el/dataset.hpp"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <iostream>
#include <random>
#include <set>
#include <numeric>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <nlohmann/json.hpp>
using namespace el;using json=nlohmann::json;
struct Batch {torch::Tensor scan,map,context,label;};
Batch batch(const std::vector<Record>& records,const std::vector<size_t>& indices,size_t begin,size_t end,torch::Device dev){
  int n=end-begin;Batch b{torch::empty({n,history*2,beams}),torch::empty({n,channels,crop,crop}),torch::empty({n,context_size}),torch::empty({n,2})};
  for(int j=0;j<n;++j){const auto& r=records[indices[begin+j]];
    std::copy(r.observation.scans.begin(),r.observation.scans.end(),b.scan[j].data_ptr<float>());
    std::copy(r.observation.map.begin(),r.observation.map.end(),b.map[j].data_ptr<float>());
    std::copy(r.observation.context.begin(),r.observation.context.end(),b.context[j].data_ptr<float>());
    b.label[j][0]=r.label.steer/.4f;b.label[j][1]=r.label.speed/5.f-1;
  }
  return {b.scan.to(dev),b.map.to(dev),b.context.to(dev),b.label.to(dev)};
}
struct Metrics {double loss=0,steer_mae=0,speed_mae=0;};
Metrics evaluate(Tln& model,const std::vector<Record>& records,const std::vector<size_t>& order,int bs,torch::Device dev){
  model->eval();torch::NoGradGuard guard;Metrics m;
  for(size_t i=0;i<order.size();i+=bs){size_t end=std::min(i+bs,order.size());auto b=batch(records,order,i,end,dev);auto pred=model->forward(b.scan,b.map,b.context);
    m.loss+=torch::smooth_l1_loss(pred,b.label).item<double>()*(end-i);auto error=(pred-b.label).abs();
    m.steer_mae+=error.select(1,0).sum().item<double>()*.4;m.speed_mae+=error.select(1,1).sum().item<double>()*5.;}
  m.loss/=records.size();m.steer_mae/=records.size();m.speed_mae/=records.size();
  if(!std::isfinite(m.loss)||!std::isfinite(m.steer_mae)||!std::isfinite(m.speed_mae))throw std::runtime_error("nonfinite evaluation");return m;
}
json metrics(const Metrics& m){return {{"loss",m.loss},{"steer_mae_rad",m.steer_mae},{"speed_mae_mps",m.speed_mae}};}
void write_json(const std::string& path,const json& value){
  {std::ofstream out(path+".pending");out<<value.dump(2);out.flush();if(!out)throw std::runtime_error("cannot write training metrics");}
  std::filesystem::rename(path+".pending",path);
}
int main(int argc,char** argv){try{
  torch::set_num_threads(2);torch::manual_seed(2026);
  if(!torch::cuda::is_available())throw std::runtime_error("CUDA LibTorch is required");torch::Device dev(torch::kCUDA);
  if(argc==1||(argc==2&&std::string(argv[1])=="--check")){
    torch::NoGradGuard guard;Tln model;model->to(dev);model->eval();Observation o;auto t=tensors(o,dev);auto output=model->forward(t[0],t[1],t[2]).to(torch::kCPU);
    if(!torch::isfinite(output).all().item<bool>())throw std::runtime_error("nonfinite forward pass");int64_t count=0;for(auto& p:model->parameters())count+=p.numel();
    std::cout<<"CUDA forward check OK; parameters="<<count<<"; no backward, optimizer step, or checkpoint written\n";return 0;
  }
  if(argc!=3||std::string(argv[1])!="--train")throw std::runtime_error("Usage: el_train --check | --train CONFIG.yaml (explicit opt-in)");
  auto config=YAML::LoadFile(argv[2]);std::string expert=config["expert"].as<std::string>();if(expert!="race"&&expert!="avoid")throw std::runtime_error("expert must be race or avoid");
  auto output=config["checkpoint"].as<std::string>();
  for(auto suffix:{"",".pending",".training.json",".training.json.pending"})if(std::filesystem::exists(output+suffix))throw std::runtime_error("refuse to overwrite existing run");
  int seed=config["seed"]?config["seed"].as<int>():2026;torch::manual_seed(seed);
  int epochs=config["epochs"].as<int>(),bs=config["batch_size"].as<int>();double lr=config["learning_rate"].as<double>();
  if(epochs<1||bs<1||!std::isfinite(lr)||lr<=0)throw std::runtime_error("invalid training parameters");
  bool guided=config["corridor_guided"] && config["corridor_guided"].as<bool>();
  if(guided && expert!="avoid")throw std::runtime_error("guided checkpoint requires Avoid data");
  int mode=expert=="race"?0:1;std::vector<Record> train,val;std::set<std::string> files,groups;json paths;
  // Test paths are checked for overlap, but test observations/labels are never loaded.
  for(auto split:{"train","validation","test"}){
    auto key=std::string(split)+"_episodes";paths[key]=json::array();
    for(auto entry:config[key]){auto path=std::filesystem::canonical(entry.as<std::string>()).string();if(!files.insert(path).second)throw std::runtime_error("duplicate episode or split leakage");
      std::ifstream mf(path+".json");json meta;mf>>meta;
      if(meta.at("status")!="accepted"||meta.at("scenario").at("mode")!=expert||meta.at("scenario").at("split")!=split)throw std::runtime_error("dataset mode/status/split mismatch");
      if(fabs(meta.value("lidar_range_m",0.f)-18.f)>.001f)throw std::runtime_error("schema 3 requires 18 m LiDAR");
      if(meta.value("corridor_guided",false)!=guided)throw std::runtime_error("guide feature mismatch");
      auto group=meta.value("source_attempt",meta.at("name").get<std::string>());
      for(auto other:{"train","validation","test"})if(std::string(other)!=split&&groups.count(std::string(other)+":"+group))throw std::runtime_error("source attempt split leakage");groups.insert(std::string(split)+":"+group);
      paths[key].push_back(path);if(std::string(split)=="test")continue;
      auto& destination=std::string(split)=="train"?train:val;auto rows=read_dataset(path);
      if(rows.size()!=meta.at("records").get<size_t>())throw std::runtime_error("dataset record count mismatch");
      for(auto& r:rows){if(!r.valid||r.collision||r.mode!=mode)throw std::runtime_error("invalid record in training manifest");destination.push_back(std::move(r));}
    }
  }
  if(train.empty()||val.empty())throw std::runtime_error("train/validation examples required");
  Tln model;
  if(config["initial_checkpoint"])torch::load(model,config["initial_checkpoint"].as<std::string>());
  model->guided=guided;model->named_buffers()["schema"][0]=guided?4:3;
  model->to(dev);torch::optim::Adam optim(model->parameters(),torch::optim::AdamOptions(lr));BestCheckpoint best(output);
  std::vector<size_t> order(train.size()),vo(val.size());std::iota(order.begin(),order.end(),0);std::iota(vo.begin(),vo.end(),0);std::mt19937 rng(seed);
  auto baseline=evaluate(model,val,vo,bs,dev);
  json report={{"status","running"},{"expert",expert},{"seed",seed},{"epochs",epochs},{"batch_size",bs},{"learning_rate",lr},{"corridor_guided",guided},{"initial_checkpoint",config["initial_checkpoint"]?config["initial_checkpoint"].as<std::string>():""},{"optimizer","Adam"},{"loss","SmoothL1 mean over normalized steering/speed"},{"selection","strictly lowest validation loss; ties keep first"},{"train_records",train.size()},{"validation_records",val.size()},{"test_evaluated",false},{"paths",paths},{"checkpoint",output},{"initial_validation",metrics(baseline)},{"history",json::array()},{"optimizer_state_saved",false},{"training_executed",true}};
  write_json(output+".training.json",report);std::cout<<std::setprecision(10)<<"train_records="<<train.size()<<" validation_records="<<val.size()<<" initial_validation="<<baseline.loss<<std::endl;
  auto started=std::chrono::steady_clock::now();
  for(int epoch=1;epoch<=epochs;++epoch){model->train();std::shuffle(order.begin(),order.end(),rng);double loss_sum=0;
    for(size_t i=0;i<order.size();i+=bs){size_t end=std::min(i+bs,order.size());auto b=batch(train,order,i,end,dev);optim.zero_grad();
      auto loss=torch::smooth_l1_loss(model->forward(b.scan,b.map,b.context),b.label);double value=loss.item<double>();if(!std::isfinite(value))throw std::runtime_error("nonfinite training loss");
      loss.backward();auto norm=torch::nn::utils::clip_grad_norm_(model->parameters(),1.);if(!std::isfinite(norm))throw std::runtime_error("nonfinite gradient norm");optim.step();loss_sum+=value*(end-i);}
    auto validation=evaluate(model,val,vo,bs,dev);bool saved=best.consider(model,validation.loss,epoch);
    report["history"].push_back({{"epoch",epoch},{"train_loss",loss_sum/train.size()},{"validation",metrics(validation)},{"saved_best",saved}});
    report["best_epoch"]=best.epoch();report["best_validation_loss"]=best.loss();write_json(output+".training.json",report);
    std::cout<<"epoch="<<epoch<<" train="<<loss_sum/train.size()<<" validation="<<validation.loss<<" best_epoch="<<best.epoch()<<" saved="<<saved<<std::endl;
  }
  // Verify the artifact on disk, rather than evaluating only the final live model.
  Tln restored;torch::load(restored,output);restored->guided=restored->named_buffers()["schema"][0].item<int>()==4;restored->to(dev);auto check=evaluate(restored,val,vo,bs,dev);
  if(fabs(check.loss-best.loss())>1e-7+fabs(best.loss())*1e-5)throw std::runtime_error("reloaded checkpoint differs from best validation loss");
  report["status"]="completed";report["reloaded_validation"]=metrics(check);report["checkpoint_reload_verified"]=true;
  report["training_seconds"]=std::chrono::duration<double>(std::chrono::steady_clock::now()-started).count();write_json(output+".training.json",report);
  std::cout<<"BEST epoch="<<best.epoch()<<" validation="<<check.loss<<" steer_mae_rad="<<check.steer_mae<<" speed_mae_mps="<<check.speed_mae<<" checkpoint="<<output<<std::endl;return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
