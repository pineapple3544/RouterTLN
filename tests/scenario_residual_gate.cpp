#include "el/scenario_driver.hpp"
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <stdexcept>

using namespace el::scenario;

// Compare deployment inference on recorded input vectors, including both gate outcomes.
int main(int argc,char** argv){
  if(argc!=4)throw std::invalid_argument("expected RCN1 RCN2 capture.jsonl");
  Residual base(argv[1]), programmed(argv[2]);
  std::ifstream capture(argv[3]);
  size_t checked=0,deferred=0;
  std::string line;
  while(std::getline(capture,line)){
    const auto row=nlohmann::json::parse(line);
    if(!row.at("valid").get<bool>())continue;
    const auto features=row.at("x").get<Features>();
    bool old_gate=true,new_gate=false;
    const auto a=base.infer(features,&old_gate);
    const auto b=programmed.infer(features,&new_gate);
    if(old_gate||a.steer!=b.steer||a.speed!=b.speed)
      throw std::runtime_error("WSP changed a control output or RCN1 requested a deferral");
    ++checked;
    deferred+=new_gate;
  }
  if(!deferred||deferred==checked)throw std::runtime_error("gate outputs do not vary");
  std::cout<<"identical controls "<<checked<<", negative gate scores "<<deferred<<"\n";
}
