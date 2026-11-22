#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
namespace fs=std::filesystem;using json=nlohmann::json;
int main(int argc,char** argv){try{
  if(argc<4)throw std::runtime_error("Usage: el_merge_teachers NEW_OUTPUT [--allow-recorded-stop] SHARD1 SHARD2 [SHARD...]");
  bool allow_stop=std::string(argv[2])=="--allow-recorded-stop";int first=allow_stop?3:2;
  if(argc-first<2)throw std::runtime_error("at least two shards required");
  fs::path out=argv[1];if(fs::exists(out))throw std::runtime_error("output already exists; refusing overwrite");
  json episodes=json::array(),shards=json::array();std::set<std::string> paths,names,attempt_names;std::vector<fs::path> attempts;
  for(int i=first;i<argc;++i){fs::path dir=fs::canonical(argv[i]);std::ifstream f(dir/"summary.json");json rows;f>>rows;
    if(!rows.is_array()||rows.empty())throw std::runtime_error("empty or invalid shard summary");
    std::ifstream first_meta(rows.at(0).at("path").get<std::string>()+".json");json metadata;first_meta>>metadata;
    std::set<std::string> expected_sources,observed_sources;
    for(auto& spec:metadata.at("teacher_config").at("episodes"))expected_sources.insert(spec.at("name").get<std::string>());
    for(auto& row:rows){if(row.value("metrics_only",false))throw std::runtime_error("probe is not training data");
      auto path=row.at("path").get<std::string>(),name=row.at("name").get<std::string>();
      if(!fs::is_regular_file(path)||!fs::is_regular_file(path+".json"))throw std::runtime_error("missing dataset or sidecar");
      if(!paths.insert(fs::canonical(path).string()).second||!names.insert(name).second)throw std::runtime_error("duplicate dataset in shards");
      observed_sources.insert(row.value("source_attempt",name));
      episodes.push_back(row);
    }
    json completion={{"status","complete"}};
    if(observed_sources!=expected_sources){
      if(!allow_stop||!fs::exists(dir/"collection_stop.json"))throw std::runtime_error("shard collection incomplete or source names disagree with its configuration");
      std::ifstream stopped(dir/"collection_stop.json");stopped>>completion;
      if(completion.at("status")!="intentionally_stopped"||completion.at("reason").get<std::string>().empty())throw std::runtime_error("invalid stop record");
      auto completed=completion.at("completed_sources").get<std::set<std::string>>();
      auto accounted=completion.at("skipped_sources").get<std::set<std::string>>();accounted.insert(completion.at("interrupted_source").get<std::string>());
      for(auto& name:completed)if(accounted.count(name))throw std::runtime_error("stopped source also marked complete");
      accounted.insert(completed.begin(),completed.end());
      if(completed!=observed_sources||accounted!=expected_sources)throw std::runtime_error("stop record does not account for all configured sources");
    }
    if(fs::exists(dir/"attempts"))for(auto& entry:fs::directory_iterator(dir/"attempts"))if(entry.path().extension()==".json"){
      if(!attempt_names.insert(entry.path().filename().string()).second)throw std::runtime_error("duplicate source attempt metadata");attempts.push_back(entry.path());}
    shards.push_back({{"directory",dir.string()},{"summary_entries",rows.size()},{"completion",completion}});
  }
  fs::create_directories(out/"attempts");for(auto& p:attempts)fs::copy_file(p,out/"attempts"/p.filename());
  std::ofstream(out/"summary.json")<<episodes.dump(2);std::ofstream(out/"shards.json")<<shards.dump(2);
  std::cout<<"Merged "<<episodes.size()<<" metadata entries; ELDS files remain in original shards. Run el_audit_teachers before training.\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
