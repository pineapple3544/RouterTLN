#include "el/dataset.hpp"
#include <filesystem>
#include <stdexcept>
#include <cstring>
namespace el {
namespace {
template<class T>void put(std::ostream& o,const T& x){o.write(reinterpret_cast<const char*>(&x),sizeof(x));}
template<class T>void get(std::istream& i,T& x){i.read(reinterpret_cast<char*>(&x),sizeof(x));if(!i)throw std::runtime_error("truncated dataset");}
void endian(){uint32_t v=1;if(*reinterpret_cast<char*>(&v)!=1)throw std::runtime_error("ELDS requires little endian");}
template<class A>void put_array(std::ostream& o,const A& x){o.write(reinterpret_cast<const char*>(x.data()),x.size()*sizeof(float));}
template<class A>void get_array(std::istream& i,A& x){i.read(reinterpret_cast<char*>(x.data()),x.size()*sizeof(float));if(!i)throw std::runtime_error("truncated observation");}
}
DatasetWriter::DatasetWriter(const std::string& path,const std::string& metadata){
  endian();if(std::filesystem::exists(path))throw std::runtime_error("refuse to overwrite episode: "+path);
  std::filesystem::create_directories(std::filesystem::path(path).parent_path());file_.open(path,std::ios::binary);
  if(!file_)throw std::runtime_error("cannot create dataset");
  file_.write("ELDS",4);put(file_,dataset_version);
  for(uint32_t v:{(uint32_t)beams,(uint32_t)history,(uint32_t)crop,(uint32_t)context_size})put(file_,v);
  std::ofstream m(path+".json");m<<metadata;if(!m)throw std::runtime_error("cannot write episode metadata");
}
void DatasetWriter::append(const Record& r){
  put(file_,r.step);put(file_,r.stamp);put(file_,r.mode);put(file_,r.valid);put(file_,r.collision);
  put_array(file_,r.observation.scans);put_array(file_,r.observation.map);put_array(file_,r.observation.context);
  put(file_,r.label.steer);put(file_,r.label.speed);put(file_,r.executed.steer);put(file_,r.executed.speed);
  if(!file_)throw std::runtime_error("dataset write failed");
}
void DatasetWriter::flush(){file_.flush();}
std::vector<Record> read_dataset(const std::string& path){
  endian();std::ifstream f(path,std::ios::binary);char magic[4];f.read(magic,4);
  if(!f||memcmp(magic,"ELDS",4))throw std::runtime_error("invalid dataset magic");
  for(uint32_t expected:{dataset_version,(uint32_t)beams,(uint32_t)history,(uint32_t)crop,(uint32_t)context_size}){uint32_t v;get(f,v);if(v!=expected)throw std::runtime_error("dataset schema mismatch");}
  std::vector<Record> records;
  while(f.peek()!=std::char_traits<char>::eof()){
    Record r;get(f,r.step);get(f,r.stamp);get(f,r.mode);get(f,r.valid);get(f,r.collision);
    get_array(f,r.observation.scans);get_array(f,r.observation.map);get_array(f,r.observation.context);
    get(f,r.label.steer);get(f,r.label.speed);get(f,r.executed.steer);get(f,r.executed.speed);
    if(!std::isfinite(r.label.steer)||!std::isfinite(r.label.speed))throw std::runtime_error("nonfinite label");
    records.push_back(r);
  }return records;
}
}
