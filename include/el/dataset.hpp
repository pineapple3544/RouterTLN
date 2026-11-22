#pragma once
#include "el/core.hpp"
#include <fstream>
namespace el {
// ELDS v1: explicit little-endian scalar fields, followed by fixed float32 arrays.
// Each file is ONE episode. Split entire files, never adjacent frames.
constexpr uint32_t dataset_version=1;
struct Record {uint64_t step=0;double stamp=0;int32_t mode=0,valid=0,collision=0;Observation observation;Control label,executed;};
class DatasetWriter {
 public:explicit DatasetWriter(const std::string&,const std::string& metadata);
  void append(const Record&);void flush();
 private:std::ofstream file_;
};
std::vector<Record> read_dataset(const std::string&);
}
