#pragma once

#include <filesystem>
#include <fstream>
#include <string_view>
#include "structs.h.cl"
#ifndef SOLUTIONWRITER_HPP
#define SOLUTIONWRITER_HPP
class SolutionWriter {
public:
  SolutionWriter(std::filesystem::path path) :
    file(path) {
    file << csv_header << std::flush;
  }
  void write_soln(
  const TenKSolution& tenK, const PUSolution& pu,
  const PlatformSolution& plat, float host_norms[], float normX, float normY, float normZ);
  
private:
  static const std::string_view csv_header;
  
  std::ofstream file;
};
#endif