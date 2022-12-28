#pragma once

#include <bits/utility.h>
#include <algorithm>
#include <fstream>
#include <ios>
#include <stdexcept>
#include <vector>
#ifndef CLLOADER_HPP
  #define CLLOADER_HPP

  #define CL_HPP_TARGET_OPENCL_VERSION 220
  #define CL_HPP_ENABLE_EXCEPTIONS
  #include <CL/opencl.hpp>

  #include <filesystem>
  #include <utility>

std::filesystem::path path_to_self();

class CLContext {
public:
  CLContext() :
    m_plat(cl::Platform::getDefault()),
    m_dev([this]() {
      cl::vector<cl::Device> devs;
      m_plat.getDevices(CL_DEVICE_TYPE_GPU, &devs);
      if (devs.size() == 0)
        throw new std::runtime_error("No GPUs available");
      return std::move(devs[0]);
    }()),
    m_ctx(m_dev),
    m_prog([this]() {
      auto spv_path = path_to_self().parent_path() / "main_gpu.spv";
      std::vector<char> data;
      {
        std::ifstream file(
          spv_path, std::ios::in | std::ios::binary | std::ios::ate);
        data.reserve(file.tellg());
        file.seekg(0, std::ios::beg);
        std::copy(
          std::istreambuf_iterator(file), std::istreambuf_iterator<char>(),
          data.begin());
      }

      return cl::Program(data, true);
    }()),
    m_queue(m_ctx) {}
  
  template <class T = char>
  cl::Buffer alloc(size_t size);
  void read_buffer(cl::Buffer& buf, void* ptr, size_t len, size_t off = 0);
  void write_buffer(cl::Buffer& buf, void* ptr, size_t len, size_t off = 0);

  template <class... Ts>
  void run_kernel(
    const char* name, const cl::NDRange& global, const cl::NDRange& local,
    Ts&&... args);

private:
  cl::Platform m_plat;
  cl::Device m_dev;
  cl::Context m_ctx;
  cl::Program m_prog;
  cl::CommandQueue m_queue;
};

template <class T>
inline cl::Buffer CLContext::alloc(size_t len) {
  return cl::Buffer(m_ctx, CL_MEM_READ_WRITE, len * sizeof(T));
}

inline void CLContext::write_buffer(cl::Buffer& buf, void* ptr, size_t len, size_t off) {
  m_queue.enqueueWriteBuffer(buf, CL_TRUE, off, len, ptr);
}

inline void CLContext::read_buffer(cl::Buffer& buf, void* ptr, size_t len, size_t off) {
  m_queue.enqueueReadBuffer(buf, CL_TRUE, off, len, ptr);
}

template <class... Ts>
inline void CLContext::run_kernel(
  const char* name, const cl::NDRange& global, const cl::NDRange& local,
  Ts&&... args) {
  cl::Kernel knl(m_prog, name);
  [&]<size_t... Is>(std::index_sequence<Is...>) {
    (knl.setArg(Is, args), ...);
  }
  (std::index_sequence_for<Ts...> {});
  m_queue.enqueueNDRangeKernel(knl, cl::NullRange, global, local);
  m_queue.finish();
}

#endif