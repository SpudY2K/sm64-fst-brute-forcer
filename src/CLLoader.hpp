#ifndef CLLOADER_HPP
#define CLLOADER_HPP

#define CL_HPP_TARGET_OPENCL_VERSION 220
#define CL_HPP_ENABLE_EXCEPTIONS
#include <CL/opencl.hpp>

#include <filesystem>
#include <utility>

#include <algorithm>
#include <fstream>
#include <ios>
#include <stdexcept>
#include <type_traits>
#include <vector>

std::filesystem::path path_to_self();

namespace detail {
  template <size_t Is>
  using size_t_sink = size_t;

  template <class T>
  class CLRangeBase;

  template <size_t... Is>
  class CLRangeBase<std::index_sequence<Is...>> {
  public:
    CLRangeBase(size_t_sink<Is>... vs) : m_data {vs...} {}

  protected:
    size_t m_data[sizeof...(Is)];
  };

  template <size_t N>
  using CLRangeBaseN = CLRangeBase<std::make_index_sequence<N>>;
}  // namespace detail

template <size_t N>
class CLRange : detail::CLRangeBaseN<N> {
private:
  using Base = detail::CLRangeBaseN<N>;

public:
  static constexpr size_t ndim = N;
  using Base::Base;

  size_t operator[](size_t i) { return m_data[i]; }

  const size_t* data() const { return m_data; }

private:
  using Base::m_data;
};

template <class... Ts>
requires(std::is_integral_v<Ts>&&...) CLRange(Ts&&...)
->CLRange<sizeof...(Ts)>;

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

  template <size_t N = 1, class... Ts>
  void run_kernel(
    const char* name, const CLRange<N>& global, const CLRange<N>& local,
    Ts&&... args);

  template <class... Ts>
  void run_kernel_once(const char* name, Ts&&... args) {
    run_kernel(name, {1}, {1}, std::forward<Ts>(args)...);
  }

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

inline void CLContext::write_buffer(
  cl::Buffer& buf, void* ptr, size_t len, size_t off) {
  m_queue.enqueueWriteBuffer(buf, CL_TRUE, off, len, ptr);
}

inline void CLContext::read_buffer(
  cl::Buffer& buf, void* ptr, size_t len, size_t off) {
  m_queue.enqueueReadBuffer(buf, CL_TRUE, off, len, ptr);
}

template <size_t N, class... Ts>
inline void CLContext::run_kernel(
  const char* name, const CLRange<N>& global, const CLRange<N>& local,
  Ts&&... args) {
  cl::Kernel knl(m_prog, name);
  [&]<size_t... Is>(std::index_sequence<Is...>) {
    (knl.setArg(Is, args), ...);
  }
  (std::index_sequence_for<Ts...> {});

  cl_int res = ::clEnqueueNDRangeKernel(
    m_queue.get(), knl.get(), N, nullptr, global.data(), local.data(), 0,
    nullptr, nullptr);
  if (res != CL_SUCCESS) {
    throw cl::Error(res, "clEnqueueNDRangeKernel failed");
  }

  m_queue.finish();
}

#endif