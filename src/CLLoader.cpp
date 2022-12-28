#include "CLLoader.hpp"
#include <array>
#include <filesystem>
#include <system_error>
#include <CL/opencl.hpp>

namespace fs = std::filesystem;

#if defined(__linux__)
#include <unistd.h>

fs::path path_to_self() {
  std::array<char, PATH_MAX> data;
  ssize_t len = readlink("/proc/self/exe", data.data(), data.size());
  if (len < 0) {
    throw std::system_error(errno, std::system_category());
  }
  return fs::path(std::string_view(data.data(), len));
}
#elif defined(_WIN32)
#include <windows.h>
fs::path path_to_self() {
  std::array<wchar_t, MAX_PATH> data;
  DWORD len = GetModuleFileNameW(NULL, data.data(), data.size());
  if (len == 0 || len == data.size()) {
    throw std::system_error(errno, std::system_category());
  }
  return fs::path(std::wstring_view(data.data(), len));
}
#endif