cmake_minimum_required(VERSION 3.15)
# Feel free to change the description string, I didn't know what to put here - superminer
project(sm64-platform-max-tilt-bruteforcer
  DESCRIPTION "Bruteforces platform tilt for BitFS 0x"
  LANGUAGES CUDA CXX
)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-march=native" SUPPORTS_MARCH_NATIVE)
check_cxx_compiler_flag("-ffp-contract=off" SUPPORTS_FFP_CONTRACT)
include(CheckIPOSupported)
check_ipo_supported(RESULT SUPPORTS_IPO)

add_subdirectory(platform_tilt_brute_forcer/platform_tilt_brute_forcer)

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the build type." FORCE)
endif()
