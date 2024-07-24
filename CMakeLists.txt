cmake_minimum_required(VERSION 3.15)
project(sm64-fst-brute-forcer
  DESCRIPTION "Bruteforces Final Speed Transfer setups for BitFS 0x"
  LANGUAGES CXX
)

set(CMAKE_C_COMPILER "/opt/intel/oneapi/2024.2/bin/icx")
set(CMAKE_CXX_COMPILER "/opt/intel/oneapi/2024.2/bin/icpx")

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-march=native" SUPPORTS_MARCH_NATIVE)
check_cxx_compiler_flag("-ffp-contract=off" SUPPORTS_FFP_CONTRACT)
include(CheckIPOSupported)
check_ipo_supported(RESULT SUPPORTS_IPO)

add_subdirectory(platform_tilt_brute_forcer/platform_tilt_brute_forcer)

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the build type." FORCE)
endif()
