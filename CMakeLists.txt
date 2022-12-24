cmake_minimum_required(VERSION 3.20)
# Feel free to change the description string, I didn't know what to put here - superminer
project(sm64-platform-max-tilt-bruteforcer
  DESCRIPTION "Bruteforces platform tilt for BitFS 0x"
  LANGUAGES CXX
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
include(CompilerConfig)
include(CLCompile)
include(SparseCheckout)

checkout_repo(cargs "https://github.com/likle/cargs.git" "stable"
  "/src/" "/include/" "/cmake" "/CMakeLists.txt"
)

include(CheckIPOSupported)
check_ipo_supported(RESULT SUPPORTS_IPO)
find_package(OpenMP)

add_subdirectory(src)

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the build type." FORCE)
endif()