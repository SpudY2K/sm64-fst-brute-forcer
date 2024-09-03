link_directories("/opt/intel/oneapi/2024.2/lib/")

add_executable(fst-brute-forcer-sycl
  "main.cpp"
  "FST.cpp"
  "FST.hpp"
  "Platform.cpp"
  "Platform.hpp"
  "Surface.cpp"
  "Surface.hpp"
  "Trig.cpp"
  "Trig.hpp"
  "vmath.cpp"
  "vmath.hpp"
)

target_include_directories(fst-brute-forcer-sycl PRIVATE /opt/intel/oneapi/2024.2/include/)
target_include_directories(fst-brute-forcer-sycl PRIVATE /opt/intel/oneapi/2024.2/include/sycl)

target_link_libraries(fst-brute-forcer-sycl PRIVATE "-lsycl")

set_target_properties(fst-brute-forcer-sycl PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED yes
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
  PDB_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-sycl-id-queries-fit-in-int -fsycl -fsycl-unnamed-lambda -fsycl-targets=amdgcn-amd-amdhsa,spir64 -Xsycl-target-backend=amdgcn-amd-amdhsa --offload-arch=gfx1030 -Wno-c++11-narrowing")

if (SUPPORTS_MARCH_NATIVE)
  if (SUPPORTS_FFP_CONTRACT)
    message(STATUS "Compiler supports -march=native -ffp-contract=off")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -ffp-contract=off")
  else()
    message(STATUS "Compiler supports -march=native but not -ffp-contract=off")
  endif()
else()
  message(STATUS "Compiler does not support -march=native")
endif()
if (SUPPORTS_IPO)
  message(STATUS "Link-time optimization supported")
  set_target_properties(fst-brute-forcer-sycl PROPERTIES
    INTERPROCEDURAL_OPTIMIZATION yes
  )
else()
  message(STATUS "Link-time optimization not supported")
endif()
