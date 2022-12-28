# Add new source files here
add_executable(main
  "BoundaryDistance.cpp"
  "BoundaryDistance.hpp"
  "main.cpp"
  "Path.cpp"
  "Path.hpp"
  "Platform.cpp"
  "Platform.hpp"
  "Surface.cpp"
  "Surface.hpp"
  "Trig.cpp"
  "Trig.hpp"
  "vmath.cpp"
  "vmath.hpp"
)

set_target_properties(main PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
  PDB_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)
target_compile_features(main PRIVATE cxx_std_20)
target_link_libraries(main PRIVATE cargs OpenCL::OpenCL)

enable_march_native(main)

if (SUPPORTS_IPO)
  message(STATUS "Link-time optimization supported")
  set_target_properties(main PROPERTIES
    INTERPROCEDURAL_OPTIMIZATION yes
  )
else()
  message(STATUS "Link-time optimization not supported")
endif()

add_opencl_spirv(main "main_gpu.spv" "main.cl")
if (OpenMP_FOUND)
  message(STATUS "OpenMP supported")
  target_link_libraries(main PUBLIC OpenMP::OpenMP_CXX)
else()
  message(STATUS "OpenMP not supported")
endif()