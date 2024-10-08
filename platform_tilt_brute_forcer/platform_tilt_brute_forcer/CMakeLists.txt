# Add new source files here
add_executable(fst-brute-forcer
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
  "utils.cpp"
  "utils.hpp"
)

set_target_properties(fst-brute-forcer PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED yes
  CMAKE_CUDA_STANDARD 17
  CMAKE_CUDA_STANDARD_REQUIRED yes
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
  PDB_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)

set_source_files_properties(FST.cpp PROPERTIES LANGUAGE CUDA)

set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -fmad=false -arch=native")

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
  set_target_properties(fst-brute-forcer PROPERTIES
    INTERPROCEDURAL_OPTIMIZATION yes
  )
else()
  message(STATUS "Link-time optimization not supported")
endif()
