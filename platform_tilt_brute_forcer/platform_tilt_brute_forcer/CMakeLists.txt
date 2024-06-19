# Add new source files here
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
add_sycl_to_target(
  TARGET fst-brute-forcer-sycl SOURCES
  "FST.cpp"
  "FST.hpp"
)

set_target_properties(fst-brute-forcer-sycl PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED yes
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
  PDB_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)

if(SUPPORTS_MARCH_NATIVE)
  if(SUPPORTS_FFP_CONTRACT)
    message(STATUS "Compiler supports -march=native -ffp-contract=off")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -ffp-contract=off")
  else()
    message(STATUS "Compiler supports -march=native but not -ffp-contract=off")
  endif()
else()
  message(STATUS "Compiler does not support -march=native")
endif()

if(SUPPORTS_IPO)
  message(STATUS "Link-time optimization supported")
  set_target_properties(fst-brute-forcer-sycl PROPERTIES
    INTERPROCEDURAL_OPTIMIZATION yes
  )
else()
  message(STATUS "Link-time optimization not supported")
endif()
