#[=[
add_opencl_spirv(<target> <output> [sources]...)
Add a SPIR-V compilation target.

target - target to place SPIR-V file with
output - filename of SPIR-V file
sources - .cl files to compile to SPIR-V

]=]
function(add_opencl_spirv target output)
  # verify values
  if (NOT TARGET ${target})
    message(FATAL_ERROR "Specified <target> must be a target. (Got ${target})")
  endif()
  if (${output} MATCHES "[/\\]")
    message(FATAL_ERROR "Output must be a file name, not a math.")
  endif()
  
  # create lists of files
  set(dep_list)
  set(comp_list)
  foreach(n RANGE 2 ${ARGC})
    file(REAL_PATH ${ARGV${n}} true_path)
    list(APPEND dep_list ${true_path})
    
    # check if the file is a .cl or .h.cl file
    # this project will use .h.cl to indicate OpenCL C/C++ headers
    cmake_path(GET true_path EXTENSION ext)
    if (${ext} STREQUAL ".cl")
      list(APPEND comp_list "${true_path}")
    endif()
  endforeach()
  
  # Setup compilation command
  if (${CMAKE_SIZEOF_VOID_P} EQUAL 4)
    set(spv_arch "spirv32")
  elseif (${CMAKE_SIZEOF_VOID_P} EQUAL 8)
    set(spv_arch "spirv64")
  endif()
    
  add_custom_command(
    OUTPUT $<TARGET_FILE_DIR:${target}>/${bin_path}
    COMMAND clang -target ${spv_arch} -cl-std=clc++ -O3 -o $<TARGET_FILE_DIR:${target}>/${bin_path} -- ${comp_list}
    DEPENDS ${dep_list}
  )
  
  set(spv_target_name "spv_compile_${target}_${output}")
  add_custom_target(${spv_target_name} DEPENDS $<TARGET_FILE_DIR:${target}>/${bin_path})
  add_dependencies(${target} ${spv_target_name})
endfunction()