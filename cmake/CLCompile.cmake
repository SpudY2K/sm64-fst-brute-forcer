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
  math(EXPR last_arg "${ARGC} - 1")
  foreach(n RANGE 2 ${last_arg})
    message(STATUS "Processing file ${ARGV${n}}")
    file(REAL_PATH "${ARGV${n}}" true_path)
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
  
  if (NOT IS_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/spirv")
    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/spirv")
  endif()
  
  add_custom_command(
    OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/spirv/${output}"
    COMMAND clang -target ${spv_arch} -cl-std=clc++2021 -O0 -o "${CMAKE_CURRENT_BINARY_DIR}/spirv/${output}" -- ${comp_list}
    DEPENDS ${dep_list}
  )
  add_custom_target("spirv_${target}_${output}"
    COMMAND ${CMAKE_COMMAND} -DSCRIPT_OPTION=checked-copy -P ${CMAKE_CURRENT_FUNCTION_LIST_FILE}
      "${CMAKE_CURRENT_BINARY_DIR}/spirv/${output}" "$<TARGET_FILE_DIR:${target}>/${output}"
    DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/spirv/${output}"
  )
  add_dependencies(${target} "spirv_${target}_${output}")
endfunction()

if (CMAKE_SCRIPT_MODE_FILE AND NOT CMAKE_PARENT_LIST_FILE)
  # Utility functions requiring a script.
  if (${SCRIPT_OPTION} STREQUAL "checked-copy")
    if (NOT EXISTS ${CMAKE_ARGV1})
      execute_process(
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_ARGV4} ${CMAKE_ARGV5}
      )
    else()
      execute_process(
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_ARGV4} ${CMAKE_ARGV5}
      )
    endif()
  endif()
endif()