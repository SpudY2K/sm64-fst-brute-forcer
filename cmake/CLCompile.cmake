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
      list(APPEND comp_list "${ARGV${n}}")
    endif()
  endforeach()
  
  # Setup variables
  if (${CMAKE_SIZEOF_VOID_P} EQUAL 4)
    set(spv_arch "spirv32")
  elseif (${CMAKE_SIZEOF_VOID_P} EQUAL 8)
    set(spv_arch "spirv64")
  endif()
  set(spv_ver "")
  
  set(tmpdir "${CMAKE_CURRENT_BINARY_DIR}/CMakeFiles/${target}.dir/spirv")
  set(tmpdir2 "${CMAKE_CURRENT_BINARY_DIR}/CMakeFiles/${target}.dir")
  if (NOT IS_DIRECTORY "${tmpdir}")
    file(MAKE_DIRECTORY "${tmpdir}")
  endif()
  if (NOT IS_DIRECTORY "${tmpdir2}")
    file(MAKE_DIRECTORY "${tmpdir2}")
  endif()
  
  set(obj_list)
  # Compilation commands
  foreach (file IN LISTS comp_list)
    cmake_path(
      ABSOLUTE_PATH file 
      BASE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} 
      NORMALIZE 
      OUTPUT_VARIABLE src_path
    )
    
    cmake_path(
      ABSOLUTE_PATH file 
      BASE_DIRECTORY ${tmpdir} 
      NORMALIZE 
      OUTPUT_VARIABLE obj_path
    )
    cmake_path(
      APPEND_STRING obj_path ".deps"
      OUTPUT_VARIABLE deps_path
    )
    cmake_path(APPEND_STRING obj_path ".spv")
    
    list(APPEND obj_list "${obj_path}")
    
    cmake_path(
      RELATIVE_PATH obj_path 
      BASE_DIRECTORY "${PROJECT_BINARY_DIR}"
      OUTPUT_VARIABLE friendly_obj_path
    )
    
    add_custom_command(
      OUTPUT "${obj_path}"
      # COMMAND clang -target "${spv_arch}${spv_ver}" -cl-std=clc++2021 -O0 
      #   -c -o "${obj_path}" -MD -MF "${deps_path}" "${src_path}"
      COMMAND ${CMAKE_COMMAND} -DSCRIPT_OPTION=spirv-compile -P ${CMAKE_CURRENT_FUNCTION_LIST_FILE} --
        "${spv_arch}${spv_ver}" "${src_path}" "${deps_path}" "${obj_path}"
      DEPENDS "${src_path}"
      DEPFILE "${deps_path}"
      COMMENT "Building SPIR-V object ${friendly_obj_path}"
    )
  endforeach()
  # Linking commands
  set(link_path "${tmpdir2}")
  cmake_path(APPEND link_path "${output}")
  
  cmake_path(
    RELATIVE_PATH link_path
    BASE_DIRECTORY "${PROJECT_BINARY_DIR}"
    OUTPUT_VARIABLE friendly_link_path
  )
  
  add_custom_command(
    OUTPUT "${link_path}"
    COMMAND spirv-link --target-env opencl2.2 ${obj_list}
    DEPENDS ${obj_list}
    COMMENT "Linking SPIR-V module ${friendly_link_path}"
  )
  
  # Copy final SPIR-V target to target dir
  add_custom_target("spirv_${target}_${output}"
    COMMAND ${CMAKE_COMMAND} -DSCRIPT_OPTION=checked-copy -P ${CMAKE_CURRENT_FUNCTION_LIST_FILE} --
    "${tmpdir2}/${output}" "$<TARGET_FILE_DIR:${target}>/${output}"
    DEPENDS "${tmpdir2}/${output}"
  )
  add_dependencies(${target} "spirv_${target}_${output}")
endfunction()

if (CMAKE_SCRIPT_MODE_FILE AND NOT CMAKE_PARENT_LIST_FILE)
  include("${CMAKE_CURRENT_LIST_DIR}/ScriptArgs.cmake")
  script_args_init()
  # Utility functions requiring a script.
  if (${SCRIPT_OPTION} STREQUAL "checked-copy")
    if (NOT EXISTS ${SCRIPT_ARGV1})
      execute_process(
        COMMAND ${CMAKE_COMMAND} -E copy ${SCRIPT_ARGV1} ${SCRIPT_ARGV2}
      )
    else()
      execute_process(
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${SCRIPT_ARGV1} ${SCRIPT_ARGV2}
      )
    endif()
  elseif(${SCRIPT_OPTION} STREQUAL "spirv-compile")
    # Jank script to compile SPIR-V until Clang improves.
    execute_process(
      COMMAND clang -target ${SCRIPT_ARGV1} -cl-std=clc++2021 -emit-llvm -O0 -o - -c ${SCRIPT_ARGV2} -MD -MF ${SCRIPT_ARGV3}
      COMMAND llvm-spirv --spirv-max-version=1.2 --spirv-target-env=CL2.0
      COMMAND spirv-opt --target-env=opencl2.2 -O -o ${SCRIPT_ARGV4}
    )
  endif()
endif()