macro(gcc_like_enable_march_native target)
  target_compile_options(${target}
    PRIVATE -march=native
  )
endmacro()

macro(msvc_enable_march_native target)
  # execute
  execute_process(COMMAND "${CMAKE_CURRENT_LIST_DIR}/msvc_march_native_helper.exe"
    OUTPUT_VARIABLE msvc_arch_option
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  target_compile_options(${target}
    PRIVATE ${msvc_arch_option}
  )
endmacro()

macro(gcc_like_disable_fp_contract target)
  target_compile_options(${target}
    PRIVATE -ffp-contract=off
  )
endmacro()

macro(msvc_disable_fp_contract target)
  target_compile_options(${target}
    PRIVATE /fp:precise
  )
endmacro()

# Sets -march=native, or equivalent on MSVC.
macro(enable_march_native target)
  if(${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
    gcc_like_enable_march_native(${target})
  elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    gcc_like_enable_march_native(${target})
  elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL "AppleClang")
    gcc_like_enable_march_native(${target})
  elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL "NVIDIA")
    gcc_like_enable_march_native(${target})
  elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL "MSVC")
    msvc_enable_march_native(${target})
  endif()
endmacro()

# Sets -ffp-contract=off, or /fp:precise on MSVC.
macro(disable_fp_contract target)
  if(${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
    gcc_like_disable_fp_contract(${target})
  elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    gcc_like_disable_fp_contract(${target})
  elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL "AppleClang")
    gcc_like_disable_fp_contract(${target})
  elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL "MSVC")
    msvc_disable_fp_contract(${target})
  endif()
endmacro()