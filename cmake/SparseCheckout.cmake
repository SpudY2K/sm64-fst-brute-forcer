#[=[
checkout_repo(<name> <url> <branch> <sources>...)
Sparsely clone from a Git repository, treating it as a CMake project.

name - name of the repository.
url - URL of the Git repository.
branch - branch, commit hash, or tag to download
sources - parts of the repository that need to be downloaded.
]=]
function(checkout_repo name url branch)
  if (${ARGC} LESS 4)
    message(FATAL_ERROR "checkout_repo requires 4 arguments: checkout_repo(<url> <branch> <outdir> <sources>...)")
  endif()
  
  set(outdir "${PROJECT_BINARY_DIR}/_git_deps/${name}")
  set(builddir "${PROJECT_BINARY_DIR}/_git_deps_build/${name}")
  
  # Adapted from this answer:
  # https://stackoverflow.com/a/60190760/10808912
  if (NOT EXISTS "${outdir}")
    file(MAKE_DIRECTORY "${outdir}")
    execute_process(
      COMMAND git init
      WORKING_DIRECTORY "${outdir}"
      COMMAND_ECHO STDOUT
      COMMAND_ERROR_IS_FATAL ANY
    )
    execute_process(
      COMMAND git config core.sparseCheckout true
      WORKING_DIRECTORY "${outdir}"
      COMMAND_ECHO STDOUT
      COMMAND_ERROR_IS_FATAL ANY
    )
    foreach(i RANGE 3 ${ARGC})
      file(APPEND "${outdir}/.git/info/sparse-checkout" "${ARGV${i}}\n")
    endforeach()
    
    execute_process(
      COMMAND git remote add origin "${url}"
      WORKING_DIRECTORY "${outdir}"
      COMMAND_ECHO STDOUT
      COMMAND_ERROR_IS_FATAL ANY
    )
  endif()
  if (NOT EXISTS "${builddir}")
    file(MAKE_DIRECTORY "${builddir}")
  endif()
  
  execute_process(
    COMMAND git fetch --depth=1 origin "${branch}"
    WORKING_DIRECTORY "${outdir}"
    COMMAND_ECHO STDOUT
    RESULT_VARIABLE fetch_res
  )
  if (fetch_res EQUAL 0)
    execute_process(
      COMMAND git checkout "${branch}"
      WORKING_DIRECTORY "${outdir}"
      COMMAND_ECHO STDOUT
      COMMAND_ERROR_IS_FATAL ANY
    )
  else()
    if (NOT EXISTS "${outdir}/${ARGV4}")
      message(SEND_ERROR "Could not fetch from `${url}`.")
    else()
      message(WARNING "Could not fetch from `${url}`. Update will be skipped.")
    endif()
  endif()
  
  add_subdirectory("${outdir}" "${builddir}")
  
endfunction()