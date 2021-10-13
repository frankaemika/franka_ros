include(CMakeParseArguments)

find_program(PEP_FORMAT_PROG pycodestyle DOC "'pycodestyle' executable")
if(PEP_FORMAT_PROG AND NOT TARGET pyformat)
  add_custom_target(check-pyformat)
endif()

function(add_pyformat_target _target)
  if(NOT PEP_FORMAT_PROG)
    return()
  endif()
  cmake_parse_arguments(ARG "" "" "FILES" ${ARGN})

  add_custom_target(check-pyformat-${_target}
    COMMAND ${PEP_FORMAT_PROG} ${ARG_FILES} 
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/..
    COMMENT "Checking ${_target} code formatting with pycodestyle"
    VERBATIM
  )
  add_dependencies(check-pyformat check-pyformat-${_target})
endfunction()