find_program(CLANG_FORMAT_PROG clang-format DOC "'clang-format' executable")
if(CLANG_FORMAT_PROG)
  add_custom_target(format
    COMMAND ${CLANG_FORMAT_PROG} -i ${CLANG_TOOLS_SOURCES} ${CLANG_TOOLS_HEADERS}
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/..
    COMMENT "Formatting source code with clang-format"
    VERBATIM
  )
  add_custom_target(check-format
    COMMAND scripts/format-check.sh ${CLANG_FORMAT_PROG} -output-replacements-xml ${CLANG_TOOLS_SOURCES} ${CLANG_TOOLS_HEADERS}
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/..
    COMMENT "Checking code formatting with clang-format"
    VERBATIM
  )
endif()
find_program(CLANG_TIDY_PROG clang-tidy DOC "'clang-tidy' executable")
if(CLANG_TIDY_PROG)
  add_custom_target(tidy
    COMMAND ${CLANG_TIDY_PROG} -p=${CMAKE_BINARY_DIR} ${CLANG_TOOLS_SOURCES}
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/..
    DEPENDS ${CLANG_TIDY_DEPENDS}
    COMMENT "Running clang-tidy"
    VERBATIM
  )
  add_custom_target(check-tidy
    COMMAND scripts/fail-on-output.sh ${CLANG_TIDY_PROG} -p=${CMAKE_BINARY_DIR} ${CLANG_TOOLS_SOURCES}
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/..
    DEPENDS ${CLANG_TIDY_DEPENDS}
    COMMENT "Running clang-tidy"
    VERBATIM
  )
  add_custom_target(tidy
    COMMAND ${CLANG_TIDY_PROG} -p=${CMAKE_BINARY_DIR} ${SOURCES}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
    DEPENDS franka_joint_state_publisher 
    COMMENT "Running clang-tidy"
    VERBATIM
  )
  add_custom_target(check-tidy
    COMMAND scripts/fail-on-output.sh ${CLANG_TIDY_PROG} -p=${CMAKE_BINARY_DIR} ${SOURCES}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
    DEPENDS franka_joint_state_publisher
    COMMENT "Running clang-tidy"
    VERBATIM
  )

endif()
