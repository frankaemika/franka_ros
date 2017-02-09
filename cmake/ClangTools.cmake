file(GLOB_RECURSE SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB_RECURSE HEADERS
  ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
)

find_program(CLANG_FORMAT_PROG clang-format DOC "'clang-format' executable")
if(CLANG_FORMAT_PROG)
  add_custom_target(format
    COMMAND ${CLANG_FORMAT_PROG} -i ${SOURCES} ${HEADERS}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
    COMMENT "Formatting source code with clang-format"
    VERBATIM
  )
  add_custom_target(check-format
    COMMAND scripts/format-check.sh ${CLANG_FORMAT_PROG} -output-replacements-xml ${SOURCES} ${HEADERS}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
    COMMENT "Checking code formatting with clang-format"
    VERBATIM
  )
endif()
find_program(CLANG_TIDY_PROG clang-tidy DOC "'clang-tidy' executable")
if(CLANG_TIDY_PROG)
  add_custom_target(tidy
    COMMAND ${CLANG_TIDY_PROG} -p=${CMAKE_BINARY_DIR} ${SOURCES}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
    DEPENDS simple_viz_node
    COMMENT "Running clang-tidy"
    VERBATIM
  )
  add_custom_target(check-tidy
    COMMAND scripts/fail-on-output.sh ${CLANG_TIDY_PROG} -p=${CMAKE_BINARY_DIR} ${SOURCES}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
    DEPENDS simple_viz_node
    COMMENT "Running clang-tidy"
    VERBATIM
  )
endif()
