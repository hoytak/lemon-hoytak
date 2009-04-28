INCLUDE_DIRECTORIES(
  ${CMAKE_SOURCE_DIR}
  ${PROJECT_BINARY_DIR}
)

CONFIGURE_FILE(
  ${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/config.h
)

ADD_LIBRARY(lemon
  arg_parser.cc
  base.cc
  color.cc
  random.cc
  bits/windows.cc
)

INSTALL(
  TARGETS lemon
  ARCHIVE DESTINATION lib
  COMPONENT library)

INSTALL(
  DIRECTORY . bits concepts
  DESTINATION include/lemon
  COMPONENT headers
  FILES_MATCHING PATTERN "*.h")

INSTALL(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/config.h
  DESTINATION include/lemon
  COMPONENT headers)
