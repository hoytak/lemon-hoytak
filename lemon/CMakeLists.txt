INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR})

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
