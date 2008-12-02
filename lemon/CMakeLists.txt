INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR})

ADD_LIBRARY(lemon
  arg_parser.cc
  base.cc
  color.cc
  lp_base.cc
  lp_skeleton.cc
  lp_utils.cc
  random.cc)

INSTALL(
  TARGETS lemon
  ARCHIVE DESTINATION lib
  COMPONENT library)

INSTALL(
  DIRECTORY . bits concepts
  DESTINATION include/lemon
  COMPONENT headers
  FILES_MATCHING PATTERN "*.h")
