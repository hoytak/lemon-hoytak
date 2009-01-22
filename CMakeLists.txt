CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(PROJECT_NAME "LEMON")
SET(PROJECT_VERSION "hg-tip" CACHE STRING "The version string.")

PROJECT(${PROJECT_NAME})

SET(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

INCLUDE(FindDoxygen)
INCLUDE(FindGhostscript)
FIND_PACKAGE(GLPK 4.33)

ADD_DEFINITIONS(-DHAVE_CONFIG_H)

IF(MSVC)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4250 /wd4355 /wd4800 /wd4996")
# Suppressed warnings:
# C4250: 'class1' : inherits 'class2::member' via dominance
# C4355: 'this' : used in base member initializer list
# C4800: 'type' : forcing value to bool 'true' or 'false' (performance warning)
# C4996: 'function': was declared deprecated
ENDIF(MSVC)

IF(GLPK_FOUND)
  SET(HAVE_LP TRUE)
  SET(HAVE_MIP TRUE)
  SET(HAVE_GLPK TRUE)
ENDIF(GLPK_FOUND)

ENABLE_TESTING()

ADD_SUBDIRECTORY(lemon)
ADD_SUBDIRECTORY(demo)
ADD_SUBDIRECTORY(doc)
ADD_SUBDIRECTORY(test)

IF(WIN32)
  INSTALL(FILES ${CMAKE_SOURCE_DIR}/cmake/nsis/lemon.ico
    DESTINATION bin)
ENDIF(WIN32)

IF(WIN32)
  SET(CPACK_PACKAGE_NAME ${PROJECT_NAME})
  SET(CPACK_PACKAGE_VENDOR
    "EGRES - Egervary Research Group on Combinatorial Optimization")
  SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY
    "LEMON - Library of Efficient Models and Optimization in Networks")
  SET(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_SOURCE_DIR}/LICENSE")

  SET(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})

  SET(CPACK_PACKAGE_INSTALL_DIRECTORY
    "${PROJECT_NAME} ${PROJECT_VERSION}")
  SET(CPACK_PACKAGE_INSTALL_REGISTRY_KEY
    "${PROJECT_NAME} ${PROJECT_VERSION}")

  # Variables to generate a component-based installer.
  #SET(CPACK_COMPONENTS_ALL headers library html_documentation)

  #SET(CPACK_COMPONENT_HEADERS_DISPLAY_NAME "C++ headers")
  #SET(CPACK_COMPONENT_LIBRARY_DISPLAY_NAME "Static library")
  #SET(CPACK_COMPONENT_HTML_DOCUMENTATION_DISPLAY_NAME "HTML documentation")

  #SET(CPACK_COMPONENT_HEADERS_DESCRIPTION
  #  "C++ header files for use with the LEMON library")
  #SET(CPACK_COMPONENT_LIBRARY_DESCRIPTION
  #  "Static library used to build programs with LEMON")
  #SET(CPACK_COMPONENT_HTML_DOCUMENTATION_DESCRIPTION
  #  "Doxygen generated documentation")

  #SET(CPACK_COMPONENT_HEADERS_DEPENDS library)

  #SET(CPACK_COMPONENT_HEADERS_GROUP "Development")
  #SET(CPACK_COMPONENT_LIBRARY_GROUP "Development")
  #SET(CPACK_COMPONENT_HTML_DOCUMENTATION_GROUP "Documentation")

  #SET(CPACK_COMPONENT_GROUP_DEVELOPMENT_DESCRIPTION
  #  "Components needed to develop software using LEMON")
  #SET(CPACK_COMPONENT_GROUP_DOCUMENTATION_DESCRIPTION
  #  "Documentation of LEMON")

  #SET(CPACK_ALL_INSTALL_TYPES Full Developer)

  #SET(CPACK_COMPONENT_HEADERS_INSTALL_TYPES Developer Full)
  #SET(CPACK_COMPONENT_LIBRARY_INSTALL_TYPES Developer Full)
  #SET(CPACK_COMPONENT_HTML_DOCUMENTATION_INSTALL_TYPES Full)

  SET(CPACK_GENERATOR "NSIS")
  SET(CPACK_NSIS_MUI_ICON "${CMAKE_SOURCE_DIR}/cmake/nsis/lemon.ico")
  SET(CPACK_NSIS_MUI_UNIICON "${CMAKE_SOURCE_DIR}/cmake/nsis/uninstall.ico")
  #SET(CPACK_PACKAGE_ICON "${CMAKE_SOURCE_DIR}/cmake/nsis\\\\installer.bmp")
  SET(CPACK_NSIS_INSTALLED_ICON_NAME "bin\\\\lemon.ico")
  SET(CPACK_NSIS_DISPLAY_NAME "${CPACK_PACKAGE_INSTALL_DIRECTORY} ${PROJECT_NAME}")
  SET(CPACK_NSIS_HELP_LINK "http:\\\\\\\\lemon.cs.elte.hu")
  SET(CPACK_NSIS_URL_INFO_ABOUT "http:\\\\\\\\lemon.cs.elte.hu")
  SET(CPACK_NSIS_CONTACT "lemon-user@lemon.cs.elte.hu")
  SET(CPACK_NSIS_CREATE_ICONS_EXTRA "
    CreateShortCut \\\"$SMPROGRAMS\\\\$STARTMENU_FOLDER\\\\Documentation.lnk\\\" \\\"$INSTDIR\\\\doc\\\\index.html\\\"
    ")
  SET(CPACK_NSIS_DELETE_ICONS_EXTRA "
    !insertmacro MUI_STARTMENU_GETFOLDER Application $MUI_TEMP
    Delete \\\"$SMPROGRAMS\\\\$MUI_TEMP\\\\Documentation.lnk\\\"
    ")

  INCLUDE(CPack)
ENDIF(WIN32)
