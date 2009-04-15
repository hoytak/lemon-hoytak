SET(PACKAGE_NAME ${PROJECT_NAME})
SET(PACKAGE_VERSION ${PROJECT_VERSION})
SET(abs_top_srcdir ${PROJECT_SOURCE_DIR})
SET(abs_top_builddir ${PROJECT_BINARY_DIR})

CONFIGURE_FILE(
  ${PROJECT_SOURCE_DIR}/doc/Doxyfile.in
  ${PROJECT_BINARY_DIR}/doc/Doxyfile
  @ONLY)

IF(DOXYGEN_EXECUTABLE AND GHOSTSCRIPT_EXECUTABLE)
  FILE(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html/)
  IF(UNIX)
    ADD_CUSTOM_TARGET(html
      COMMAND rm -rf gen-images
      COMMAND mkdir gen-images
      COMMAND ${GHOSTSCRIPT_EXECUTABLE} -dNOPAUSE -dBATCH -q -dEPSCrop -dTextAlphaBits=4 -dGraphicsAlphaBits=4 -sDEVICE=pngalpha -r18 -sOutputFile=gen-images/bipartite_matching.png ${CMAKE_CURRENT_SOURCE_DIR}/images/bipartite_matching.eps
      COMMAND ${GHOSTSCRIPT_EXECUTABLE} -dNOPAUSE -dBATCH -q -dEPSCrop -dTextAlphaBits=4 -dGraphicsAlphaBits=4 -sDEVICE=pngalpha -r18 -sOutputFile=gen-images/bipartite_partitions.png ${CMAKE_CURRENT_SOURCE_DIR}/images/bipartite_partitions.eps
      COMMAND ${GHOSTSCRIPT_EXECUTABLE} -dNOPAUSE -dBATCH -q -dEPSCrop -dTextAlphaBits=4 -dGraphicsAlphaBits=4 -sDEVICE=pngalpha -r18 -sOutputFile=gen-images/connected_components.png ${CMAKE_CURRENT_SOURCE_DIR}/images/connected_components.eps
      COMMAND ${GHOSTSCRIPT_EXECUTABLE} -dNOPAUSE -dBATCH -q -dEPSCrop -dTextAlphaBits=4 -dGraphicsAlphaBits=4 -sDEVICE=pngalpha -r18 -sOutputFile=gen-images/edge_biconnected_components.png ${CMAKE_CURRENT_SOURCE_DIR}/images/edge_biconnected_components.eps
      COMMAND ${GHOSTSCRIPT_EXECUTABLE} -dNOPAUSE -dBATCH -q -dEPSCrop -dTextAlphaBits=4 -dGraphicsAlphaBits=4 -sDEVICE=pngalpha -r18 -sOutputFile=gen-images/grid_graph.png ${CMAKE_CURRENT_SOURCE_DIR}/images/grid_graph.eps
      COMMAND ${GHOSTSCRIPT_EXECUTABLE} -dNOPAUSE -dBATCH -q -dEPSCrop -dTextAlphaBits=4 -dGraphicsAlphaBits=4 -sDEVICE=pngalpha -r18 -sOutputFile=gen-images/node_biconnected_components.png ${CMAKE_CURRENT_SOURCE_DIR}/images/node_biconnected_components.eps
      COMMAND ${GHOSTSCRIPT_EXECUTABLE} -dNOPAUSE -dBATCH -q -dEPSCrop -dTextAlphaBits=4 -dGraphicsAlphaBits=4 -sDEVICE=pngalpha -r18 -sOutputFile=gen-images/nodeshape_0.png ${CMAKE_CURRENT_SOURCE_DIR}/images/nodeshape_0.eps
      COMMAND ${GHOSTSCRIPT_EXECUTABLE} -dNOPAUSE -dBATCH -q -dEPSCrop -dTextAlphaBits=4 -dGraphicsAlphaBits=4 -sDEVICE=pngalpha -r18 -sOutputFile=gen-images/nodeshape_1.png ${CMAKE_CURRENT_SOURCE_DIR}/images/nodeshape_1.eps
      COMMAND ${GHOSTSCRIPT_EXECUTABLE} -dNOPAUSE -dBATCH -q -dEPSCrop -dTextAlphaBits=4 -dGraphicsAlphaBits=4 -sDEVICE=pngalpha -r18 -sOutputFile=gen-images/nodeshape_2.png ${CMAKE_CURRENT_SOURCE_DIR}/images/nodeshape_2.eps
      COMMAND ${GHOSTSCRIPT_EXECUTABLE} -dNOPAUSE -dBATCH -q -dEPSCrop -dTextAlphaBits=4 -dGraphicsAlphaBits=4 -sDEVICE=pngalpha -r18 -sOutputFile=gen-images/nodeshape_3.png ${CMAKE_CURRENT_SOURCE_DIR}/images/nodeshape_3.eps
      COMMAND ${GHOSTSCRIPT_EXECUTABLE} -dNOPAUSE -dBATCH -q -dEPSCrop -dTextAlphaBits=4 -dGraphicsAlphaBits=4 -sDEVICE=pngalpha -r18 -sOutputFile=gen-images/nodeshape_4.png ${CMAKE_CURRENT_SOURCE_DIR}/images/nodeshape_4.eps
      COMMAND ${GHOSTSCRIPT_EXECUTABLE} -dNOPAUSE -dBATCH -q -dEPSCrop -dTextAlphaBits=4 -dGraphicsAlphaBits=4 -sDEVICE=pngalpha -r18 -sOutputFile=gen-images/strongly_connected_components.png ${CMAKE_CURRENT_SOURCE_DIR}/images/strongly_connected_components.eps
      COMMAND rm -rf html
      COMMAND ${DOXYGEN_EXECUTABLE} Doxyfile
      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  ELSEIF(WIN32)
    ADD_CUSTOM_TARGET(html
      COMMAND if exist gen-images rmdir /s /q gen-images
      COMMAND mkdir gen-images
      COMMAND ${GHOSTSCRIPT_EXECUTABLE} -dNOPAUSE -dBATCH -q -dEPSCrop -dTextAlphaBits=4 -dGraphicsAlphaBits=4 -sDEVICE=pngalpha -r18 -sOutputFile=gen-images/bipartite_matching.png ${CMAKE_CURRENT_SOURCE_DIR}/images/bipartite_matching.eps
      COMMAND ${GHOSTSCRIPT_EXECUTABLE} -dNOPAUSE -dBATCH -q -dEPSCrop -dTextAlphaBits=4 -dGraphicsAlphaBits=4 -sDEVICE=pngalpha -r18 -sOutputFile=gen-images/bipartite_partitions.png ${CMAKE_CURRENT_SOURCE_DIR}/images/bipartite_partitions.eps
      COMMAND ${GHOSTSCRIPT_EXECUTABLE} -dNOPAUSE -dBATCH -q -dEPSCrop -dTextAlphaBits=4 -dGraphicsAlphaBits=4 -sDEVICE=pngalpha -r18 -sOutputFile=gen-images/connected_components.png ${CMAKE_CURRENT_SOURCE_DIR}/images/connected_components.eps
      COMMAND ${GHOSTSCRIPT_EXECUTABLE} -dNOPAUSE -dBATCH -q -dEPSCrop -dTextAlphaBits=4 -dGraphicsAlphaBits=4 -sDEVICE=pngalpha -r18 -sOutputFile=gen-images/edge_biconnected_components.png ${CMAKE_CURRENT_SOURCE_DIR}/images/edge_biconnected_components.eps
      COMMAND ${GHOSTSCRIPT_EXECUTABLE} -dNOPAUSE -dBATCH -q -dEPSCrop -dTextAlphaBits=4 -dGraphicsAlphaBits=4 -sDEVICE=pngalpha -r18 -sOutputFile=gen-images/grid_graph.png ${CMAKE_CURRENT_SOURCE_DIR}/images/grid_graph.eps
      COMMAND ${GHOSTSCRIPT_EXECUTABLE} -dNOPAUSE -dBATCH -q -dEPSCrop -dTextAlphaBits=4 -dGraphicsAlphaBits=4 -sDEVICE=pngalpha -r18 -sOutputFile=gen-images/node_biconnected_components.png ${CMAKE_CURRENT_SOURCE_DIR}/images/node_biconnected_components.eps
      COMMAND ${GHOSTSCRIPT_EXECUTABLE} -dNOPAUSE -dBATCH -q -dEPSCrop -dTextAlphaBits=4 -dGraphicsAlphaBits=4 -sDEVICE=pngalpha -r18 -sOutputFile=gen-images/nodeshape_0.png ${CMAKE_CURRENT_SOURCE_DIR}/images/nodeshape_0.eps
      COMMAND ${GHOSTSCRIPT_EXECUTABLE} -dNOPAUSE -dBATCH -q -dEPSCrop -dTextAlphaBits=4 -dGraphicsAlphaBits=4 -sDEVICE=pngalpha -r18 -sOutputFile=gen-images/nodeshape_1.png ${CMAKE_CURRENT_SOURCE_DIR}/images/nodeshape_1.eps
      COMMAND ${GHOSTSCRIPT_EXECUTABLE} -dNOPAUSE -dBATCH -q -dEPSCrop -dTextAlphaBits=4 -dGraphicsAlphaBits=4 -sDEVICE=pngalpha -r18 -sOutputFile=gen-images/nodeshape_2.png ${CMAKE_CURRENT_SOURCE_DIR}/images/nodeshape_2.eps
      COMMAND ${GHOSTSCRIPT_EXECUTABLE} -dNOPAUSE -dBATCH -q -dEPSCrop -dTextAlphaBits=4 -dGraphicsAlphaBits=4 -sDEVICE=pngalpha -r18 -sOutputFile=gen-images/nodeshape_3.png ${CMAKE_CURRENT_SOURCE_DIR}/images/nodeshape_3.eps
      COMMAND ${GHOSTSCRIPT_EXECUTABLE} -dNOPAUSE -dBATCH -q -dEPSCrop -dTextAlphaBits=4 -dGraphicsAlphaBits=4 -sDEVICE=pngalpha -r18 -sOutputFile=gen-images/nodeshape_4.png ${CMAKE_CURRENT_SOURCE_DIR}/images/nodeshape_4.eps
      COMMAND ${GHOSTSCRIPT_EXECUTABLE} -dNOPAUSE -dBATCH -q -dEPSCrop -dTextAlphaBits=4 -dGraphicsAlphaBits=4 -sDEVICE=pngalpha -r18 -sOutputFile=gen-images/strongly_connected_components.png ${CMAKE_CURRENT_SOURCE_DIR}/images/strongly_connected_components.eps
      COMMAND if exist html rmdir /s /q html
      COMMAND ${DOXYGEN_EXECUTABLE} Doxyfile
      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  ENDIF(UNIX)
  INSTALL(
    DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html/
    DESTINATION share/doc
    COMPONENT html_documentation)
ENDIF(DOXYGEN_EXECUTABLE AND GHOSTSCRIPT_EXECUTABLE)
