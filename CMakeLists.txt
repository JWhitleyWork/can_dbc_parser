cmake_minimum_required(VERSION 3.5)
project(can_dbc_parser
  VERSION 1.1.0
  LANGUAGES C CXX
)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

add_library(${PROJECT_NAME} SHARED
  src/Dbc.cpp
  src/DbcBuilder.cpp
  src/DbcMessage.cpp
  src/DbcSignal.cpp
  src/LineParser.cpp
)

target_compile_options(${PROJECT_NAME} PRIVATE -Wno-unused-function)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    # Different paths for includes for build and install phase supported
    # via INSTALL_INTERFACE and BUILD_INTERFACE generator expressions.
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

set_target_properties(${PROJECT_NAME} PROPERTIES
  VERSION ${can_dbc_parser_VERSION}
  SOVERSION ${can_dbc_parser_VERSION_MAJOR}
)

set(CMAKE_INSTALL_CMAKEDIR share/can_dbc_parser/cmake)

install(DIRECTORY include/can_dbc_parser/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT can_dbc_parserTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
  PUBLIC_HEADER DESTINATION include
)

install(EXPORT can_dbc_parserTargets
  FILE can_dbc_parserTargets.cmake
  NAMESPACE CanDbcParser::
  DESTINATION ${CMAKE_INSTALL_CMAKEDIR}
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(can_dbc_parserConfigVersion.cmake
  COMPATIBILITY AnyNewerVersion
)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/can_dbc_parserConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_CMAKEDIR}
)

install(FILES can_dbc_parserConfig.cmake
  DESTINATION ${CMAKE_INSTALL_CMAKEDIR}
)

CONFIGURE_FILE(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
  IMMEDIATE @ONLY
)

ADD_CUSTOM_TARGET(uninstall
  "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
)

