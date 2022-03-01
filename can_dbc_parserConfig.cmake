list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})

# If find_package called with REQUIRED or QUIET we need to
# forward down it
unset(extraArgs)
if(${CMAKE_FIND_PACKAGE_NAME}_FIND_QUIETLY)
  list(APPEND extraArgs QUIET)
endif()
if(${CMAKE_FIND_PACKAGE_NAME}_FIND_REQUIRED)
  list(APPEND extraArgs REQUIRED)
endif()

include("${CMAKE_CURRENT_LIST_DIR}/can_dbc_parserTargets.cmake")

set(can_dbc_parser_FOUND TRUE)
