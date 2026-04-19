#----------------------------------------------------------------
# Generated CMake target import file for configuration "MinSizeRel".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "simpleble::simpleble" for configuration "MinSizeRel"
set_property(TARGET simpleble::simpleble APPEND PROPERTY IMPORTED_CONFIGURATIONS MINSIZEREL)
set_target_properties(simpleble::simpleble PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_MINSIZEREL "CXX"
  IMPORTED_LOCATION_MINSIZEREL "${_IMPORT_PREFIX}/lib/simpleble-minsizerel.lib"
  )

list(APPEND _cmake_import_check_targets simpleble::simpleble )
list(APPEND _cmake_import_check_files_for_simpleble::simpleble "${_IMPORT_PREFIX}/lib/simpleble-minsizerel.lib" )

# Import target "simpleble::simpleble-c" for configuration "MinSizeRel"
set_property(TARGET simpleble::simpleble-c APPEND PROPERTY IMPORTED_CONFIGURATIONS MINSIZEREL)
set_target_properties(simpleble::simpleble-c PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_MINSIZEREL "CXX"
  IMPORTED_LOCATION_MINSIZEREL "${_IMPORT_PREFIX}/lib/simpleble-c-minsizerel.lib"
  )

list(APPEND _cmake_import_check_targets simpleble::simpleble-c )
list(APPEND _cmake_import_check_files_for_simpleble::simpleble-c "${_IMPORT_PREFIX}/lib/simpleble-c-minsizerel.lib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
