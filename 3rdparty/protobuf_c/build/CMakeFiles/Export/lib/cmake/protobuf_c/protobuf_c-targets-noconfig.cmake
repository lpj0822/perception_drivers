#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "protobuf_c" for configuration ""
set_property(TARGET protobuf_c APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(protobuf_c PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "C"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libprotobuf_c.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS protobuf_c )
list(APPEND _IMPORT_CHECK_FILES_FOR_protobuf_c "${_IMPORT_PREFIX}/lib/libprotobuf_c.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
