#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ruka::libcxxcanard" for configuration ""
set_property(TARGET ruka::libcxxcanard APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(ruka::libcxxcanard PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "C;CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/liblibcxxcanard.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS ruka::libcxxcanard )
list(APPEND _IMPORT_CHECK_FILES_FOR_ruka::libcxxcanard "${_IMPORT_PREFIX}/lib/liblibcxxcanard.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
