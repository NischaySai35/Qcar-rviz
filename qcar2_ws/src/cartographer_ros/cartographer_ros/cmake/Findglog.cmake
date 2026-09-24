# FindGlog.cmake
# Find the Google Glog logging library
#
# This module defines:
#  GLOG_INCLUDE_DIRS - where to find logging.h
#  GLOG_LIBRARIES    - libraries to link against to use glog.
#  GLOG_FOUND        - True if glog found.

find_package(PkgConfig QUIET)
if(PKG_CONFIG_FOUND)
  pkg_check_modules(PC_GLOG QUIET libglog)
endif()

find_path(GLOG_INCLUDE_DIR
  NAMES glog/logging.h
  HINTS ${PC_GLOG_INCLUDE_DIRS}
  PATHS /usr/include /usr/local/include
)

find_library(GLOG_LIBRARY
  NAMES glog
  HINTS ${PC_GLOG_LIBRARY_DIRS}
  PATHS /usr/lib /usr/local/lib /usr/lib/aarch64-linux-gnu
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(glog
  REQUIRED_VARS GLOG_LIBRARY GLOG_INCLUDE_DIR
)

if(GLOG_FOUND)
  set(GLOG_LIBRARIES ${GLOG_LIBRARY})
  set(GLOG_INCLUDE_DIRS ${GLOG_INCLUDE_DIR})

  if(NOT TARGET glog::glog)
    add_library(glog::glog UNKNOWN IMPORTED)
    set_target_properties(glog::glog PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES "${GLOG_INCLUDE_DIRS}"
      IMPORTED_LOCATION "${GLOG_LIBRARIES}"
    )
  endif()
endif()

mark_as_advanced(GLOG_INCLUDE_DIR GLOG_LIBRARY)
