find_path(NLOPT_INCLUDE_DIRS
    NAMES nlopt.h
    HINTS /usr/local/inclue /usr/include /opt/)

find_library(NLOPT_LIBRARIES
    NAMES nlopt
    HINTS /usr/local/lib /usr/lib /opt)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(nlopt DEFAULT_MSG NLOPT_LIBRARIES)
