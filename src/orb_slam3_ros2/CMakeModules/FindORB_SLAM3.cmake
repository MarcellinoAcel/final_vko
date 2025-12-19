# ========= CONFIG ROOT DIR =======================================
# NOTE: DO NOT use "~" in CMake. Expand to full path.
if(NOT ORB_SLAM3_ROOT_DIR)
  set(ORB_SLAM3_ROOT_DIR "/home/marcel/ORB_SLAM3")
endif()

message(STATUS "Using ORB_SLAM3_ROOT_DIR = ${ORB_SLAM3_ROOT_DIR}")

# ========= FIND ORB_SLAM3 ========================================
find_path(ORB_SLAM3_INCLUDE_DIR
  NAMES System.h
  PATHS ${ORB_SLAM3_ROOT_DIR}/include
)

find_library(ORB_SLAM3_LIBRARY
  NAMES ORB_SLAM3 libORB_SLAM3
  PATHS ${ORB_SLAM3_ROOT_DIR}/lib
)

# ========= FIND DBoW2 =============================================
find_path(DBoW2_INCLUDE_DIR
  NAMES BowVector.h
  PATHS ${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2/DBoW2
)

find_library(DBoW2_LIBRARY
  NAMES DBoW2
  PATHS ${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2/lib
)

# ========= FIND G2O ===============================================
find_path(G2O_INCLUDE_DIR
  NAMES g2o/core/base_vertex.h
  PATHS ${ORB_SLAM3_ROOT_DIR}/Thirdparty/g2o
)

find_library(G2O_LIBRARY
  NAMES g2o
  PATHS ${ORB_SLAM3_ROOT_DIR}/Thirdparty/g2o/lib
)

# ========= CHECK ALL FOUND =======================================
include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(
  ORB_SLAM3
  DEFAULT_MSG
  ORB_SLAM3_LIBRARY
  ORB_SLAM3_INCLUDE_DIR
  DBoW2_INCLUDE_DIR
  DBoW2_LIBRARY
  G2O_INCLUDE_DIR
  G2O_LIBRARY
)

# ========= EXPORT VARIABLES =======================================
set(ORB_SLAM3_LIBRARIES
  ${ORB_SLAM3_LIBRARY}
  ${DBoW2_LIBRARY}
  ${G2O_LIBRARY}
)

set(ORB_SLAM3_INCLUDE_DIRS
  ${ORB_SLAM3_INCLUDE_DIR}
  ${DBoW2_INCLUDE_DIR}
  ${G2O_INCLUDE_DIR}
)

mark_as_advanced(
  ORB_SLAM3_INCLUDE_DIR
  ORB_SLAM3_LIBRARY
  DBoW2_INCLUDE_DIR
  DBoW2_LIBRARY
  G2O_INCLUDE_DIR
  G2O_LIBRARY
)
