find_path(CSPARSE_INCLUDE_DIR NAMES cs.h
  PATHS
  /usr/include/suitesparse
  /usr/include
  /opt/local/include
  /usr/local/include
  /sw/include
  /usr/include/ufsparse
  /opt/local/include/ufsparse
  /usr/local/include/ufsparse
  /sw/include/ufsparse
  PATH_SUFFIXES
  suitesparse
  )

include_directories(include 
    ${CSPARSE_INCLUDE_DIR}
)

list(APPEND ALL_TARGET_LIBRARIES ${Sophus_LIBRARIES} fmt::fmt)
