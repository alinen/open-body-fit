set(DART_LIBS dart dart-gui boost_system assimp glut)
set(DART_DIR ~/graphics-devel/dart)
include_directories(${INCLUDE_DIRS} ${DART_DIR} ${DART_DIR}/build)
link_directories(${LIBRARY_DIRS} ${DART_DIR}/build/lib)

set(LEVMAR_LIBS levmar m dl lapack;blas f2c)
set(LEVMAR_DIR ~/graphics-devel/levmar-2.6)
include_directories(${INCLUDE_DIRS} ${LEVMAR_DIR})
link_directories(${LIBRARY_DIRS} ${LEVMAR_DIR}/build)

SET(SOURCES
  loadpoints.cpp 
  physicsskeleton.cpp 
  dartskeleton.cpp 
  unitutils.cpp 
  ik.cpp 
  anthropometrics.cpp 
  bodyfitparams.cpp
  loadpoints.h 
  physicsskeleton.h 
  dartskeleton.h 
  unitutils.h 
  ik.h 
  anthropometrics.h 
  bodyfitparams.h
)

add_executable(open-body-fit openbodyfit.cpp ${SOURCES})
target_link_libraries(open-body-fit imgui ${CORE} ${DART_LIBS} ${LEVMAR_LIBS})
