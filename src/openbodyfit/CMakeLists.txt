
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
