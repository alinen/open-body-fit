project(open-body-fit)
cmake_minimum_required(VERSION 3.14.0)

if (WIN32) # Include win64 platforms

  find_package(OpenGL REQUIRED)
  find_library(GLEW NAMES glew32s PATHS src/agl/external/lib/x64)
  find_library(GLFW NAMES glfw3 PATHS src/agl/external/lib)

  set(INCLUDE_DIRS
     .
     src/atk
     src/agl
     src/agl/agl
     src/agl/agl/mesh/
     src/agl/external/include )

  set(LIBRARY_DIRS
    src/atk/atk
    src/agl/agl
    src/agl/external/lib)

  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_FLAGS 
     "/wd4018 /wd4244 /wd4305 
     /D_CRT_SECURE_NO_WARNINGS 
     /D_CRT_NONSTDC_NO_DEPRECATE 
     /D NOMINMAX /DGLEW_STATIC
     /EHsc")

  set(VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
  set(CMAKE_EXE_LINKER_FLAGS "/NODEFAULTLIB:\"MSVCRT\" /NODEFAULTLIB:\"LIBCMT\"")

  set(CORE atk agl ${GLEW} ${GLFW} opengl32.lib)
  set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)
  set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)

elseif (APPLE)

  set(CMAKE_MACOSX_RPATH 1)
  set(CMAKE_CXX_FLAGS "-Wall -Wno-deprecated-declarations -Wno-reorder-ctor -Wno-unused-function -Wno-unused-variable -g -stdlib=libc++ -std=c++14")
  set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)
  set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)
  find_library(GL_LIB OpenGL)
  find_library(GLFW glfw)
  add_definitions(-DAPPLE)

  set(INCLUDE_DIRS
    /System/Library/Frameworks
    .
    src/atk
    src/agl
    src/agl/agl
    src/agl/external/include
    /usr/local/include)

  set(CORE atk agl ${GLFW} ${GL_LIB})

elseif (UNIX)

  set(CMAKE_CXX_FLAGS "-Wall -g -std=c++14 -Wno-comment -Wno-sign-compare -Wno-reorder -Wno-unused-function")
  set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)
  set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)

  FIND_PACKAGE(OpenGL REQUIRED) 
  FIND_PACKAGE(GLEW REQUIRED)

  set(INCLUDE_DIRS
    .
    src
    src/atk
    src/agl
    src/agl/agl
    src/external/dart
    src/external/dart/build
    src/levmar-2.6
    src/external
    src/external/include)

  set(LIBRARY_DIRS
    /usr/X11R6/lib
    /usr/local/lib
    src
    src/external/dart/build/lib
    src/levmar-2.6/build
    src/external/lib
    src/atk/atk
    src/agl/agl)

  add_definitions(-DUNIX)
  set(DART_LIBS dart assimp)
  set(LEVMAR_LIBS levmar m dl lapack;blas f2c)
  set(CORE atk agl GLEW glfw GL X11 ${DART_LIBS} ${LEVMAR_LIBS})

endif()

include_directories(${INCLUDE_DIRS})
link_directories(${LIBRARY_DIRS})
add_subdirectory(src)
