cmake_minimum_required(VERSION 3.16)
project(hwil_exec CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# JSBSim: point JSBSIM_ROOT at an install prefix (cmake -DJSBSIM_ROOT=...),
# or install it system-wide (/usr/local). See docs/setup.md.
set(JSBSIM_ROOT "" CACHE PATH "JSBSim install prefix")
find_path(JSBSIM_INCLUDE_DIR FGFDMExec.h
  HINTS ${JSBSIM_ROOT}/include/JSBSim /usr/local/include/JSBSim /usr/include/JSBSim)
find_library(JSBSIM_LIBRARY NAMES JSBSim
  HINTS ${JSBSIM_ROOT}/lib ${JSBSIM_ROOT}/lib64 /usr/local/lib /usr/lib)
if(NOT JSBSIM_INCLUDE_DIR OR NOT JSBSIM_LIBRARY)
  message(FATAL_ERROR "JSBSim not found. Build and install it, then pass -DJSBSIM_ROOT=<prefix>.")
endif()
message(STATUS "JSBSim headers: ${JSBSIM_INCLUDE_DIR}")
message(STATUS "JSBSim library: ${JSBSIM_LIBRARY}")

find_package(Threads REQUIRED)

add_executable(hwil_exec
  src/main.cpp
  src/config.cpp
  src/fdm.cpp
  src/mission.cpp
  src/controller.cpp)
target_include_directories(hwil_exec PRIVATE ${JSBSIM_INCLUDE_DIR} src)
target_link_libraries(hwil_exec PRIVATE ${JSBSIM_LIBRARY} Threads::Threads)
target_compile_options(hwil_exec PRIVATE -Wall -Wextra -O2)
