cmake_minimum_required(VERSION 3.2)

project(ZED_with_OpenCV
VERSION         1.0
LANGUAGES       CXX
DESCRIPTION     "A set of programs that use ZED camera with OpenCV's powerful algorithms"
)

#=============================ZED DEFAULT CMAKE CONFIGURATION=============================
option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)

if (NOT LINK_SHARED_ZED AND MSVC)
    message(FATAL_ERROR "LINK_SHARED_ZED OFF : ZED SDK static libraries not available on Windows")
endif()


if (NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
SET(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

SET(EXECUTABLE_OUTPUT_PATH ".")

find_package(ZED 3 REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)

if (LINK_SHARED_ZED)
    SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})
else()
    SET(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY})
endif()

include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})

#=============================ZED DEFAULT CMAKE CONFIGURATION=============================
#=========================================================================================

### OpenCV
find_package(OpenCV 3.4.11 QUIET)
message(STATUS "OpenCV version : ${OpenCV_VERSION}")
message(STATUS "OpenCV libraries : ${OpenCV_LIBRARIES}")
message(STATUS "OpenCV include directories : ${OpenCV_INCLUDE_DIRS}")
###

add_executable(test main.cpp)
add_definitions(-std=c++14 -O3)
target_link_libraries(test ${ZED_LIBS} ${OpenCV_LIBRARIES})
target_include_directories(test PUBLIC ${OpenCV_INCLUDE_DIRS})
