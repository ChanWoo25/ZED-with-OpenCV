CMAKE_MINIMUM_REQUIRED(VERSION 2.4)
PROJECT(ZED)

option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)

if (NOT LINK_SHARED_ZED AND MSVC)
    message(FATAL_ERROR "LINK_SHARED_ZED OFF : ZED SDK static libraries not available on Windows")
endif()

SET(EXECUTABLE_OUTPUT_PATH ".")

find_package(ZED 3 REQUIRED)
find_package(OpenCV 3.4.11 REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)

include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

link_directories(${ZED_LIBRARY_DIR})
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})

add_definitions(-std=c++14 -O3)

if (LINK_SHARED_ZED)
    SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})
else()
    SET(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY})
endif()

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

add_executable(main main.cpp src/calib_zed.cpp include/calib_zed.hpp)

target_include_directories(main 
PUBLIC 
    "${CMAKE_HOME_DIRECTORY}/include"
    "${OpenCV_INCLUDE_DIRS}"
    "${ZED_INCLUDE_DIRS}"
    "${CUDA_INCLUDE_DIRS}"
)
target_link_libraries(main 
PUBLIC 
    "${OpenCV_LIBRARIES}"
    "${ZED_LIBS}"
)