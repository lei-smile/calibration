cmake_minimum_required(VERSION 3.5)
project(lidar2cam_calib)

#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(OpenCV REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include 
                    ${OpenCV_INCLUDE_DIRS}/include
                    /usr/include)

link_directories(${PROJECT_SOURCE_DIR}/lib
                 /usr/lib)

add_executable(lidar_repro src/lidar_reproject.cpp)
target_link_libraries(lidar_repro ${OpenCV_LIBS})

set(CMAKE_BUILD_TYPE Release)
