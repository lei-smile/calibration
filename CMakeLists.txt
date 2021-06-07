cmake_minimum_required(VERSION 3.1.0)
project(sensor_calib)

find_package (GeographicLib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include
                    ${CERES_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
)



add_executable(cam_calib src/cam2RTK_calib.cpp)
target_link_libraries(cam_calib ${GeographicLib_LIBRARIES} ${OpenCV_LIBS})

add_executable(check_cam_calib src/check_cam_calib.cpp)
target_link_libraries(check_cam_calib ${GeographicLib_LIBRARIES} ${OpenCV_LIBS})



set(CMAKE_BUILD_TYPE Debug)
