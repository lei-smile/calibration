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


add_executable(radar_static_calib src/radar_static_calib.cpp)
target_link_libraries(radar_static_calib ${GeographicLib_LIBRARIES} ${CERES_LIBRARIES} ${OpenCV_LIBS})

add_executable(radar_static_calib_yaw src/radar_static_calib_yaw.cpp)
target_link_libraries(radar_static_calib_yaw ${GeographicLib_LIBRARIES} ${CERES_LIBRARIES} ${OpenCV_LIBS})


set(CMAKE_BUILD_TYPE Debug)
