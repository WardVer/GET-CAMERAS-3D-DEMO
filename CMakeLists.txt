cmake_minimum_required(VERSION 3.8)

project(FEB_Cone_detection_2020 VERSION 0.0.1)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
IF(CMAKE_COMPILER_IS_GNUCC)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
    set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")
ENDIF(CMAKE_COMPILER_IS_GNUCC)

find_package( OpenCV 4 REQUIRED )
link_directories(usr/lib)
include_directories(./inc /home/ward/cameras/Galaxy_Linux-x86_Gige-U3_32bits-64bits_1.2.1911.9122(2)/Galaxy_Linux-x86_Gige-U3_32bits-64bits_1.2.1911.9122/Galaxy_camera/inc)

add_library(
    3d
    ./src/3d_functions.cpp
    ./inc/3d_functions.hpp
)

add_library(
    cameras
    ./src/cameras.cpp
    ./inc/cameras.hpp
)

add_executable(demo ./src/demo.cpp)
add_executable(calibrate ./src/calibrate.cpp)
add_executable(calibrate_pics ./src/calibrate_pics.cpp)

Target_link_libraries(3d ${OpenCV_LIBS})
Target_link_libraries(calibrate ${OpenCV_LIBS})
Target_link_libraries(cameras gxiapi)
Target_link_libraries(calibrate_pics ${OpenCV_LIBS} gxiapi cameras)
Target_link_libraries(demo ${OpenCV_LIBS} 3d cameras)
