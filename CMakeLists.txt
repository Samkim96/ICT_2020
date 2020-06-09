cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIERED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "-o3")

#configure_file(ICT.names ICT.names COPYONLY)
#configure_file(yolov3-ICT.cfg yolov3-ICT.cfg COPYONLY)
#configure_file(yolov3-ICT_best.weights yolov3-ICT_best.weights COPYONLY)

project(ICT_2020)

include_directories(${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/SerialComm ${PROJECT_SOURCE_DIR}/VidProc ${PROJECT_SOURCE_DIR}/DetecObj ${PROJECT_SOURCE_DIR}/Gimbal)
include_directories(/usr/lib/aarch64-linux-gnu/gstreamer-1.0/include /usr/include/gstreamer-1.0 /usr/lib/aarch64-linux-gnu/glib-2.0/include /usr/include/glib-2.0 /usr/local/include/libserial /usr/include/i2c)

add_subdirectory(VidProc)
add_subdirectory(DetecObj)
add_subdirectory(Gimbal)

set(MAIN_LIBS ${MAIN_LIBS} ICT-VidProc ICT-DetecObj ICT-Gimbal)

link_directories(/usr/local/lib)

add_executable(ICT_2020 ICT_2020.cpp)
target_link_libraries(ICT_2020 serial i2c ${MAIN_LIBS} ${OpenCV_LIBS})
