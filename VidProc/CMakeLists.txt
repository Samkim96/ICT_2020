add_library(ICT-VidProc VidProc.cpp)

find_package(OpenCV)

target_link_libraries(ICT-VidProc ${OpenCV_LIBS})
