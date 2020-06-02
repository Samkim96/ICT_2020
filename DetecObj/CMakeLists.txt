add_library(ICT-DetecObj DetecObj.cpp)

find_package(OpenCV)

target_link_libraries(ICT-DetecObj ${OpenCV_LIBS})
