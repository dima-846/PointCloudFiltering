cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(filtration)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(filtration filtration.cpp)
target_link_libraries(filtration ${PCL_LIBRARIES} ${OpenCV_LIBS})
