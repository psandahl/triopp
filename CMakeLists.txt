cmake_minimum_required(VERSION 3.0)

project(trio)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
add_compile_options(-Wall -Wextra -pedantic -Werror)

include_directories(lib)

find_package(OpenCV 4.4.0 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIB_DIR})

add_subdirectory(lib)
add_subdirectory(app)
add_subdirectory(test)

