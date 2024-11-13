cmake_minimum_required (VERSION 2.8.12)

# Project name

project (calib_cg_param)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

set (Ceres_DIR "")

find_package (OpenCV)
find_package (glog REQUIRED)
find_package (Eigen3 REQUIRED)
find_package(Ceres REQUIRED)


include_directories(
    ./
    ./include
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)


add_executable( opt_ceres src/opt_ceres.cpp )
target_link_libraries( opt_ceres ${OpenCV_LIBS} ${CERES_LIBRARIES} glog)