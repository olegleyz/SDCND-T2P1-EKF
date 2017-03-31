cmake_minimum_required(VERSION 3.6)
project(kalman2D)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES main.cpp measurement_package.h tracking.cpp tracking.h kalman_filter.cpp kalman_filter.h Jacobian.cpp Jacobian.h rmse.cpp rmse.h)
add_executable(kalman2D ${SOURCE_FILES})