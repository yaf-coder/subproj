cmake_minimum_required(VERSION 3.16)
project(bathyscaphe CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)

include(FetchContent)
FetchContent_Declare(
    Eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.4.0
    GIT_SHALLOW TRUE
)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(Eigen)

find_package(Threads REQUIRED)

add_executable(bathyscaphe
    src/main.cpp
    src/physics/dynamics.cpp
    src/physics/integrator.cpp
    src/physics/powertrain.cpp
    src/planner/planner.cpp
    src/geo/coords.cpp
    src/geo/land_mask.cpp
    src/sim/sim_manager.cpp
    src/net/server.cpp
)

target_include_directories(bathyscaphe
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party
)

target_link_libraries(bathyscaphe
    PRIVATE
        Eigen3::Eigen
        Threads::Threads
)
