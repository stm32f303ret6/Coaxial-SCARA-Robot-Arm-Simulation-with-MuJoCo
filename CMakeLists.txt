cmake_minimum_required(VERSION 3.12)

project(q4)
set(CMAKE_CXX_STANDARD 17)

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -std=c++17)
# Add the path to the MuJoCo include directory
#include_directories(/home/rsc/Downloads/mujoco/mujoco-2.3.3/include/mujoco)
#include_directories(/home/rsc/Downloads/mujoco/mujoco-2.3.3/include)
include_directories(/home/rsc/Downloads/mujoco_332/mujoco-3.3.2/include)
include_directories(/home/rsc/Downloads/mujoco_332/mujoco-3.3.2/include/mujoco)
link_directories(/home/rsc/Downloads/mujoco_332/mujoco-3.3.2/bin)
include_directories(include)



find_package(GLEW REQUIRED)
find_package(Eigen3 REQUIRED)


set(SOURCES
    main.cc
    mujoco_viz.c
)

set(SOURCES ${SOURCES} mujoco_viz.c)


include_directories(${GLEW_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
link_libraries(${GLEW_LIBRARIES})

# Add your source files
add_executable(main src/main.cpp)

# Link against the MuJoCo libraries
target_link_libraries(main -lmujoco -lglfw -lGL )
