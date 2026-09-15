cmake_minimum_required(VERSION 3.16)

project(basic_tracking)

set(MUJOCO_DIR "$ENV{HOME}/local/lib/cmake/mujoco") 
list(APPEND CMAKE_PREFIX_PATH "${MUJOCO_DIR}")

find_package(mujoco REQUIRED)
find_package(glfw3 REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

add_executable(basic_tracking basic_tracking.cc)
target_link_libraries(
  basic_tracking
  mujoco::mujoco
  glfw
  Eigen3::Eigen
#  Threads::Threads
)
#target_link_options(basic PRIVATE ${MUJOCO_SAMPLE_LINK_OPTIONS})


