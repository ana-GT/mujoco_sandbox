cmake_minimum_required(VERSION 3.16)

project(basic_tracking)

set(MUJOCO_DIR "$ENV{HOME}/local/lib/cmake/mujoco") 

set(SCENE_VAR "../../scenes/basic_tracking.xml")
set(MUJOCO_MENAGERIE_DIR_VAR "$ENV{HOME}/Software/mujoco_menagerie")


configure_file(config.h.in "${PROJECT_BINARY_DIR}/config.h" @ONLY)

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
)
target_include_directories(basic_tracking PRIVATE "${PROJECT_BINARY_DIR}")


