cmake_minimum_required(VERSION 3.16)

project(basic_tracking)

set(MUJOCO_DIR "$ENV{HOME}/local/lib/cmake/mujoco") 
set(SCENE_VAR "/home/ana/Research/robot_industrial/mujoco_sandbox/scenes/basic_tracking.xml")

#set(ROBOT_VAR "/home/ana/Software/mujoco_menagerie/universal_robots_ur10e/ur10e.xml")
#set(ROBOT_EE_VAR  "wrist_3_link")

set(ROBOT_VAR "/home/ana/Software/mujoco_menagerie/franka_emika_panda/panda.xml")
set(ROBOT_EE_VAR  "link7")

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
#  Threads::Threads
)
target_include_directories(basic_tracking PRIVATE "${PROJECT_BINARY_DIR}")
#target_link_options(basic PRIVATE ${MUJOCO_SAMPLE_LINK_OPTIONS})


