#pragma once

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <Eigen/Dense>
#include <iostream>

// **********************
// Global variables
// **********************
std::string g_ee_body_name;
std::string g_target_name;
std::string g_target_actuator;
int g_num_dofs;
bool track;

// MuJoCo data structures
mjModel*   model;
mjData*    data;
mjvCamera  cam;
mjvOption  opt;
mjvScene   scn;
mjrContext con;

// Visualization
GLFWwindow* window;

// mouse interaction
bool   button_left;
bool   button_middle;
bool   button_right;
double lastx;
double lasty;


// Function declaration
void initGlobal(const std::string &_ee_name, const std::string &_target_name,  const std::string &_target_actuator);
bool loadModelData(int argc, const char** argv);
void loadKinematics();
void initializeViz();
void setupIKControl();

void setCallbacks();
void keyboard(GLFWwindow* _window, int _key, int _scancode, int _act, int _mods);
void mouse_button(GLFWwindow* _window, int _button, int _act, int _mods);
void mouse_move(GLFWwindow* _window, double _xpos, double _ypos);
void scroll(GLFWwindow* _window, double _xoffset, double _yoffset);

void setupIKControl(); 
void ik_control(const mjModel* _model, mjData* _data);
bool getObjectPos(const mjModel* _model, mjData* _data,
               const std::string &_name, 
               Eigen::Vector3d &_bp);

