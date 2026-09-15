
#include "basic_tracking.h"

/**
 * @function initGlobal
 */
void initGlobal(const std::string &_ee_name, const std::string &_target_name,
		const std::string &_target_actuator) {

   g_ee_body_name =  _ee_name;
   g_target_name = _target_name;
   g_target_actuator = _target_actuator;
   
   track = false;
   
   model = NULL;
   data = NULL;
   window = NULL;
   
   button_left = false;
   button_middle = false;
   button_right = false;
   
   lastx = 0;
   lasty = 0;
}
                
/**
 * @function loadModelData
 */
bool loadModelData(int argc, const char** argv) {

  if (argc < 2) {
    std::printf(" USAGE:  basic modelfile\n");
    return false;
  }

  // load and compile model
  char error[1000] = "Could not load binary model";
  /*if (std::strlen(argv[1]) > 4 && !std::strcmp(argv[1] + std::strlen(argv[1]) - 4, ".mjb")) {
    model = mj_loadModel(argv[1], 0);
  } else {
    model = mj_loadXML(argv[1], 0, error, 1000);
  }*/
  mjSpec* spec_scene = mj_parseXML(argv[1], NULL, error, sizeof(error));
  mjSpec* spec_robot = mj_parseXML(argv[2], NULL, error, sizeof(error));
  
  
//   parent->compiler.degree = 0;
//   child->compiler.degree = 1;
   mjsElement* frame = mjs_addFrame(mjs_findBody(spec_scene, "world"), NULL)->element;
   mjsElement* body = mjs_addBody(mjs_findBody(spec_robot, "world"), NULL)->element;
   mjsBody* attached_body_1 = NULL; 
   attached_body_1 = mjs_asBody(mjs_attach(frame, body, "attached-", "-suffix"));
       

  if(!attached_body_1) {
    printf("Could not attach it \n");
  } else {
    printf("Could have atached YES! \n");
  }
  
  model = mj_compile(spec_scene, NULL);
  
  if (!model) { 
    mju_error("Load model error: %s", error); 
    return false;  
  }

  // make data
  data = mj_makeData(model);
  return true;
}

/**
 * @function loadKinematics
 */
void loadKinematics() {

   g_num_dofs = 6;

  int num_act = model->nactuator;
  int num_jts = model->njnt;
  printf("Num actuators: %d and num joints: %d !!!! \n", num_act, num_jts);
}

/**
 * @function initializeViz
 */
void initializeViz() {
  
  // create window, make OpenGL context current, request v-sync
  window = glfwCreateWindow(1200, 900, "Basic Tracking", NULL, NULL);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjr_defaultContext(&con);

  // create scene and context
  mjv_makeScene(model, &scn, 2000);
  mjr_makeContext(model, &con, mjFONTSCALE_150);  
}

/**
 * @function setCallbacks
 */
void setCallbacks() {
  glfwSetKeyCallback(window, keyboard);
  glfwSetCursorPosCallback(window, mouse_move);
  glfwSetMouseButtonCallback(window, mouse_button);
  glfwSetScrollCallback(window, scroll);
}


/**
 * @function keyboard callback
 */
void keyboard(GLFWwindow* _window, int _key, int _scancode, int _act, int _mods) {
  
  double dv = 0.5;

  int target_id = mj_name2id(model, mjOBJ_ACTUATOR, g_target_actuator.c_str());

  if(target_id < 0) {
    printf("Track object id is not valid! Return \n");
    return;
  }

  // backspace: reset simulation
  if (_act == GLFW_PRESS)
  {
    switch(_key) {
      case GLFW_KEY_BACKSPACE:
      {
        mj_resetData(model, data);
        mj_forward(model, data);
      } break;
      case GLFW_KEY_0:
      {
        mjtNum pose[g_num_dofs] = {0.0, 0.0, 0.0, 0.0, 0, 0};
        for(int i = 0; i < 6; ++i)
        { 
          data->ctrl[i] = pose[i];
        }

      } break;
      case GLFW_KEY_1:
      {
        mjtNum pose[g_num_dofs] = {0.0, -1.5707, 0.0, -1.5707, 0, 0};
        for(int i = 0; i < 6; ++i)
        { 
          data->ctrl[i] = pose[i];
        }

      } break;
      case GLFW_KEY_2:
      {
        mjtNum pose[g_num_dofs] = {0.707, -1.5708, 1.5708, -1.5707, -1.5708, 0};
        for(int i = 0; i < g_num_dofs; ++i)
        { 
          data->ctrl[i] = pose[i];
        }

      } break;

      // Move box left
      case GLFW_KEY_A:
      {
        data->ctrl[target_id] = -dv;
      } break;
      case GLFW_KEY_S:
      {
        data->ctrl[target_id] = 0.0;
      } break;
      case GLFW_KEY_D:
      {
        data->ctrl[target_id] = dv;
      } break;

      case GLFW_KEY_O:
      {
        track = true;
      } break;
      case GLFW_KEY_F:
      {
        track = false;
      } break;

    }
  } // if act
}

/**
 * @function mouse_button callback
 */
void mouse_button(GLFWwindow* _window, int _button, int _act, int _mods) {
  // update button state
  button_left   = (glfwGetMouseButton(_window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
  button_middle = (glfwGetMouseButton(_window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
  button_right  = (glfwGetMouseButton(_window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

  // update mouse position
  glfwGetCursorPos(_window, &lastx, &lasty);
}

/**
 * @function mouse move callback
 */
void mouse_move(GLFWwindow* _window, double _xpos, double _ypos) {
  // no buttons down: nothing to do
  if (!button_left && !button_middle && !button_right) { return; }

  // compute mouse displacement, save
  double dx = _xpos - lastx;
  double dy = _ypos - lasty;
  lastx     = _xpos;
  lasty     = _ypos;

  // get current window size
  int width, height;
  glfwGetWindowSize(_window, &width, &height);

  // get shift key state
  bool mod_shift = (glfwGetKey(_window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                    glfwGetKey(_window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

  // determine action based on mouse button
  mjtMouse action;
  if (button_right) {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (button_left) {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  } else {
    action = mjMOUSE_ZOOM;
  }

  // move camera
  mjv_moveCamera(model, action, dx / height, dy / height, &cam);
}

/**
 * @function scroll callback
 */
void scroll(GLFWwindow* _window, double _xoffset, double _yoffset) {
  // emulate vertical mouse motion = 5% of window height
  mjv_moveCamera(model, mjMOUSE_ZOOM, 0, -0.05 * _yoffset, &cam);
}

/**
 * @function setupIKControl
 */
void setupIKControl() {
  mjcb_control = ik_control;
}


void ik_control(const mjModel* _model, mjData* _data) {

  double thresh = 0.005;
  double damping = 0.1;
  double step_size = 2.0*M_PI/180.0;


  int nv = _model->nv;

  int ee_id = mj_name2id(_model, mjOBJ_BODY, g_ee_body_name.c_str());
  if(ee_id < 1)
    return;

  Eigen::Vector3d ee_pos;
  Eigen::Vector3d box_pos;
  Eigen::Vector3d err;

  ee_pos << _data->xpos[3*ee_id], _data->xpos[3*ee_id + 1], _data->xpos[3*ee_id + 2];

  getObjectPos(_model, _data, g_target_name, box_pos);
  box_pos(2) = box_pos(2) + 0.4; // Make EE go above it

  err = (box_pos - ee_pos);

  if(!track)
    return;

  if(err.norm() >= thresh) {

    mjtNum* jacp = new mjtNum[3*nv];
    mjtNum* jacr = new mjtNum[3*nv];


    Eigen::VectorXd dq;
    Eigen::VectorXd q(g_num_dofs);
    Eigen::MatrixXd jp(3, g_num_dofs);


    mj_jac(_model, _data, jacp, jacr, box_pos.data(), ee_id);

    for(int i = 0; i < 3; ++i) {
      for(int j = 0; j < g_num_dofs; ++j) {
        jp(i, j) = jacp[nv*i+j];
      }
    }

    for(int i = 0; i < g_num_dofs; ++i) {
      q(i) = data->qpos[i];
    }

    // num_dofs * num_dofs
    Eigen::MatrixXd prod;
    Eigen::MatrixXd j_inv;
    prod = jp*jp.transpose() + damping*Eigen::MatrixXd::Identity(3, 3);

    if( fabs(prod.determinant()) < 0.0001 )
    {
      j_inv = jp.transpose() * prod.completeOrthogonalDecomposition().pseudoInverse();
    } else {
      j_inv = jp.transpose() * prod.inverse();
    }
    

    dq = j_inv * err;
    dq.normalize();
    q += step_size * dq;
    
    for(int i = 0; i < g_num_dofs; ++i)
      data->ctrl[i] = q[i];

  } // if err

} // ik_control


bool getObjectPos(const mjModel* _model, mjData* _data,
               const std::string &_name, 
               Eigen::Vector3d &_bp)
{
  int id = mj_name2id(_model, mjOBJ_BODY, _name.c_str());
  if(id < 0)
    return false;

  _bp << _data->xpos[3*id],  _data->xpos[3*id + 1], _data->xpos[3*id + 2];

  return true;
}


/**
 * @function runLoop
 */
void runLoop() {

    while (!glfwWindowShouldClose(window)) {

    mjtNum simstart = data->time;
    while (data->time - simstart < 1.0 / 60.0) { 
      mj_step(model, data); 
    }

    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(model, data, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
  }
}

/**
 * @function cleanup
 */
void cleanup() {
  // free visualization storage
  mjv_freeScene(&scn);
  mjr_freeContext(&con);

  // free MuJoCo model and data
  mj_deleteData(data);
  mj_deleteModel(model);
}


/**
 * @function main
 */
int main(int argc, const char** argv) {

  initGlobal("wrist_3_link", "red_cube", "red_cube_vel");

  if(!loadModelData(argc, argv))
    return EXIT_FAILURE;

  loadKinematics();

  // init GLFW
  if (!glfwInit()) { 
    mju_error("Could not initialize GLFW"); 
  }

  // initialize visualization data structures
  initializeViz();

  // install GLFW mouse and keyboard callbacks
  setCallbacks();

  // Initialize control
  setupIKControl();

  // run main loop
  runLoop();

  // cleanup before ending
  cleanup();

  return EXIT_SUCCESS;
}
