
#include "basic_tracking.h"

/**
 * @function initGlobal
 */
bool initGlobal(int argc, const char** argv,
                const std::string &_target_name,
		const std::string &_target_actuator_x,
		const std::string &_target_actuator_y) {
   
   std::string robot_opt;
   if(argc < 2) {
     printf("Error, needs an argument, panda or ur10 \n");
     return false;
   }

   robot_opt = argv[1];
   if(robot_opt == "panda")
   {
      g_robot_filename = std::string(MENAGERIE_DIR) + std::string("/franka_emika_panda/panda.xml"); 
      g_ee_body_name = "link7";
   } else if(robot_opt == "ur10") {
     g_ee_body_name = "wrist_3_link";
     g_robot_filename = std::string(MENAGERIE_DIR) + std::string("/universal_robots_ur10e/ur10e.xml");
   } else {
     printf("No model available \n");
     return false;
   }

   g_target_name = _target_name;
   g_target_actuator_x = _target_actuator_x;
   g_target_actuator_y = _target_actuator_y;   
   
   track = false;
   
   model = NULL;
   data = NULL;
   window = NULL;
   
   button_left = false;
   button_middle = false;
   button_right = false;
   
   lastx = 0;
   lasty = 0;
   return true;
}
                
/**
 * @function loadModelData
 */
bool loadModelData() {


  // load and compile model
  char error[1000] = "Could not load binary model";

  mjSpec *spec_scene, *spec_robot;
  
  spec_scene = mj_parseXML(SCENE_FILENAME, NULL, error, sizeof(error));
  spec_robot = mj_parseXML(g_robot_filename.c_str(), NULL, error, sizeof(error));
  
  if(!spec_scene || !spec_robot) {
    printf("Error loading either \n\t * scene: %s or \n\t * robot: %s \n", SCENE_FILENAME, g_robot_filename.c_str());
    return false;
  }  

   mjsElement* frame = NULL;
   mjsElement* robot_root = NULL;
      
   frame = mjs_addFrame(mjs_findBody(spec_scene, "world"), NULL)->element;   
   
   mjsElement* rb = mjs_firstElement(spec_robot, mjOBJ_BODY);   
   robot_root = mjs_firstChild(mjs_asBody(rb), mjOBJ_BODY, 0);

   
   mjsBody* attached_body_1 = NULL; 
   attached_body_1 = mjs_asBody(mjs_attach(frame, robot_root, "", ""));


  if(!attached_body_1) {
    printf("Could not attach it \n");
  } else {
    printf("Could have atached YES! \n");
  }
  
  model = mj_compile(spec_scene, NULL);
  
  if (!model) { 
    mju_error("Load model error: %s", mjs_getError(spec_scene)); 
    return false;  
  }


  // make data
  data = mj_makeData(model);

  loadKinematics();

  return true;
}

/**
 * @function loadKinematics
 */
void loadKinematics() {


   for(int i = 0; i < model->nbody; ++i) {
     const char* ni = mj_id2name(model, mjOBJ_BODY, i);
   }

   std::string chain_body, last_body;
   int chain_id, last_id;

   last_body = g_ee_body_name;   
   last_id = mj_name2id(model, mjOBJ_BODY, last_body.c_str());

   std::vector<std::pair<int, std::string>> chain;
   
   do {
     chain.push_back( {last_id, last_body} );
     
     chain_id = model->body_parentid[last_id];
     chain_body = mj_id2name(model, mjOBJ_BODY, chain_id);

     last_id = chain_id;
     last_body = chain_body;
   } while(last_id > 0);
   
   // Get joints
   g_num_dofs = 0;

   std::pair<int, std::string> base;
   for(int i = 0; i <= chain.size() - 1; ++i) {
      
      if(model->body_jntnum[chain[i].first] > 0) {
        g_num_dofs += 1;
        base = chain[i];
      }
   }

  // joint of fixed_link -> DOF -> base
  int jt_id = model->body_jntadr[base.first];

   
  printf("BASE BODY: %s id: %d. Body jnt addr: %d dof adr: %d, qpos adr: %d \n", 
         base.second.c_str(), base.first, 
         model->body_jntadr[base.first], model->body_dofadr[base.first], 
         model->jnt_qposadr[jt_id]);
  
  
  g_start_v = model->body_dofadr[base.first]; // dofs (e.g. 6 for the first pos/rot of cube)
  g_start_q = model->jnt_qposadr[jt_id]; // 7, global coordinates
  
  for(int i = 0; i < model->nactuator;  ++i) {
    if(model->actuator_trnid[2*i] == jt_id) {
      g_start_u = i; // actuator (e.g. 2, after vel_x, vel_y for cube);
      break;
    }
  }
    
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

  int target_id_x = mj_name2id(model, mjOBJ_ACTUATOR, g_target_actuator_x.c_str());
  int target_id_y = mj_name2id(model, mjOBJ_ACTUATOR, g_target_actuator_y.c_str());
  
  if(target_id_x < 0 || target_id_y < 0) {
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
          data->ctrl[g_start_u + i] = pose[i];
        }

      } break;
      case GLFW_KEY_1:
      {
        mjtNum pose[g_num_dofs] = {0.0, -1.5707, 0.0, -1.5707, 0, 0};
        for(int i = 0; i < 6; ++i)
        { 
          data->ctrl[g_start_u + i] = pose[i];
        }

      } break;
      case GLFW_KEY_2:
      {
        mjtNum pose[g_num_dofs] = {0.707, -1.5708, 1.5708, -1.5707, -1.5708, 0};
        for(int i = 0; i < g_num_dofs; ++i)
        { 
          data->ctrl[g_start_u + i] = pose[i];
        }

      } break;

      // Move box left
      case GLFW_KEY_A:
      {
        data->ctrl[target_id_x] = -dv;
      } break;
      case GLFW_KEY_S:
      {
        data->ctrl[target_id_x] = 0.0;
        data->ctrl[target_id_y] = 0.0;
      } break;
      case GLFW_KEY_D:
      {
        data->ctrl[target_id_x] = dv;
      } break;

      case GLFW_KEY_W:
      {
        data->ctrl[target_id_y] = -dv;
      } break;
      
      case GLFW_KEY_X:
      {
        data->ctrl[target_id_y] = dv;
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
  int nu = _model->nu;
  int nq = _model->nq;

  int ee_id = mj_name2id(_model, mjOBJ_BODY, g_ee_body_name.c_str());
  if(ee_id < 1)
    return;

  Eigen::Vector3d ee_pos;
  Eigen::Vector3d box_pos;
  Eigen::Vector3d err;

  getObjectPos(_model, _data, g_ee_body_name, ee_pos);
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
        jp(i, j) = jacp[nv*i + g_start_v + j];
      }
    }

    for(int i = 0; i < g_num_dofs; ++i) {
      q(i) = data->qpos[g_start_q + i];
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
      data->ctrl[g_start_u + i] = q[i];

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

  if(!initGlobal(argc, argv, "red_cube", "red_cube_vel_x", "red_cube_vel_y"))
    return false;

  if(!loadModelData())
    return EXIT_FAILURE;

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
