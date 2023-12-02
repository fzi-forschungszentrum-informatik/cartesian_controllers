// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    mujoco_simulator.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2022/09/27
 *
 */
//-----------------------------------------------------------------------------


#include "rackki_learning/mujoco_simulator.h"
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"
#include "geometry_msgs/msg/detail/twist_stamped__struct.hpp"
#include <filesystem>
#include <memory>
#include <random>
#include <vector>

namespace rackki_learning {

MuJoCoSimulator::MuJoCoSimulator() {}

void MuJoCoSimulator::targetWrenchCallback(
  const geometry_msgs::msg::WrenchStamped::SharedPtr wrench)
{
  // Synchronize target wrenches with state feedback.
  if (command_mutex.try_lock())
  {
    m_target_wrench[0] = wrench->wrench.force.x;
    m_target_wrench[1] = wrench->wrench.force.y;
    m_target_wrench[2] = wrench->wrench.force.z;
    m_target_wrench[3] = wrench->wrench.torque.x;
    m_target_wrench[4] = wrench->wrench.torque.y;
    m_target_wrench[5] = wrench->wrench.torque.z;

    // Indices into the data vectors
    int p = m_active_body * 3; // position [x,y,z]
    int r = m_active_body * 4; // rotation quaternion [w,x,y,z]
    int v = m_active_body * 6; // velocity [rot, lin]

    // Object pose w.r.t. the assembly goal.
    auto pose_msg               = geometry_msgs::msg::PoseStamped();
    pose_msg.header.frame_id    = "world";
    pose_msg.header.stamp       = m_node->now();
    pose_msg.pose.position.x    = d->xpos[p + 0];
    pose_msg.pose.position.y    = d->xpos[p + 1];
    pose_msg.pose.position.z    = d->xpos[p + 2];
    pose_msg.pose.orientation.w = d->xquat[r + 0];
    pose_msg.pose.orientation.x = d->xquat[r + 1];
    pose_msg.pose.orientation.y = d->xquat[r + 2];
    pose_msg.pose.orientation.z = d->xquat[r + 3];
    m_feedback_pose_publisher->publish(pose_msg);

    // Object twist w.r.t. the assembly goal.
    auto twist_msg            = geometry_msgs::msg::TwistStamped();
    twist_msg.header.frame_id = "world";
    twist_msg.header.stamp    = m_node->now();
    twist_msg.twist.angular.x = d->cvel[v + 0];
    twist_msg.twist.angular.y = d->cvel[v + 1];
    twist_msg.twist.angular.z = d->cvel[v + 2];
    twist_msg.twist.linear.x  = d->cvel[v + 3];
    twist_msg.twist.linear.y  = d->cvel[v + 4];
    twist_msg.twist.linear.z  = d->cvel[v + 5];
    m_feedback_twist_publisher->publish(twist_msg);

    command_mutex.unlock();
  }
}

void MuJoCoSimulator::keyboardCB(GLFWwindow* window, int key, int scancode, int act, int mods)
{
  getInstance().keyboardCBImpl(window, key, scancode, act, mods);
}

void MuJoCoSimulator::keyboardCBImpl([[maybe_unused]] GLFWwindow* window,
                                     int key,
                                     [[maybe_unused]] int scancode,
                                     int act,
                                     [[maybe_unused]] int mods)
{
  // backspace: reset simulation
  if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
  {
    mj_resetData(m, d);
    if (!m_starting_poses.empty())
    {
      auto device  = std::random_device();
      auto random  = std::default_random_engine(device());
      auto indices = std::uniform_int_distribution<std::size_t>(0, m_starting_poses.size() - 1);
      auto idx     = indices(random);
      auto pose    = m_starting_poses[idx];
      mju_copy(d->qpos, pose.data(), 7);
    }
    else
    {
      mju_copy(d->qpos, m->key_qpos, 7); // initial states from xml
    }
    command_mutex.lock();
    m_target_wrench = {0, 0, 0, 0, 0, 0};
    command_mutex.unlock();
    mj_forward(m, d);
  }

  // GLFW_KEY_SPACE: record current pose
  if (act == GLFW_PRESS && key == GLFW_KEY_SPACE)
  {
    std::array<double, 7> pose;
    mju_copy(pose.data(), d->qpos, 7);
    m_starting_poses.emplace_back(pose);
  }
}


// mouse button callback
void MuJoCoSimulator::mouseButtonCB(GLFWwindow* window, int button, int act, int mods)
{
  getInstance().mouseButtonCBImpl(window, button, act, mods);
}

void MuJoCoSimulator::mouseButtonCBImpl(GLFWwindow* window,
                                        [[maybe_unused]] int button,
                                        [[maybe_unused]] int act,
                                        [[maybe_unused]] int mods)
{
  // update button state
  button_left   = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
  button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
  button_right  = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

  // update mouse position
  glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void MuJoCoSimulator::mouseMoveCB(GLFWwindow* window, double xpos, double ypos)
{
  getInstance().mouseMoveCBImpl(window, xpos, ypos);
}

void MuJoCoSimulator::mouseMoveCBImpl(GLFWwindow* window, double xpos, double ypos)
{
  // no buttons down: nothing to do
  if (!button_left && !button_middle && !button_right)
  {
    return;
  }

  // compute mouse displacement, save
  double dx = xpos - lastx;
  double dy = ypos - lasty;
  lastx     = xpos;
  lasty     = ypos;

  // get current window size
  int width, height;
  glfwGetWindowSize(window, &width, &height);

  // get shift key state
  bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                    glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

  // determine action based on mouse button
  mjtMouse action;
  if (button_right)
  {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  }
  else if (button_left)
  {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  }
  else
  {
    action = mjMOUSE_ZOOM;
  }

  // move camera
  mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}


// scroll callback
void MuJoCoSimulator::scrollCB(GLFWwindow* window, double xoffset, double yoffset)
{
  getInstance().scrollCBImpl(window, xoffset, yoffset);
}

void MuJoCoSimulator::scrollCBImpl([[maybe_unused]] GLFWwindow* window,
                                   [[maybe_unused]] double xoffset,
                                   double yoffset)
{
  // emulate vertical mouse motion = 5% of window height
  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

void MuJoCoSimulator::controlCB(const mjModel* m, mjData* d)
{
  getInstance().controlCBImpl(m, d);
}

void MuJoCoSimulator::controlCBImpl(const mjModel* m, mjData* d)
{
  command_mutex.lock();

  constexpr size_t cart_dim = 6; // 3 translation, 3 rotation

  for (size_t i = 0; i < cart_dim; ++i)
  {
    // Damp each of the free joint's dimensions individually with a
    // user-specified value.
    m->dof_damping[i] = m->jnt_user[i];
  }

  // Apply external force-torque vector.
  for (size_t i = 0; i < m_target_wrench.size(); ++i)
  {
    constexpr int xfrc_dim                        = 6;
    d->xfrc_applied[m_active_body * xfrc_dim + i] = m_target_wrench[i];
  }

  command_mutex.unlock();
}

int MuJoCoSimulator::simulate()
{
  return getInstance().simulateImpl();
}

int MuJoCoSimulator::simulateImpl()
{
  mj_loadAllPluginLibraries(MUJOCO_PLUGIN_DIR, nullptr);

  // Initialize ROS2 node
  m_node = std::make_shared<rclcpp::Node>("simulator",
                                          rclcpp::NodeOptions().allow_undeclared_parameters(true));

  // Fetch parameters directly from launch file
  auto model_xml = m_node->declare_parameter<std::string>("mujoco_model");
  auto mesh_dir  = m_node->declare_parameter<std::string>("mesh_directory");

  // Load mesh files into a virtual file system.
  // MuJoCo's xml compiler will look there first when creating the model.
  auto mj_vfs = std::make_unique<mjVFS>();
  mj_defaultVFS(mj_vfs.get());

  for (const auto& entry : std::filesystem::directory_iterator(mesh_dir))
  {
    mj_addFileVFS(mj_vfs.get(),
                  // Append a forward slash for MuJoCo if non-existent
                  (std::string(mesh_dir).back() == '/') ? mesh_dir.c_str()
                                                        : std::string(mesh_dir + '/').c_str(),
                  entry.path().filename().c_str());
  }

  // Load and compile model
  char error[1000] = "Could not load binary model";
  m                = mj_loadXML(model_xml.c_str(), mj_vfs.get(), error, 1000);
  if (!m)
  {
    mju_error_s("Load model error: %s", error);
    return 1;
  }
  d = mj_makeData(m);

  // Set initial state with the keyframe mechanism from xml
  mju_copy(d->qpos, m->key_qpos, m->nq);

  // Identify the active assembly component.
  std::string convention = "active_component";
  for (int i = 0; i < m->nbody; ++i)
  {
    auto name = std::string(&m->names[m->name_bodyadr[i]]);
    if (name == convention)
    {
      m_active_body = i;
    }
  }
  if (!m_active_body)
  {
    mju_error_s("Error in model xml: We need a body named %s", convention.c_str());
    return 2;
  }

  // ROS2 communication
  m_feedback_pose_publisher =
    m_node->create_publisher<geometry_msgs::msg::PoseStamped>("/current_pose", 3);

  m_feedback_twist_publisher =
    m_node->create_publisher<geometry_msgs::msg::TwistStamped>("/current_twist", 3);

  m_target_wrench_subscriber = m_node->create_subscription<geometry_msgs::msg::WrenchStamped>(
    "/target_wrench",
    3,
    std::bind(&MuJoCoSimulator::targetWrenchCallback, this, std::placeholders::_1));

  m_reset_server = m_node->create_service<std_srvs::srv::Trigger>(
    std::string(m_node->get_name()) + "/reset",
    [this]([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
           std::shared_ptr<std_srvs::srv::Trigger::Response> response) -> void {
      m_reset_simulation = true;
      response->success  = true;
      response->message  = "success";
    });

  m_ready = true;

  // init GLFW
  if (!glfwInit())
  {
    mju_error("Could not initialize GLFW");
  }

  // create window, make OpenGL context current, request v-sync
  GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // initialize visualization data structures
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjr_defaultContext(&con);

  // create scene and context
  mjv_makeScene(m, &scn, 2000);
  mjr_makeContext(m, &con, mjFONTSCALE_150);

  // install GLFW mouse and keyboard callbacks
  glfwSetKeyCallback(window, keyboardCB);
  glfwSetMouseButtonCallback(window, mouseButtonCB);
  glfwSetCursorPosCallback(window, mouseMoveCB);
  glfwSetScrollCallback(window, scrollCB);

  // Connect our specific control input callback for MuJoCo's engine.
  mjcb_control = MuJoCoSimulator::controlCB;

  // run main loop, target real-time simulation and 60 fps rendering
  while (!glfwWindowShouldClose(window))
  {
    // advance interactive simulation for 1/60 sec
    //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
    //  this loop will finish on time for the next frame to be rendered at 60 fps.
    //  Otherwise add a cpu timer and exit this loop when it is time to render.
    mjtNum simstart = d->time;
    while (d->time - simstart < 1.0 / 60.0)
    {
      mj_step(m, d);
    }

    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();

    if (m_reset_simulation)
    {
      getInstance().keyboardCBImpl(nullptr, GLFW_KEY_BACKSPACE, 0, GLFW_PRESS, 0);
      m_reset_simulation = false;
    }
  }

  // free visualization storage
  mjv_freeScene(&scn);
  mjr_freeContext(&con);

  // free MuJoCo model and data
  mj_deleteData(d);
  mj_deleteModel(m);
  glfwTerminate();

  return 0;
}

} // namespace rackki_learning
