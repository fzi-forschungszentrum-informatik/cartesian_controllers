// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    mujoco_simulator.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2022/09/27
 *
 */
//-----------------------------------------------------------------------------

#pragma once

#include <array>
#include <atomic>
#include <cstdio>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "GLFW/glfw3.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "mujoco/mujoco.h"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace rackki_learning {

/**
 * @brief MuJoCo's physics engine with rendering and basic window mouse interaction
 *
 * A MuJoCo-based simulator for teleoperation and data generation.
 *
 * We use this simulator to demonstrate assembly by teleoperation with a simple
 * teach device.  The assembly consists of two objects: the active component
 * that we actively steer with a target_wrench, and the passive component that
 * is everything else in the environment.
 * The idea is to record human behavior (= target_wrench) while steering the
 * active component together with the active component's state feedback (pose,
 * twist) and obtain datasets for machine learning.
 *
 * The simulator is implemented as a singleton class, which circumvents the problem of
 * using OpenGL-related global function pointers for rendering.
 *
 * User code interfaces this class by getting an instance and calling static
 * functions on it.  It's designed to run with an independent simulation rate,
 * disjoint from ROS2 in a separate thread.
 *
 */
class MuJoCoSimulator
{
private:
  MuJoCoSimulator();

  // For topic communication with ROS2
  std::shared_ptr<rclcpp::Node> m_node;
  std::atomic_bool m_ready = false;

  static MuJoCoSimulator& getInstance()
  {
    static MuJoCoSimulator simulator;
    return simulator;
  }

public:
  // Modern singleton approach
  MuJoCoSimulator(const MuJoCoSimulator&) = delete;
  MuJoCoSimulator& operator=(const MuJoCoSimulator&) = delete;
  MuJoCoSimulator(MuJoCoSimulator&&)                 = delete;
  MuJoCoSimulator& operator=(MuJoCoSimulator&&) = delete;

  // Use this in ROS2 code
  static bool ready() { return getInstance().m_ready; };
  static std::shared_ptr<rclcpp::Node> getNode() { return getInstance().m_node; };


  // ROS2 interfaces
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_feedback_pose_publisher;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr m_feedback_twist_publisher;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr m_target_wrench_subscriber;
  void targetWrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr wrench);
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_reset_server;
  std::atomic_bool m_reset_simulation = false;

  // Identifier for the active assembly component
  int m_active_body = 0;

  // MuJoCo data structures
  mjModel* m = NULL; // MuJoCo model
  mjData* d  = NULL; // MuJoCo data
  mjvCamera cam;     // abstract camera
  mjvOption opt;     // visualization options
  mjvScene scn;      // abstract scene
  mjrContext con;    // custom GPU context

  // mouse interaction
  bool button_left   = false;
  bool button_middle = false;
  bool button_right  = false;
  double lastx       = 0;
  double lasty       = 0;

  // Buffers for data exchange with ROS2
  std::vector<double> m_target_wrench = {0, 0, 0, 0, 0, 0};
  std::vector<std::array<double, 7> > m_starting_poses;

  // Safety guards for buffers
  std::mutex command_mutex;

  // Keyboard callback
  static void keyboardCB(GLFWwindow* window, int key, int scancode, int act, int mods);
  void keyboardCBImpl(GLFWwindow* window, int key, int scancode, int act, int mods);

  // Mouse button callback
  static void mouseButtonCB(GLFWwindow* window, int button, int act, int mods);
  void mouseButtonCBImpl(GLFWwindow* window, int button, int act, int mods);

  // Mouse move callback
  static void mouseMoveCB(GLFWwindow* window, double xpos, double ypos);
  void mouseMoveCBImpl(GLFWwindow* window, double xpos, double ypos);

  // Scroll callback
  static void scrollCB(GLFWwindow* window, double xoffset, double yoffset);
  void scrollCBImpl(GLFWwindow* window, double xoffset, double yoffset);

  // Control input callback for the solver
  static void controlCB(const mjModel* m, mjData* d);
  void controlCBImpl(const mjModel* m, mjData* d);

  // Call this in a separate thread
  static int simulate();
  int simulateImpl();
};

} // namespace rackki_learning
