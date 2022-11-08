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

#include <atomic>
#include <cstdio>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "GLFW/glfw3.h"
#include "mujoco.h"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

namespace rackki_learning {

/**
 * @brief MuJoCo's physics engine with rendering and basic window mouse interaction
 *
 * A MuJoCo-based simulator for teleoperation and data generation
 *
 * TODO:
 * It's implemented as a singleton class, which circumvents the problem of
 * using OpenGL-related global function pointers for rendering.
 * Most of the code is currently left as-is and we mostly add the necessary
 * enhancements for data exchange with the ROS2-control thread.
 *
 * User code interfaces this class by getting an instance and calling static
 * functions on it.  It's designed to run with an independent simulation rate,
 * disjoint from ROS2-control in a separate thread.
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
  static bool ready() {return getInstance().m_ready;};
  static std::shared_ptr<rclcpp::Node> getNode() {return getInstance().m_node;};


  // ROS2 input and output topics
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_feedback_pose_publisher;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr m_feedback_twist_publisher;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr m_target_wrench_subscriber;
  void targetWrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr wrench);

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
