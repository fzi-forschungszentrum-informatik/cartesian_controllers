#ifndef SKILL_CONTROLLER_H_INCLUDED
#define SKILL_CONTROLLER_H_INCLUDED

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "kdl/chain.hpp"
#include "rackki_interfaces/srv/set_target.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <array>
#include <atomic>
#include <hardware_interface/loaned_state_interface.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/frames.hpp>
#include <mutex>
#include <tensorflow/cc/saved_model/loader.h>
#include <thread>

namespace rackki_learning {

class SkillController : public controller_interface::ControllerInterface
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  [[nodiscard]] controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  [[nodiscard]] controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  LifecycleNodeInterface::CallbackReturn on_init() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

private:
  void updateInputSequence();
  void updateJointStates();
  void cleanup();
  bool setTarget(const std::string& target);

  std::thread m_serving_thread;
  tensorflow::SavedModelBundleLite m_bundle;
  std::deque<std::pair<std::string, tensorflow::Tensor> > m_input_sequence;
  std::vector<double> m_mean;
  std::vector<double> m_sigma;

  std::vector<std::string> m_joint_names;
  KDL::JntArray m_joint_positions;
  KDL::JntArray m_joint_velocities;
  geometry_msgs::msg::WrenchStamped m_target_wrench;
  std::mutex m_joint_mutex;
  KDL::Frame m_target_pose;
  std::mutex m_target_mutex;
  std::atomic<bool> m_has_target = false;
  rclcpp::Service<rackki_interfaces::srv::SetTarget>::SharedPtr m_set_target_server;
  std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;
  std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr m_target_wrench_publisher;

  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface> >
    m_joint_state_pos_handles;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface> >
    m_joint_state_vel_handles;

  KDL::Chain m_robot_chain;
  std::unique_ptr<KDL::ChainFkSolverVel_recursive> m_end_effector_solver;
  std::atomic<bool> m_active = false;
};

} // namespace rackki_learning

#endif
