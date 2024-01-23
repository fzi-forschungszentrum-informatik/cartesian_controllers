#ifndef SKILL_CONTROLLER_H_INCLUDED
#define SKILL_CONTROLLER_H_INCLUDED

#include "cartesian_skill_controller/msg/controller_state.hpp"
#include "cartesian_skill_controller/srv/set_target.hpp"
#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "kdl/chain.hpp"
#include "kdl/chainfksolver.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <array>
#include <atomic>
#include <hardware_interface/loaned_state_interface.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <mutex>
#include <tensorflow/cc/saved_model/loader.h>
#include <thread>

namespace cartesian_skill_controller {

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
  constexpr static int FEATURE_DIM = 7;
  tensorflow::Tensor buildInputTensor();
  geometry_msgs::msg::WrenchStamped processOutputTensor(const tensorflow::Tensor& output);
  void cleanup();
  bool setTarget(const std::string& target);
  bool done();
  KDL::Frame getCurrentPoseToTarget();

  std::thread m_serving_thread;
  tensorflow::SavedModelBundleLite m_bundle;
  std::deque<std::array<float, FEATURE_DIM> > m_input_sequence;
  std::vector<double> m_mean;
  std::vector<double> m_sigma;

  std::vector<std::string> m_joint_names;
  KDL::JntArray m_joint_positions;
  KDL::Frame m_target_pose;
  std::mutex m_target_mutex;
  std::atomic<bool> m_has_target = false;
  rclcpp::Service<cartesian_skill_controller::srv::SetTarget>::SharedPtr m_set_target_server;
  std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;
  std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr m_target_wrench_publisher;
  rclcpp::Publisher<cartesian_skill_controller::msg::ControllerState>::SharedPtr
    m_controller_state_publisher;

  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface> >
    m_joint_state_pos_handles;

  KDL::Chain m_robot_chain;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> m_end_effector_solver;
  std::atomic<bool> m_active   = false;
  std::atomic<bool> m_shutdown = false;
};

} // namespace cartesian_skill_controller

#endif
