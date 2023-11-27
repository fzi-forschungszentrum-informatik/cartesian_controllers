#include "rackki_learning/skill_controller.h"
#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "rclcpp/logging.hpp"
#include "urdf/model.h"
#include <memory>
#include <tensorflow/cc/ops/const_op.h>
#include <tensorflow/cc/saved_model/loader.h>
#include <tensorflow/core/framework/tensor.h>
#include <tensorflow/core/framework/tensor_shape.h>
#include <tensorflow/core/public/session.h>

namespace rackki_learning {

controller_interface::InterfaceConfiguration
SkillController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  return conf;
}

controller_interface::InterfaceConfiguration SkillController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  conf.names.reserve(2 * m_joint_names.size());
  for (const auto& joint_name : m_joint_names)
  {
    conf.names.push_back(joint_name + "/position");
    conf.names.push_back(joint_name + "/velocity");
  }
  return conf;
}

SkillController::CallbackReturn SkillController::on_init()
{
  auto_declare<std::string>("model_path", "");
  auto_declare<std::string>("robot_description", "");
  auto_declare<std::string>("robot_base_link", "");
  auto_declare<std::string>("end_effector_link", "");
  auto_declare<std::vector<std::string> >("joints", std::vector<std::string>());
  return CallbackReturn::SUCCESS;
}

SkillController::CallbackReturn
SkillController::on_configure([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  auto robot_model       = urdf::Model();
  auto robot_tree        = KDL::Tree();
  m_joint_names          = get_node()->get_parameter("joints").as_string_array();
  auto robot_description = get_node()->get_parameter("robot_description").as_string();
  auto robot_base_link   = get_node()->get_parameter("robot_base_link").as_string();
  auto end_effector_link = get_node()->get_parameter("end_effector_link").as_string();
  auto model_path        = get_node()->get_parameter("model_path").as_string();
  auto status            = tensorflow::LoadSavedModel(
    tensorflow::SessionOptions(), tensorflow::RunOptions(), model_path, {"serve"}, &m_bundle);
  if (!status.ok())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Something is wrong with the trained model");
    return CallbackReturn::ERROR;
  }
  if (robot_description.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No robot_description specified");
    return CallbackReturn::ERROR;
  }
  if (robot_base_link.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No robot_base_link specified");
    return CallbackReturn::ERROR;
  }
  if (end_effector_link.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No end_effector_link is empty");
    return CallbackReturn::ERROR;
  }
  if (!robot_model.initString(robot_description))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse urdf model from 'robot_description'");
    return CallbackReturn::ERROR;
  }
  if (!kdl_parser::treeFromUrdfModel(robot_model, robot_tree))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse KDL tree from urdf model");
    return CallbackReturn::ERROR;
  }
  if (!robot_tree.getChain(robot_base_link, end_effector_link, m_robot_chain))
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse robot chain from urdf model");
    return CallbackReturn::ERROR;
  }
  if (m_joint_names.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "No joints specified");
    return CallbackReturn::ERROR;
  }

  m_joint_positions     = KDL::JntArray(m_joint_names.size());
  m_joint_velocities    = KDL::JntArray(m_joint_names.size());
  m_end_effector_solver = std::make_unique<KDL::ChainFkSolverVel_recursive>(m_robot_chain);

  return CallbackReturn::SUCCESS;
}

SkillController::CallbackReturn
SkillController::on_activate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  if (!controller_interface::get_ordered_interfaces(this->state_interfaces_,
                                                    m_joint_names,
                                                    hardware_interface::HW_IF_POSITION,
                                                    m_joint_state_pos_handles))
  {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Expected %zu '%s' state interfaces, got %zu.",
                 m_joint_names.size(),
                 hardware_interface::HW_IF_POSITION,
                 m_joint_state_pos_handles.size());
    return CallbackReturn::ERROR;
  }
  if (!controller_interface::get_ordered_interfaces(this->state_interfaces_,
                                                    m_joint_names,
                                                    hardware_interface::HW_IF_VELOCITY,
                                                    m_joint_state_vel_handles))
  {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Expected %zu '%s' state interfaces, got %zu.",
                 m_joint_names.size(),
                 hardware_interface::HW_IF_VELOCITY,
                 m_joint_state_vel_handles.size());
    return CallbackReturn::ERROR;
  }

  m_joint_mutex.lock();
  updateJointStates();
  m_joint_mutex.unlock();

  m_active         = true;
  m_serving_thread = std::thread([this]() {
    while (m_active)
    {
      auto inputs  = buildInputTensor();
      auto outputs = std::vector<tensorflow::Tensor>();
      auto status = m_bundle.GetSession()->Run(inputs, {"StatefulPartitionedCall:0"}, {}, &outputs);
      if (status.ok())
      {
        auto values                     = outputs[0].flat<float>();
        m_target_wrench.header.stamp    = this->get_node()->now();
        m_target_wrench.wrench.force.x  = values(0);
        m_target_wrench.wrench.force.y  = values(1);
        m_target_wrench.wrench.force.z  = values(2);
        m_target_wrench.wrench.torque.x = values(3);
        m_target_wrench.wrench.torque.y = values(4);
        m_target_wrench.wrench.torque.z = values(5);
      }
      else
      {
        auto clock = *get_node()->get_clock();
        RCLCPP_WARN_STREAM_THROTTLE(
          get_node()->get_logger(), clock, 3000, "Error in model prediction: " << status.message());
      }
    }
  });

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type
SkillController::update([[maybe_unused]] const rclcpp::Time& time,
                        [[maybe_unused]] const rclcpp::Duration& period)
{
  if (m_joint_mutex.try_lock())
  {
    updateJointStates();
    m_joint_mutex.unlock();
  }
  return controller_interface::return_type::OK;
}

SkillController::CallbackReturn
SkillController::on_deactivate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  cleanup();
  return CallbackReturn::SUCCESS;
}

SkillController::CallbackReturn
SkillController::on_shutdown([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  cleanup();
  auto status = m_bundle.GetSession()->Close();
  if (!status.ok())
  {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

std::vector<std::pair<std::string, tensorflow::Tensor> > SkillController::buildInputTensor()
{
  // Compute current end effector pose and velocity w.r.t the robot base link
  KDL::FrameVel end_effector;
  m_joint_mutex.lock();
  m_end_effector_solver->JntToCart(KDL::JntArrayVel(m_joint_positions, m_joint_velocities),
                                   end_effector);
  m_joint_mutex.unlock();

  // The neural network expects all inputs with respect to the target frame.
  KDL::Frame current_pose  = m_target_pose.Inverse() * end_effector.GetFrame();
  KDL::Twist current_twist = m_target_pose.Inverse() * end_effector.GetTwist();
  double current_pose_qx;
  double current_pose_qy;
  double current_pose_qz;
  double current_pose_qw;
  current_pose.M.GetQuaternion(current_pose_qx, current_pose_qy, current_pose_qz, current_pose_qw);

  auto input_shape = tensorflow::TensorShape({1, 1, 19});
  tensorflow::Input::Initializer input(std::initializer_list<float>({
                                         static_cast<float>(current_pose.p.x()),
                                         static_cast<float>(current_pose.p.y()),
                                         static_cast<float>(current_pose.p.z()),
                                         static_cast<float>(current_pose_qx),
                                         static_cast<float>(current_pose_qy),
                                         static_cast<float>(current_pose_qz),
                                         static_cast<float>(current_pose_qw),
                                         static_cast<float>(current_twist.vel.x()),
                                         static_cast<float>(current_twist.vel.y()),
                                         static_cast<float>(current_twist.vel.z()),
                                         static_cast<float>(current_twist.rot.x()),
                                         static_cast<float>(current_twist.rot.y()),
                                         static_cast<float>(current_twist.rot.z()),
                                         static_cast<float>(m_target_wrench.wrench.force.x),
                                         static_cast<float>(m_target_wrench.wrench.force.y),
                                         static_cast<float>(m_target_wrench.wrench.force.z),
                                         static_cast<float>(m_target_wrench.wrench.torque.x),
                                         static_cast<float>(m_target_wrench.wrench.torque.y),
                                         static_cast<float>(m_target_wrench.wrench.torque.z),
                                       }),
                                       input_shape);

  return {{"serving_default_lstm_input:0", input.tensor}};
}

void SkillController::updateJointStates()
{
  for (size_t i = 0; i < m_joint_names.size(); ++i)
  {
    m_joint_positions(i)  = m_joint_state_pos_handles[i].get().get_value();
    m_joint_velocities(i) = m_joint_state_vel_handles[i].get().get_value();
  }
}

void SkillController::cleanup()
{
  if (m_active)
  {
    m_joint_state_pos_handles.clear();
    m_joint_state_vel_handles.clear();
    this->release_interfaces();
    m_active = false;
  }

  if (m_serving_thread.joinable())
  {
    m_serving_thread.join();
  }
}

} // namespace rackki_learning

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rackki_learning::SkillController, controller_interface::ControllerInterface)
