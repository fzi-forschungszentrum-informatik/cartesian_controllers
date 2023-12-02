#include "rackki_learning/skill_controller.h"
#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "rclcpp/logging.hpp"
#include "urdf/model.h"
#include "yaml-cpp/yaml.h"
#include <algorithm>
#include <deque>
#include <kdl/frames.hpp>
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
  auto_declare<double>("max_force", 30.0);
  auto_declare<double>("max_torque", 3.0);
  auto_declare<int>("prediction_memory", 30);
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

  try
  {
    auto config = YAML::LoadFile(model_path + "/input_scaling.yaml");
    for (const auto& entry : config["mean"])
    {
      m_mean.emplace_back(entry.as<double>());
    }
    for (const auto& entry : config["sigma"])
    {
      m_sigma.emplace_back(entry.as<double>());
    }
  }
  catch (const YAML::Exception& e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to load input scaling from file: %s", e.what());
    return CallbackReturn::ERROR;
  }
  if (m_mean.size() != 19 || m_sigma.size() != 19)
  {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Wrong size of input scaling: mean: %lu, sigma: %lu",
                 m_mean.size(),
                 m_sigma.size());
    return CallbackReturn::ERROR;
  }

  m_joint_positions         = KDL::JntArray(m_joint_names.size());
  m_joint_velocities        = KDL::JntArray(m_joint_names.size());
  m_end_effector_solver     = std::make_unique<KDL::ChainFkSolverVel_recursive>(m_robot_chain);
  m_target_wrench_publisher = get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
    std::string(get_node()->get_name()) + "/target_wrench", 1);

  m_tf_buffer   = std::make_unique<tf2_ros::Buffer>(get_node()->get_clock());
  m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

  m_set_target_server = get_node()->create_service<rackki_interfaces::srv::SetTarget>(
    std::string(get_node()->get_name()) + "/set_target",
    [this](const std::shared_ptr<rackki_interfaces::srv::SetTarget::Request> request,
           std::shared_ptr<rackki_interfaces::srv::SetTarget::Response> response) -> void {
      response->success = setTarget(request->tf_name);
    });

  return CallbackReturn::SUCCESS;
}

SkillController::CallbackReturn
SkillController::on_activate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  if (!m_has_target)
  {
    RCLCPP_WARN(get_node()->get_logger(), "No TF target specified.");
    return CallbackReturn::ERROR;
  }

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
      updateInputSequence();
      auto outputs = std::vector<tensorflow::Tensor>();
      auto status  = m_bundle.GetSession()->Run({m_input_sequence.begin(), m_input_sequence.end()},
                                               {"StatefulPartitionedCall:0"},
                                               {},
                                               &outputs);
      if (status.ok())
      {
        float max_force  = std::abs(get_node()->get_parameter("max_force").as_double());
        float max_torque = std::abs(get_node()->get_parameter("max_torque").as_double());
        auto values      = outputs[0].flat<float>();
        m_target_wrench.header.stamp    = this->get_node()->now();
        m_target_wrench.wrench.force.x  = std::clamp(values(0), -max_force, max_force);
        m_target_wrench.wrench.force.y  = std::clamp(values(1), -max_force, max_force);
        m_target_wrench.wrench.force.z  = std::clamp(values(2), -max_force, max_force);
        m_target_wrench.wrench.torque.x = std::clamp(values(3), -max_torque, max_torque);
        m_target_wrench.wrench.torque.y = std::clamp(values(4), -max_torque, max_torque);
        m_target_wrench.wrench.torque.z = std::clamp(values(5), -max_torque, max_torque);
        m_target_wrench_publisher->publish(m_target_wrench);
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

void SkillController::updateInputSequence()
{
  // Compute current end effector pose and velocity w.r.t the robot base link
  KDL::FrameVel end_effector;
  m_joint_mutex.lock();
  m_end_effector_solver->JntToCart(KDL::JntArrayVel(m_joint_positions, m_joint_velocities),
                                   end_effector);
  m_joint_mutex.unlock();

  // The neural network expects all inputs with respect to the target frame.
  m_target_mutex.lock();
  KDL::Frame current_pose  = m_target_pose.Inverse() * end_effector.GetFrame();
  KDL::Twist current_twist = m_target_pose.Inverse() * end_effector.GetTwist();
  m_target_mutex.unlock();
  double current_pose_qx;
  double current_pose_qy;
  double current_pose_qz;
  double current_pose_qw;
  current_pose.M.GetQuaternion(current_pose_qx, current_pose_qy, current_pose_qz, current_pose_qw);

  auto input_shape = tensorflow::TensorShape({1, 1, 19});
  tensorflow::Input::Initializer input(
    std::initializer_list<float>({
      static_cast<float>((current_pose.p.x() - m_mean[0]) / m_sigma[0]),
      static_cast<float>((current_pose.p.y() - m_mean[1]) / m_sigma[1]),
      static_cast<float>((current_pose.p.z() - m_mean[2]) / m_sigma[2]),
      static_cast<float>((current_pose_qx - m_mean[3]) / m_sigma[3]),
      static_cast<float>((current_pose_qy - m_mean[4]) / m_sigma[4]),
      static_cast<float>((current_pose_qz - m_mean[5]) / m_sigma[5]),
      static_cast<float>((current_pose_qw - m_mean[6]) / m_sigma[6]),
      static_cast<float>((current_twist.vel.x() - m_mean[7]) / m_sigma[7]),
      static_cast<float>((current_twist.vel.y() - m_mean[8]) / m_sigma[8]),
      static_cast<float>((current_twist.vel.z() - m_mean[9]) / m_sigma[9]),
      static_cast<float>((current_twist.rot.x() - m_mean[10]) / m_sigma[10]),
      static_cast<float>((current_twist.rot.y() - m_mean[11]) / m_sigma[11]),
      static_cast<float>((current_twist.rot.z() - m_mean[12]) / m_sigma[12]),
      static_cast<float>((m_target_wrench.wrench.force.x - m_mean[13]) / m_sigma[13]),
      static_cast<float>((m_target_wrench.wrench.force.y - m_mean[14]) / m_sigma[14]),
      static_cast<float>((m_target_wrench.wrench.force.z - m_mean[15]) / m_sigma[15]),
      static_cast<float>((m_target_wrench.wrench.torque.x - m_mean[16]) / m_sigma[16]),
      static_cast<float>((m_target_wrench.wrench.torque.y - m_mean[17]) / m_sigma[17]),
      static_cast<float>((m_target_wrench.wrench.torque.z - m_mean[18]) / m_sigma[18]),
    }),
    input_shape);

  m_input_sequence.push_front({"serving_default_lstm_input:0", input.tensor});
  if (m_input_sequence.size() > get_node()->get_parameter("prediction_memory").as_int())
  {
    m_input_sequence.pop_back();
  }
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
  m_active     = false;
  m_has_target = false;

  if (m_serving_thread.joinable())
  {
    m_serving_thread.join();
  }

  m_input_sequence.clear();
  m_joint_state_pos_handles.clear();
  m_joint_state_vel_handles.clear();
  this->release_interfaces();
}

bool SkillController::setTarget(const std::string& target)
{
  geometry_msgs::msg::TransformStamped t;
  auto base = get_node()->get_parameter("robot_base_link").as_string();
  try
  {
    t = m_tf_buffer->lookupTransform(base, target, tf2::TimePointZero);
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_INFO(get_node()->get_logger(),
                "Could not lookup transform from %s to %s: %s",
                target.c_str(),
                base.c_str(),
                ex.what());
    return false;
  }
  m_target_mutex.lock();
  m_target_pose.p[0] = t.transform.translation.x;
  m_target_pose.p[1] = t.transform.translation.y;
  m_target_pose.p[2] = t.transform.translation.z;
  m_target_pose.M    = KDL::Rotation::Quaternion(
    t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w);
  m_target_mutex.unlock();
  m_has_target = true;
  return true;
}

} // namespace rackki_learning

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rackki_learning::SkillController, controller_interface::ControllerInterface)
