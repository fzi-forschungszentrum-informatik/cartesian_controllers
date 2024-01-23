////////////////////////////////////////////////////////////////////////////////
// Copyright 2024 FZI Research Center for Information Technology
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

//-----------------------------------------------------------------------------
/*!\file    skill_controller.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2024/01/23
 *
 */
//-----------------------------------------------------------------------------

#include "cartesian_skill_controller/skill_controller.h"
#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/tree.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "rclcpp/logging.hpp"
#include "urdf/model.h"
#include "yaml-cpp/yaml.h"
#include <algorithm>
#include <chrono>
#include <deque>
#include <kdl/frames.hpp>
#include <memory>
#include <tensorflow/cc/ops/const_op.h>
#include <tensorflow/cc/saved_model/loader.h>
#include <tensorflow/core/framework/tensor.h>
#include <tensorflow/core/framework/tensor_shape.h>
#include <tensorflow/core/public/session.h>
#include <vector>

namespace cartesian_skill_controller {

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
  }
  return conf;
}

SkillController::CallbackReturn SkillController::on_init()
{
  auto_declare<std::string>("model_path", "");
  auto_declare<std::string>("robot_description", "");
  auto_declare<std::string>("robot_base_link", "");
  auto_declare<std::string>("end_effector_link", "");
  auto_declare<std::vector<std::string> >("joints", {});
  auto_declare<double>("max_force", 30.0);
  auto_declare<double>("max_torque", 3.0);
  auto_declare<int>("prediction_memory", 30);
  auto_declare<int>("prediction_rate", 10);
  auto_declare<double>("prediction_scale", 1.0);
  auto_declare<std::vector<double> >("done_when_in", {0.0, 0.0});
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
  if (m_mean.size() != FEATURE_DIM || m_sigma.size() != FEATURE_DIM)
  {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Wrong size of input scaling: mean: %lu, sigma: %lu",
                 m_mean.size(),
                 m_sigma.size());
    return CallbackReturn::ERROR;
  }

  m_joint_positions         = KDL::JntArray(m_joint_names.size());
  m_end_effector_solver     = std::make_unique<KDL::ChainFkSolverPos_recursive>(m_robot_chain);
  m_target_wrench_publisher = get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
    std::string(get_node()->get_name()) + "/target_wrench", 1);
  m_controller_state_publisher =
    get_node()->create_publisher<cartesian_skill_controller::msg::ControllerState>(
      std::string(get_node()->get_name()) + "/controller_state", 1);

  m_tf_buffer   = std::make_unique<tf2_ros::Buffer>(get_node()->get_clock());
  m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

  m_set_target_server = get_node()->create_service<cartesian_skill_controller::srv::SetTarget>(
    std::string(get_node()->get_name()) + "/set_target",
    [this](const std::shared_ptr<cartesian_skill_controller::srv::SetTarget::Request> request,
           std::shared_ptr<cartesian_skill_controller::srv::SetTarget::Response> response) -> void {
      response->success = setTarget(request->tf_name);
    });

  m_shutdown       = false;
  m_serving_thread = std::thread([this]() {
    using ControllerState       = cartesian_skill_controller::msg::ControllerState;
    auto publishControllerState = [this](const ControllerState::_state_type& state) -> void {
      auto controller_state  = ControllerState();
      controller_state.state = state;
      m_controller_state_publisher->publish(controller_state);
    };

    while (!m_shutdown)
    {
      while (!m_active)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      m_input_sequence.clear();
      while (m_active)
      {
        auto inputs  = buildInputTensor();
        auto outputs = std::vector<tensorflow::Tensor>();
        auto status  = m_bundle.GetSession()->Run({{"serving_default_embedding_input:0", inputs}},
                                                 {"StatefulPartitionedCall:0"},
                                                 {},
                                                 &outputs);
        if (status.ok())
        {
          publishControllerState(ControllerState::STATE_RUNNING);
          auto target_wrench = processOutputTensor(outputs[0]);
          m_target_wrench_publisher->publish(target_wrench);
          using namespace std::chrono;
          auto interval = round<milliseconds>(
            duration<double>(1.0 / get_node()->get_parameter("prediction_rate").as_int()));
          std::this_thread::sleep_for(interval);
        }
        else
        {
          publishControllerState(ControllerState::STATE_ERROR);
          auto clock = *get_node()->get_clock();
          RCLCPP_WARN_STREAM_THROTTLE(get_node()->get_logger(),
                                      clock,
                                      3000,
                                      "Error in model prediction: " << status.message());
        }

        if (done())
        {
          publishControllerState(ControllerState::STATE_DONE);
          auto zero_wrench            = geometry_msgs::msg::WrenchStamped();
          zero_wrench.header.stamp    = this->get_node()->now();
          zero_wrench.header.frame_id = get_node()->get_parameter("robot_base_link").as_string();
          m_target_wrench_publisher->publish(zero_wrench);
        }
      }
    }
  });
  m_serving_thread.detach();

  return CallbackReturn::SUCCESS;
}

SkillController::CallbackReturn
SkillController::on_activate([[maybe_unused]] const rclcpp_lifecycle::State& previous_state)
{
  m_joint_state_pos_handles.clear();

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

  m_active = true;
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type
SkillController::update([[maybe_unused]] const rclcpp::Time& time,
                        [[maybe_unused]] const rclcpp::Duration& period)
{
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
  m_shutdown = true;
  cleanup();
  auto status = m_bundle.GetSession()->Close();
  if (!status.ok())
  {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

tensorflow::Tensor SkillController::buildInputTensor()
{
  auto current_pose = getCurrentPoseToTarget();
  double current_pose_qx;
  double current_pose_qy;
  double current_pose_qz;
  double current_pose_qw;
  current_pose.M.GetQuaternion(current_pose_qx, current_pose_qy, current_pose_qz, current_pose_qw);

  auto point = std::array<float, FEATURE_DIM>({
    static_cast<float>((current_pose.p.x() - m_mean[0]) / m_sigma[0]),
    static_cast<float>((current_pose.p.y() - m_mean[1]) / m_sigma[1]),
    static_cast<float>((current_pose.p.z() - m_mean[2]) / m_sigma[2]),
    static_cast<float>((current_pose_qx - m_mean[3]) / m_sigma[3]),
    static_cast<float>((current_pose_qy - m_mean[4]) / m_sigma[4]),
    static_cast<float>((current_pose_qz - m_mean[5]) / m_sigma[5]),
    static_cast<float>((current_pose_qw - m_mean[6]) / m_sigma[6]),
  });
  m_input_sequence.push_front(point);
  if (m_input_sequence.size() > get_node()->get_parameter("prediction_memory").as_int())
  {
    m_input_sequence.pop_back();
  }

  tensorflow::Tensor input(
    tensorflow::DT_FLOAT,
    tensorflow::TensorShape({1, static_cast<long>(m_input_sequence.size()), FEATURE_DIM}));
  auto input_data = input.tensor<float, 3>();

  for (int i = 0; i < m_input_sequence.size(); ++i)
  {
    for (int j = 0; j < FEATURE_DIM; ++j)
    {
      input_data(0, i, j) = m_input_sequence[i][j];
    }
  }
  input.tensor<float, 3>() = input_data;
  return input;
}

geometry_msgs::msg::WrenchStamped
SkillController::processOutputTensor(const tensorflow::Tensor& output)
{
  double max_force           = std::abs(get_node()->get_parameter("max_force").as_double());
  double max_torque          = std::abs(get_node()->get_parameter("max_torque").as_double());
  double scale               = std::abs(get_node()->get_parameter("prediction_scale").as_double());
  auto values                = output.flat<float>();
  auto predicted_wrench      = KDL::Wrench();
  predicted_wrench.force[0]  = values(0);
  predicted_wrench.force[1]  = values(1);
  predicted_wrench.force[2]  = values(2);
  predicted_wrench.torque[0] = values(3);
  predicted_wrench.torque[1] = values(4);
  predicted_wrench.torque[2] = values(5);
  predicted_wrench = m_target_pose.M * predicted_wrench; // force control in robot base coordinates

  auto target_wrench            = geometry_msgs::msg::WrenchStamped();
  target_wrench.header.stamp    = this->get_node()->now();
  target_wrench.header.frame_id = get_node()->get_parameter("robot_base_link").as_string();
  target_wrench.wrench.force.x =
    std::clamp(predicted_wrench.force[0] * scale, -max_force, max_force);
  target_wrench.wrench.force.y =
    std::clamp(predicted_wrench.force[1] * scale, -max_force, max_force);
  target_wrench.wrench.force.z =
    std::clamp(predicted_wrench.force[2] * scale, -max_force, max_force);
  target_wrench.wrench.torque.x =
    std::clamp(predicted_wrench.torque[0] * scale, -max_torque, max_torque);
  target_wrench.wrench.torque.y =
    std::clamp(predicted_wrench.torque[1] * scale, -max_torque, max_torque);
  target_wrench.wrench.torque.z =
    std::clamp(predicted_wrench.torque[2] * scale, -max_torque, max_torque);
  return target_wrench;
}

void SkillController::cleanup()
{
  m_active     = false;
  m_has_target = false;
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

bool SkillController::done()
{
  auto dist  = getCurrentPoseToTarget().p.Norm();
  auto range = get_node()->get_parameter("done_when_in").as_double_array();
  return range[0] < dist && dist < range[1];
}

KDL::Frame SkillController::getCurrentPoseToTarget()
{
  // Compute current end effector pose w.r.t the robot base link
  KDL::Frame end_effector;
  for (size_t i = 0; i < m_joint_names.size(); ++i)
  {
    m_joint_positions(i) = m_joint_state_pos_handles[i].get().get_value();
  }
  m_end_effector_solver->JntToCart(m_joint_positions, end_effector);

  // The neural network expects all inputs with respect to the target frame.
  m_target_mutex.lock();
  KDL::Frame current_pose = m_target_pose.Inverse() * end_effector;
  m_target_mutex.unlock();
  return current_pose;
}

} // namespace cartesian_skill_controller

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(cartesian_skill_controller::SkillController,
                       controller_interface::ControllerInterface)