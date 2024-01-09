////////////////////////////////////////////////////////////////////////////////
// Copyright 2022 FZI Research Center for Information Technology
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
/*!\file    system_interface.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2022/01/31
 *
 */
//-----------------------------------------------------------------------------

#pragma once

#include <cartesian_controller_base/ROS2VersionConfig.h>

#include <map>
#include <thread>

#include "cartesian_controller_simulation/mujoco_simulator.h"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

#if defined CARTESIAN_CONTROLLERS_FOXY
#include "hardware_interface/base_interface.hpp"
#endif

namespace cartesian_controller_simulation
{
// Two custom hardware interfaces for torque-actuated robots:
// proportional gain (stiffness) and derivative gain (damping).
constexpr char HW_IF_STIFFNESS[] = "stiffness";
constexpr char HW_IF_DAMPING[] = "damping";

/**
 * @brief A MuJoCo-based, standalone simulator for cartesian_controllersand ROS2-control
 *
 * This class provides a simulated robot for controller development and
 * testing.  It's instantiated via the usual ROS2-conform lifecylce as a
 * controller_manager coordinated library.
 *
 */
#if defined CARTESIAN_CONTROLLERS_GALACTIC || defined CARTESIAN_CONTROLLERS_HUMBLE || \
  defined CARTESIAN_CONTROLLERS_IRON
class Simulator : public hardware_interface::SystemInterface
#elif defined CARTESIAN_CONTROLLERS_FOXY
class Simulator : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
#endif
{
public:
  using return_type = hardware_interface::return_type;
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  RCLCPP_SHARED_PTR_DEFINITIONS(Simulator)

#if defined CARTESIAN_CONTROLLERS_GALACTIC || defined CARTESIAN_CONTROLLERS_HUMBLE || \
  defined CARTESIAN_CONTROLLERS_IRON
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
#elif defined CARTESIAN_CONTROLLERS_FOXY
  return_type configure(const hardware_interface::HardwareInfo & info) override;
#endif

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

#if defined CARTESIAN_CONTROLLERS_HUMBLE || defined CARTESIAN_CONTROLLERS_IRON
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

#elif defined CARTESIAN_CONTROLLERS_GALACTIC || defined CARTESIAN_CONTROLLERS_FOXY
  return_type read() override;
  return_type write() override;
#endif

#if defined CARTESIAN_CONTROLLERS_FOXY
  return_type start() override;
  return_type stop() override;
#endif

private:
  // Command buffers for the controllers
  std::vector<double> m_position_commands;
  std::vector<double> m_velocity_commands;

  // State buffers for the controllers
  std::vector<double> m_positions;
  std::vector<double> m_velocities;
  std::vector<double> m_efforts;

  // Anydrive gains
  std::vector<double> m_stiffness;
  std::vector<double> m_damping;

  // Run MuJoCo's solver in a separate thread
  std::thread m_simulation;

  // Parameters
  std::string m_mujoco_model;
};

}  // namespace cartesian_controller_simulation
