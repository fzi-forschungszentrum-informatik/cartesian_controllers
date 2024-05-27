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
/*!\file    system_interface.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2022/01/31
 *
 */
//-----------------------------------------------------------------------------

#include "cartesian_controller_simulation/system_interface.h"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "cartesian_controller_simulation/mujoco_simulator.h"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cartesian_controller_simulation
{

Simulator::CallbackReturn Simulator::on_init(const hardware_interface::HardwareInfo & info)
{
  // Keep an internal copy of the given configuration
  if (hardware_interface::SystemInterface::on_init(info) != Simulator::CallbackReturn::SUCCESS)
  {
    return Simulator::CallbackReturn::ERROR;
  }

  // Start the simulator in parallel.
  // Let the thread's destructor clean-up all resources
  // once users close the simulation window.
  m_mujoco_model = info_.hardware_parameters["mujoco_model"];
  m_simulation = std::thread(MuJoCoSimulator::simulate, m_mujoco_model);
  m_simulation.detach();

  m_positions.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  m_velocities.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  m_efforts.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  m_position_commands.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  m_velocity_commands.resize(info_.joints.size(), 0.0);

  // Default gains
  m_stiffness.resize(info_.joints.size(), 0);
  m_damping.resize(info_.joints.size(), 0);

  // Initialize joint gains for the simulator
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    m_stiffness[i] = std::stod(info_.joints[i].parameters.at("p"));
    m_damping[i] = std::stod(info_.joints[i].parameters.at("d"));
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 2)
    {
      RCLCPP_ERROR(rclcpp::get_logger("Simulator"),
                   "Joint '%s' needs two possible command interfaces.", joint.name.c_str());

      return Simulator::CallbackReturn::ERROR;

    }

    if (!(joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.command_interfaces[1].name == hardware_interface::HW_IF_VELOCITY))
    {
      RCLCPP_ERROR(rclcpp::get_logger("Simulator"),
                   "Joint '%s' needs the following command interfaces in that order: %s, %s.",
                   joint.name.c_str(), hardware_interface::HW_IF_POSITION,
                   hardware_interface::HW_IF_VELOCITY);

      return Simulator::CallbackReturn::ERROR;

    }

    if (joint.state_interfaces.size() != 3)
    {
      RCLCPP_ERROR(rclcpp::get_logger("Simulator"), "Joint '%s' needs 3 state interfaces.",
                   joint.name.c_str());

      return Simulator::CallbackReturn::ERROR;

    }

    if (!(joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_EFFORT))
    {
      RCLCPP_ERROR(rclcpp::get_logger("Simulator"),
                   "Joint '%s' needs the following state interfaces in that order: %s, %s, and %s.",
                   joint.name.c_str(), hardware_interface::HW_IF_POSITION,
                   hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_EFFORT);

      return Simulator::CallbackReturn::ERROR;

    }
  }

  return Simulator::CallbackReturn::SUCCESS;

}

std::vector<hardware_interface::StateInterface> Simulator::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &m_positions[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &m_velocities[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &m_efforts[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Simulator::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &m_position_commands[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &m_velocity_commands[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, cartesian_controller_simulation::HW_IF_STIFFNESS, &m_stiffness[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, cartesian_controller_simulation::HW_IF_DAMPING, &m_damping[i]));
  }

  return command_interfaces;
}

Simulator::return_type Simulator::prepare_command_mode_switch(
  [[maybe_unused]] const std::vector<std::string> & start_interfaces,
  [[maybe_unused]] const std::vector<std::string> & stop_interfaces)
{
  // TODO: Exclusive OR for position and velocity commands

  return return_type::OK;
}




Simulator::return_type Simulator::read([[maybe_unused]] const rclcpp::Time & time,
                                       [[maybe_unused]] const rclcpp::Duration & period)

{
  MuJoCoSimulator::getInstance().read(m_positions, m_velocities, m_efforts);

  // Start with the current positions as safe default, but let active
  // controllers overrride them in each cycle.
  if (std::any_of(m_position_commands.begin(), m_position_commands.end(),
                  [](double i) { return std::isnan(i); }))
  {
    m_position_commands = m_positions;
  }

  // TODO: Reconnect the MuJoCo simulator once we have implemented joint control with
  // - Gravity compensation
  // - integral gains for steady state accuracy
  m_positions = m_position_commands;
  m_velocities = m_velocity_commands;

  return return_type::OK;
}


Simulator::return_type Simulator::write([[maybe_unused]] const rclcpp::Time & time,
                                        [[maybe_unused]] const rclcpp::Duration & period)

{
  MuJoCoSimulator::getInstance().write(m_position_commands, m_velocity_commands, m_stiffness,
                                       m_damping);
  return return_type::OK;
}

}  // namespace cartesian_controller_simulation

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(cartesian_controller_simulation::Simulator,
                       hardware_interface::SystemInterface)
