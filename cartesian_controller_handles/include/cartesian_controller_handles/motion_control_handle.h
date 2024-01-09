////////////////////////////////////////////////////////////////////////////////
// Copyright 2019 FZI Research Center for Information Technology
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
/*!\file    motion_control_handle.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/11/06
 *
 */
//-----------------------------------------------------------------------------

#ifndef MOTION_CONTROL_HANDLE_H_INCLUDED
#define MOTION_CONTROL_HANDLE_H_INCLUDED

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "cartesian_controller_base/ROS2VersionConfig.h"
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/publisher.hpp"
#include "visualization_msgs/msg/interactive_marker.hpp"
#include "visualization_msgs/msg/interactive_marker_feedback.hpp"

namespace cartesian_controller_handles
{
/**
 * @brief Implements a drag-and-drop control handle in RViz
 *
 * This motion control handle can be used to control the end effector of a
 * robotic manipulator in Cartesian space with an interactive marker.
 * A common use case is quick teaching of Cartesian motion.
 * The motion of the marker (both orientation and translation) is published as a
 * geometry_msgs/PoseStamped, which can be followed by motion-based Cartesian
 * controllers, such as the \ref CartesianMotionController or the \ref
 * CartesianComplianceController.
 */
class MotionControlHandle : public controller_interface::ControllerInterface
{
public:
  MotionControlHandle();
  ~MotionControlHandle();

#if defined CARTESIAN_CONTROLLERS_GALACTIC || defined CARTESIAN_CONTROLLERS_HUMBLE || \
  defined CARTESIAN_CONTROLLERS_IRON
  virtual LifecycleNodeInterface::CallbackReturn on_init() override;
#elif defined CARTESIAN_CONTROLLERS_FOXY
  virtual controller_interface::return_type init(const std::string & controller_name) override;
#endif

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

#if defined CARTESIAN_CONTROLLERS_GALACTIC || defined CARTESIAN_CONTROLLERS_HUMBLE || \
  defined CARTESIAN_CONTROLLERS_IRON
  controller_interface::return_type update(const rclcpp::Time & time,
                                           const rclcpp::Duration & period) override;
#elif defined CARTESIAN_CONTROLLERS_FOXY
  controller_interface::return_type update() override;
#endif

private:
  /**
     * @brief Move visual marker in RViz according to user interaction
     *
     * This function also stores the marker pose internally.
     *
     * @param feedback The message containing the current pose of the marker
     */
  void updateMotionControlCallback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback);

  /**
     * @brief React to changes in the interactive marker menu
     *
     * @param feedback The message containing the current menu configuration
     */
  void updateMarkerMenuCallback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback);

  /**
     * @brief Add all relevant marker controls for interaction in RViz
     *
     * You must call \a applyChanges() on the marker server for the controls to
     * take effect.
     *
     * @param marker The marker to add the controls to
     */
  static void prepareMarkerControls(visualization_msgs::msg::InteractiveMarker & marker);

  /**
     * @brief Adds interactive controls (arrows) to a marker.
     *
     * Both move and rotate controls are added along the specified
     * axis. The axis is normalized.
     *
     * @param marker The marker to which the controls are added
     * @param x X-axis component
     * @param y Y-axis component
     * @param z Z-axis component
     */
  static void addAxisControl(visualization_msgs::msg::InteractiveMarker & marker, double x,
                             double y, double z);

  /**
     * @brief Add a sphere visualization to the interactive marker
     *
     * @param marker The marker to add the visualization to
     * @param scale The scale of the visualization. Bounding box in meter.
     */
  static void addMarkerVisualization(visualization_msgs::msg::InteractiveMarker & marker,
                                     double scale);

  /**
     * @brief Get the current pose of the specified end-effector
     *
     * @return The current end-effector pose with respect to the specified base link
     */
  geometry_msgs::msg::PoseStamped getEndEffectorPose();

  // Handles to the joints
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> m_joint_handles;

  std::vector<std::string> m_joint_names;

  // Kinematics
  std::string m_robot_base_link;
  std::string m_end_effector_link;
  std::string m_target_frame_topic;
  KDL::Chain m_robot_chain;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> m_fk_solver;

  geometry_msgs::msg::PoseStamped m_current_pose;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr m_pose_publisher;

  // Interactive marker
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> m_server;

  visualization_msgs::msg::InteractiveMarker m_marker;  //!< Controller handle for RViz
};

}  // namespace cartesian_controller_handles

#endif
