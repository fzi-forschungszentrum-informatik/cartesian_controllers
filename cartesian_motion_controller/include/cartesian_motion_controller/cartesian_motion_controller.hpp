// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    cartesian_motion_controller.hpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/27
 *
 */
//-----------------------------------------------------------------------------

#ifndef CARTESIAN_MOTION_CONTROLLER_HPP_INCLUDED
#define CARTESIAN_MOTION_CONTROLLER_HPP_INCLUDED

// Project
#include <cartesian_motion_controller/cartesian_motion_controller.h>

// KDL
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

// URDF
#include <urdf/model.h>

// Other
#include <boost/algorithm/clamp.hpp>

namespace cartesian_motion_controller
{

template <class HardwareInterface>
CartesianMotionController<HardwareInterface>::
CartesianMotionController()
{
}

template <class HardwareInterface>
bool CartesianMotionController<HardwareInterface>::
init(HardwareInterface* hw, ros::NodeHandle& nh)
{
  std::string robot_description;
  urdf::Model robot_model;
  KDL::Tree   robot_tree;
  KDL::Chain  robot_chain;

  // Get controller specific configuration
  if (!nh.getParam("/robot_description",robot_description))
  {
    ROS_ERROR("Failed to load '/robot_description' from parameter server");
  }
  if (!nh.getParam("robot_base_link",m_robot_base_link))
  {
    ROS_ERROR_STREAM("Failed to load " << nh.getNamespace() + "/robot_base_link" << " from parameter server");
  }
  if (!nh.getParam("end_effector_link",m_end_effector_link))
  {
    ROS_ERROR_STREAM("Failed to load " << nh.getNamespace() + "/end_effector_link" << " from parameter server");
  }
  if (!nh.getParam("target_frame",m_target_frame))
  {
    ROS_ERROR_STREAM("Failed to load " << nh.getNamespace() + "/target_frame" << " from parameter server");
  }

  // Build a kinematic chain of the robot
  if (!robot_model.initString(robot_description))
  {
    ROS_ERROR("Failed to parse urdf model from 'robot_description'");
  }
  if (!kdl_parser::treeFromUrdfModel(robot_model,robot_tree))
  {
    const std::string error = ""
      "Failed to parse KDL tree from urdf model";
    ROS_ERROR_STREAM(error);
    throw std::runtime_error(error);
  }
  if (!robot_tree.getChain(m_robot_base_link,m_end_effector_link,robot_chain))
  {
    const std::string error = ""
      "Failed to parse robot chain from urdf model. "
      "Are you sure that both your 'robot_base_link' and 'end_effector_link' exist?";
    ROS_ERROR_STREAM(error);
    throw std::runtime_error(error);
  }

  // Get names of controllable joints from the parameter server
  if (!nh.getParam("joints",m_joint_names))
  {
    const std::string error = ""
    "Failed to load " + nh.getNamespace() + "/joints" + " from parameter server";
    ROS_ERROR_STREAM(error);
    throw std::runtime_error(error);
  }

  // Get the joint handles to use in the control loop
  for (size_t i = 0; i < m_joint_names.size(); ++i)
  {
    m_joint_handles.push_back(hw->getHandle(m_joint_names[i]));
  }

  // Initialize solver
  m_forward_dynamics_solver.init(robot_chain);

  // Initialize Cartesian pid controllers
  controlMotionError.init(nh);



  return true;
}

template <class HardwareInterface>
void CartesianMotionController<HardwareInterface>::
starting(const ros::Time& time)
{
  // Copy joint state to internal simulation
  m_forward_dynamics_solver.setStartState(m_joint_handles);
}

template <class HardwareInterface>
void CartesianMotionController<HardwareInterface>::
stopping(const ros::Time&)
{
}

template <class HardwareInterface>
void CartesianMotionController<HardwareInterface>::
update(const ros::Time& time, const ros::Duration& period)
{
  // Forward Dynamics turns the search for the according joint motion into a
  // control process. So, we control the internal model until we meet the
  // Cartesian target motion. This internal control needs some simulation time
  // steps.
  const int steps = 10;
  for (int i = 0; i < steps; ++i)
  {
    // The internal 'simulation time' is deliberately independent of the outer
    // control cycle.
    ros::Duration internal_period(0.02);

    // Compute the motion error = target - current.
    ctrl::Vector6D error = computeMotionError();

    // Compute system input from Cartesian motion error
    ctrl::Vector6D cartesian_input = controlMotionError(error,internal_period);

    // Turn Cartesian control command into joint motion
    computeJointControlCmds(cartesian_input,internal_period,m_simulated_joint_motion);
  }

  // Write final commands to the hardware interface
  writeJointControlCmds(m_simulated_joint_motion);
}

template <>
void CartesianMotionController<hardware_interface::PositionJointInterface>::
writeJointControlCmds(const trajectory_msgs::JointTrajectoryPoint& motion)
{
  // Take position commands
  for (size_t i = 0; i < m_joint_handles.size(); ++i)
  {
    m_joint_handles[i].setCommand(motion.positions[i]);
  }
}

template <>
void CartesianMotionController<hardware_interface::VelocityJointInterface>::
writeJointControlCmds(const trajectory_msgs::JointTrajectoryPoint& motion)
{
  // Take velocity commands
  for (size_t i = 0; i < m_joint_handles.size(); ++i)
  {
    m_joint_handles[i].setCommand(motion.velocities[i]);
  }
}

template <class HardwareInterface>
void CartesianMotionController<HardwareInterface>::
computeJointControlCmds(
        const ctrl::Vector6D& cartesian_input,
        const ros::Duration& period,
        trajectory_msgs::JointTrajectoryPoint& joint_output)
{
  joint_output = m_forward_dynamics_solver.getJointControlCmds(
      period,
      cartesian_input);
}

template <class HardwareInterface>
ctrl::Vector6D CartesianMotionController<HardwareInterface>::
computeMotionError()
{
  // Compute motion error wrt robot_base_link
  tf::StampedTransform current_pose = m_forward_dynamics_solver.getEndEffectorPose();
  tf::StampedTransform target_pose;

  try
  {
    /* // This blocks eternally, but why?

    m_tf_listener.waitForTransform( // returns immediately if frames exist
        m_robot_base_link,
        m_target_frame,
        ros::Time(0),   // anyone will be ok
        ros::Duration(0.1)
        );
    */

    // This lookupTransform() version really uses the latest transform and
    // doesn't limit the lookup to the last common time.
    m_tf_listener.lookupTransform(
        m_robot_base_link,  // I want my pose displayed in this frame
        ros::Time(0),
        m_target_frame,
        ros::Time(0),
        m_robot_base_link,  // Const reference
        target_pose);
  }
  catch (tf::TransformException& e)
  {
    ROS_WARN_THROTTLE(3,"CartesianMotionController: %s",e.what());
    return ctrl::Vector6D::Zero();
  }

  // Use KDL math
  KDL::Frame current_pose_kdl;
  KDL::Frame target_pose_kdl;
  tf::transformTFToKDL(current_pose,current_pose_kdl);
  tf::transformTFToKDL(target_pose,target_pose_kdl);

  // Transformation from target -> current corresponds to error = target - current
  KDL::Frame error_kdl = target_pose_kdl * current_pose_kdl.Inverse();

  // Use Rodrigues Vector for a compact representation of orientation errors
  // Only for angles within [0,Pi)
  KDL::Vector rot_axis = KDL::Vector::Zero();
  double angle    = error_kdl.M.GetRotAngle(rot_axis);   // rot_axis is normalized
  double distance = error_kdl.p.Normalize();

  // Clamp maximal tolerated error.
  // The remaining error will be handled in the next control cycle.
  const double max_angle = 1.0;
  const double max_distance = 1.0;
  angle    = boost::algorithm::clamp(angle,-max_angle,max_angle);
  distance = boost::algorithm::clamp(distance,-max_distance,max_distance);

  // Scale errors to allowed magnitudes
  rot_axis = rot_axis * angle;
  error_kdl.p = error_kdl.p * distance;

  // Reassign values
  ctrl::Vector6D error;
  error(0) = error_kdl.p.x();
  error(1) = error_kdl.p.y();
  error(2) = error_kdl.p.z();
  error(3) = rot_axis(0);
  error(4) = rot_axis(1);
  error(5) = rot_axis(2);

  return error;
}

} // namespace

#endif
