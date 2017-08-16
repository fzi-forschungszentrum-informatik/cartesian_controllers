// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    joint_to_cartesian_controller.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/08/15
 *
 */
//-----------------------------------------------------------------------------

// Pluginlib
#include <pluginlib/class_list_macros.h>

// Project
#include <joint_to_cartesian_controller/joint_to_cartesian_controller.h>
#include <cartesian_controller_base/Utility.h>

// KDL
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

// URDF
#include <urdf/model.h>

// tf
#include <tf_conversions/tf_kdl.h>

// Other
#include <map>

namespace cartesian_controllers
{
  /**
   * @brief Turn joint positions into end-effector pose
   *
   * The name JointTrajectoryController is somewhat misleading for this
   * controller, since it only turns joint positions into end-effector poses
   * without interpolation. The only reason for this is, that having the
   * substring JointTrajectoryController in the loaded controller type assures
   * compatibility with the rqt joint trajectory controller plugin, which
   * checks for this substring when looking for controllers.
   */
  typedef joint_to_cartesian_controller::JointToCartesianController JointTrajectoryController;
}

PLUGINLIB_EXPORT_CLASS(cartesian_controllers::JointTrajectoryController, controller_interface::ControllerBase)






namespace joint_to_cartesian_controller
{

JointToCartesianController::JointToCartesianController()
{
}

bool JointToCartesianController::init(hardware_interface::JointStateInterface* hw, ros::NodeHandle& nh)
{
  std::string robot_description;
  urdf::Model robot_model;
  KDL::Tree   robot_tree;
  KDL::Chain  robot_chain;

  // Get controller specific configuration
  if (!nh.getParam("/robot_description",robot_description))
  {
    ROS_ERROR("Failed to load '/robot_description' from parameter server");
    return false;
  }
  if (!nh.getParam("robot_base_link",m_robot_base_link))
  {
    ROS_ERROR_STREAM("Failed to load " << nh.getNamespace() + "/robot_base_link" << " from parameter server");
    return false;
  }
  if (!nh.getParam("end_effector_link",m_end_effector_link))
  {
    ROS_ERROR_STREAM("Failed to load " << nh.getNamespace() + "/end_effector_link" << " from parameter server");
    return false;
  }
  if (!nh.getParam("published_target_name",m_target_name))
  {
    ROS_ERROR_STREAM("Failed to load " << nh.getNamespace() + "/published_target_name" << " from parameter server");
    return false;
  }

  // Build a kinematic chain of the robot
  if (!robot_model.initString(robot_description))
  {
    ROS_ERROR("Failed to parse urdf model from 'robot_description'");
    return false;
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

  // Adjust joint buffers
  m_positions.data = ctrl::VectorND::Zero(m_joint_handles.size());
  m_velocities.data = ctrl::VectorND::Zero(m_joint_handles.size());

  // Initialize forward kinematics solver
  m_fk_solver.reset(new KDL::ChainFkSolverPos_recursive(robot_chain));

  // Initialize controller topics
  m_controller_state_publisher =
    nh.advertise<control_msgs::JointTrajectoryControllerState>("state",10);

  m_controller_command_subscriber =
    nh.subscribe("command",3,&JointToCartesianController::controllerCommandCallback,this);

  return true;
}

void JointToCartesianController::starting(const ros::Time& time)
{
  // Get current joint positions from hardware
  for (size_t i = 0; i < m_joint_handles.size(); ++i)
  {
    m_positions(i) = m_joint_handles[i].getPosition();
  }
}

void JointToCartesianController::stopping(const ros::Time& time)
{
}

void JointToCartesianController::update(const ros::Time& time, const ros::Duration& period)
{
  // Solve forward kinematics
  KDL::Frame frame;
  m_fk_solver->JntToCart(m_positions,frame);

  // Publish end-effector pose to tf
  tf::Transform frame_tf;
  tf::poseKDLToTF(frame,frame_tf);
  m_tf_broadcaster.sendTransform(
      tf::StampedTransform(
        frame_tf,
        ros::Time::now(),
        m_robot_base_link,
        m_target_name));

  // Publish controller state
  control_msgs::JointTrajectoryControllerState state;
  state.header.stamp = ros::Time::now();
  for (size_t i = 0; i < m_joint_names.size(); ++i)
  {
    state.joint_names.push_back(m_joint_names[i]);

    // Mimic ideal system response
    state.desired.positions.push_back(m_positions(i));
    state.actual.positions.push_back(m_positions(i));
    state.error.positions.push_back(0.0);

    state.desired.velocities.push_back(m_velocities(i));
    state.actual.velocities.push_back(m_velocities(i));
    state.error.velocities.push_back(0.0);
  }

  m_controller_state_publisher.publish(state);
}

void JointToCartesianController::controllerCommandCallback(
    const trajectory_msgs::JointTrajectory& cmd)
{
  // Only copy target commands of the first point
  std::map <std::string, double> name_pos_map;
  for (size_t i = 0; i < cmd.joint_names.size(); ++i)
  {
    name_pos_map[cmd.joint_names[i]] = cmd.points[0].positions[i];
  }
  std::map <std::string, double> name_vel_map;
  for (size_t i = 0; i < cmd.joint_names.size(); ++i)
  {
    name_vel_map[cmd.joint_names[i]] = cmd.points[0].velocities[i];
  }

  // Bring joint order in sequence
  for (size_t i = 0; i < m_joint_names.size(); ++i)
  {
    m_positions(i) = name_pos_map[m_joint_names[i]];
    m_velocities(i) = name_vel_map[m_joint_names[i]];
  }
}

}
