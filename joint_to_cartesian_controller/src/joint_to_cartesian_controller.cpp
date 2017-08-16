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
  return true;
}

void JointToCartesianController::starting(const ros::Time& time)
{
}

void JointToCartesianController::stopping(const ros::Time& time)
{
}

void JointToCartesianController::update(const ros::Time& time, const ros::Duration& period)
{
}

}
