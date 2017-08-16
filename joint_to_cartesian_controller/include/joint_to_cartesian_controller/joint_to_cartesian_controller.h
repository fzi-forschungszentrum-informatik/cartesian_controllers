// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    joint_to_cartesian_controller.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/08/15
 *
 */
//-----------------------------------------------------------------------------

#ifndef JOINT_TO_CARTESIAN_CONTROLLER_H_INCLUDED
#define JOINT_TO_CARTESIAN_CONTROLLER_H_INCLUDED

// Project
#include <joint_to_cartesian_controller/JointControllerAdapter.h>

// ROS
#include <ros/ros.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <trajectory_msgs/JointTrajectory.h>

// ros_controls
#include <controller_interface/controller.h>
#include <hardware_interface/joint_state_interface.h>
#include <controller_manager/controller_manager.h>

// KDL
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

// tf
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

namespace joint_to_cartesian_controller
{

class JointToCartesianController
  : public controller_interface::Controller<hardware_interface::JointStateInterface>
{
  public:
    JointToCartesianController();

    bool init(hardware_interface::JointStateInterface* hw, ros::NodeHandle& nh);

    void starting(const ros::Time& time);

    void stopping(const ros::Time& time);

    void update(const ros::Time& time, const ros::Duration& period);

  private:
    std::string                m_end_effector_link;
    std::string                m_robot_base_link;
    std::string                m_target_name;
    KDL::JntArray              m_positions;
    KDL::JntArray              m_velocities;
    std::vector<std::string>   m_joint_names;
    tf::TransformBroadcaster   m_tf_broadcaster;
    ros::Publisher             m_controller_state_publisher;
    ros::Subscriber            m_controller_command_subscriber;

    JointControllerAdapter     m_controller_adapter;

    std::vector<
      hardware_interface::JointStateHandle>   m_joint_handles;

    boost::shared_ptr<
      KDL::ChainFkSolverPos_recursive>        m_fk_solver;

    boost::shared_ptr<
      controller_manager::ControllerManager>  m_controller_manager;


    void controllerCommandCallback(const trajectory_msgs::JointTrajectory& cmd);
};

}

#endif
