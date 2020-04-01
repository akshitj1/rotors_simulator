#ifndef ROTORS_CONTROL_TAILSITTER_CONTROLLER_NODE_H
#define ROTORS_CONTROL_TAILSITTER_CONTROLLER_NODE_H

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>

#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/AttitudeThrust.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Odometry.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "rotors_control/common.h"

namespace rotors_control
{

class TailsitterControllerNode
{
    ros::NodeHandle nh;
    ros::Subscriber odometry_sub;
    ros::Publisher motor_velocity_reference_pub;    


    TailsitterControllerNode(const ros::NodeHandle &_nh)
        : nh(_nh)
    {
        odometry_sub = nh.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                                    &OdometryCallback, this);

        motor_velocity_reference_pub = nh.advertise<mav_msgs::Actuators>(
            mav_msgs::default_topics::COMMAND_ACTUATORS, 1);
    }
    ~TailsitterControllerNode() {}

    void OdometryCallback(const nav_msgs::OdometryConstPtr &odometry_msg)
    {
        ROS_INFO_ONCE("TailsitterController got first odometry message.");

        EigenOdometry odometry;
        eigenOdometryFromMsg(odometry_msg, &odometry);

        Eigen::VectorXd ref_rotor_velocities;
        calcActuatorStates(ref_rotor_velocities);

        publishActuatorCmds(ref_rotor_velocities);
    }

    void publishActuatorCmds(const Eigen::VectorXd& ref_rotor_velocities)
    {
        mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

        actuator_msg->angular_velocities.clear();
        for (int i = 0; i < ref_rotor_velocities.size(); i++)
            actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
        // actuator_msg->header.stamp = odometry_msg->header.stamp;

        motor_velocity_reference_pub.publish(actuator_msg);
    }

    void calcActuatorStates(Eigen::VectorXd& ref_rotor_velocities) {
    }
};
} // namespace rotors_control

#endif