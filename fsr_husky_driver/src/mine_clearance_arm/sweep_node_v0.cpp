/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, ISR University of Coimbra.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Goncalo Cabrita on 16/10/2013
* Updated: Baptiste Gil on 08/09/2014
*********************************************************************/

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>

#define ARM_LENGTH 1.06
#define COIL_LENGTH 0.62
#define COIL_WIDTH 0.16

ros::Subscriber msg_sub_;
ros::Subscriber odo_vel_sub_;

 double hoffset = 0.0;
 double increment=0.0;
 bool flag_sweep = false;
 double sweep_speed_;
 double min_sweep_speed_;
 double min_alpha_sweep;
 double max_alpha_sweep;
 double max_vel_metal_det;


void odo_velCallback(const nav_msgs::Odometry& msg)
{
    double linear_speed = msg.twist.twist.linear.x;
    double alpha_sweep = max_alpha_sweep - min_alpha_sweep; //total sweep angle

    double max_dist_robot =  COIL_WIDTH * sin(alpha_sweep/2) + COIL_LENGTH * cos (alpha_sweep/2); //max distance robot can move

    sweep_speed_ = (linear_speed * alpha_sweep/ max_dist_robot);

    ROS_INFO("lin_vel - %f - max_dist - %f", linear_speed, max_dist_robot);


    if( sweep_speed_ < min_sweep_speed_)
        sweep_speed_ = min_sweep_speed_;
    else
        if (sqrt(pow(linear_speed,2)+pow(sweep_speed_*ARM_LENGTH,2))> max_vel_metal_det){
            double max_robo_vel=max_vel_metal_det/(sqrt(1+pow(ARM_LENGTH * alpha_sweep/ max_dist_robot,2)));
            sweep_speed_ = sqrt(pow(max_vel_metal_det,2)-pow(max_robo_vel,2))/ARM_LENGTH;
            ROS_INFO("Sweep speed out of range!");
        }

    ROS_INFO("Sweep speed (rad/s) - %f - Sweep (m/s) - %f", sweep_speed_, sqrt(pow(linear_speed,2)+pow(sweep_speed_*ARM_LENGTH,2)));

}

 void msgCallback(const std_msgs::String& msg)
 {
   if(msg.data == "start")
   {
       ROS_INFO("Sweep initiated!");
       flag_sweep=true;
   }
   else if(msg.data == "stop")
   {
        ROS_INFO("Sweep paused!");
        flag_sweep=false;
   }

 }


int main(int argc, char **argv)
{
    ros::init(argc, argv, "sweep_node");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    msg_sub_ = n.subscribe("sweep_state", 1, msgCallback);

    pn.param("increment", increment, 0.005);
    pn.param("min", min_alpha_sweep, -0.9);
    pn.param("max", max_alpha_sweep, 0.9);
    double height;
    pn.param("height", height, -0.10);
    pn.param("min_sweep_speed", min_sweep_speed_, 0.2);    // speed of sweep (rad/s)
    pn.param("max_vel_metal_det", max_vel_metal_det, 1.0); // maximum velocity of metal detector (m/s)
    //    pn.param("speed", sweep_speed_, 0.2);
    sweep_speed_ = min_sweep_speed_;
    double acceleration;
    pn.param("acceleration", acceleration, 2.8);
    bool sweep_speed_control;
    n.param("set_sweep_speed_control", sweep_speed_control, true);

    if (sweep_speed_control);
    odo_vel_sub_ = n.subscribe("encoder_corrected", 1, odo_velCallback);

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("/arm_controller/follow_joint_trajectory", true);
    ROS_INFO("Sweep -- Waiting for the action server to start...");
    ac.waitForServer();
    ROS_INFO("Sweep -- Got it!");

    //Function to calculate sweep_speed

    double position = min_alpha_sweep;

    while(ros::ok())
    {
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory.header.stamp = ros::Time::now();
        goal.trajectory.points.resize(1);
        goal.trajectory.joint_names.push_back("upper_arm_joint");
        if(flag_sweep)
            goal.trajectory.points[0].positions.push_back(height+hoffset);
        else
            goal.trajectory.points[0].positions.push_back(0.0);
        goal.trajectory.points[0].velocities.push_back(0.0);
        goal.trajectory.points[0].accelerations.push_back(0.05);
        goal.trajectory.joint_names.push_back("arm_axel_joint");
        if(flag_sweep)
            goal.trajectory.points[0].positions.push_back(position = position == min_alpha_sweep ? max_alpha_sweep : min_alpha_sweep);
        else
            goal.trajectory.points[0].positions.push_back(position = position == min_alpha_sweep ? min_alpha_sweep : max_alpha_sweep);
        goal.trajectory.points[0].velocities.push_back(sweep_speed_);
        goal.trajectory.points[0].accelerations.push_back(acceleration);

        goal.path_tolerance.resize(2);
        goal.path_tolerance[0].name = "upper_arm_joint";
        goal.path_tolerance[0].position = -1;
        goal.path_tolerance[1].name = "arm_axel_joint";
        goal.path_tolerance[1].position = -1;

        goal.goal_tolerance.resize(2);
        goal.goal_tolerance[0].name = "upper_arm_joint";
        goal.goal_tolerance[0].position = 0.05;
        goal.goal_tolerance[1].name = "arm_axel_joint";
        goal.goal_tolerance[1].position = 0.05;

        goal.goal_time_tolerance = ros::Duration(30.0);

        ac.sendGoal(goal);

        // Wait for the action to return
        bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

        if(!finished_before_timeout)
        {
            actionlib::SimpleClientGoalState state = ac.getState();
            ROS_ERROR("Sweep -- %s!", state.toString().c_str());
        }
        ros::spinOnce();
    }

    return 0;
}
