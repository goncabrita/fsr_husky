/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, ISR University of Coimbra.
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
* Author: Gonçalo Cabrita on 28/02/2014
*
*********************************************************************/

#include <ros/ros.h>
#include <laser_assembler/AssembleScans2.h>

/*#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>*/
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cloud_assember");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    double tilt_speed;
    pn.param("tilt_speed", tilt_speed, 1.17);

    double lower_tilt;
    pn.param("lower_tilt", lower_tilt, -0.6);

    double upper_tilt;
    pn.param("upper_tilt", upper_tilt, 0.0);

    double timeout;
    pn.param("timeout", timeout, 10.0);

   double sleeptime;
   pn.param("sleep_interval",sleeptime,0.5);

    //actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> tilt_ac("/ptu_controller/follow_joint_trajectory", true);
    //ROS_INFO("PointCloud Generator -- Waiting for tilt action server to start...");
    //tilt_ac.waitForServer();
    /*ROS_INFO("PointCloud Generator -- Got it!");
    ros::topic::waitForMessage*/



    ROS_INFO("PointCloud Generator -- Waiting for laser assembler service to start...");
    ros::service::waitForService("assemble_scans2");
    ROS_INFO("PointCloud Generator -- Got it!");

    ros::ServiceClient client = n.serviceClient<laser_assembler::AssembleScans2>("assemble_scans2");
    laser_assembler::AssembleScans2 assembler_srv;

    ros::Publisher tilt_sub = n.advertise<std_msgs::Float64>("ptu_d46_tilt_controller/command", 1);

    ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud2>("assembled_cloud", 1);

    // Move laser to the start position
    double tilt = upper_tilt;

    /*control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.header.stamp = ros::Time::now();
    goal.trajectory.joint_names.resize(1);
    goal.trajectory.points.resize(1);
    goal.trajectory.joint_names[0] = "ptu_d46_tilt_joint";
    goal.trajectory.points[0].positions.push_back(tilt);
    goal.trajectory.points[0].velocities.push_back(tilt_speed);
    goal.trajectory.points[0].accelerations.push_back(0.5);
    goal.trajectory.points[0].effort.push_back(0.0);
    goal.trajectory.points[0].time_from_start = ros::Duration(2.0);
    goal.path_tolerance.resize(1);
    goal.path_tolerance[0].name = "ptu_d46_tilt_joint";
    goal.path_tolerance[0].position = -1;

    goal.goal_tolerance.resize(1);
    goal.goal_tolerance[0].name = "ptu_d46_tilt_joint";
    goal.goal_tolerance[0].position = 0.01;

    goal.goal_time_tolerance = ros::Duration(2.0);

    tilt_ac.sendGoal(goal);*/


    std_msgs::Float64 pos;
    pos.data=0.0;

    tilt_sub.publish(pos);


    ros::Duration(2.0).sleep();

    while(ros::ok())
    {
        assembler_srv.request.begin = ros::Time::now();

        pos.data = (tilt = (tilt == upper_tilt) ? lower_tilt : upper_tilt);
        tilt_sub.publish(pos);

        ros::Duration(sleeptime).sleep();

         assembler_srv.request.end = ros::Time::now();

         if(client.call(assembler_srv))
            {
                cloud_pub.publish(assembler_srv.response.cloud);
            }
            else
            {
                ROS_WARN("PointCloud Generator - Service call failed!");
            }
    }

    return 0;
}
