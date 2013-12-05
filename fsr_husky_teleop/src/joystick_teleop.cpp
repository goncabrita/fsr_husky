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
* Author: Gonçalo Cabrita on 06/06/2013
* Copied from the Squirtle teleop ROS package
*********************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"

class FSRHuskyTeleop
{
public:
  FSRHuskyTeleop();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void velCallback(const geometry_msgs::Twist::ConstPtr& vel);
  void jointCallback(const sensor_msgs::JointState::ConstPtr& joint);
  void publish();

  ros::NodeHandle ph_, nh_;

  // Axis
  int robot_linear_;
  int robot_angular_;
  int pan_;
  int tilt_;
  int robot_dead_man_switch_;
  int pan_and_tilt_trigger_;
  // Buttons
  int arm_button_;
  int reset_button_;
  int brake_button_;

  double l_scale_;
  double a_scale_;

  double upper_pan_limit_;
  double lower_pan_limit_;
  double upper_tilt_limit_;
  double lower_tilt_limit_;
  double pan_speed_;
  double tilt_speed_;

  ros::Publisher vel_pub_;
  ros::Publisher pan_and_tilt_pub_;
  ros::Publisher arm_pub_;
  ros::Subscriber vel_sub_;
  ros::Subscriber joy_sub_;
  ros::Subscriber joint_sub_;

  bool dead_man_switch_pressed_;
  bool brake_button_pressed_;

  bool mux_;

  bool pan_and_tilt_moving_;
  bool arm_moving_;

  geometry_msgs::Twist vel_;

  sensor_msgs::JointState joints_;

  ros::Timer timer_;
};

FSRHuskyTeleop::FSRHuskyTeleop(): ph_("~")
{
  ph_.param("robot_linear_axis", robot_linear_, 1);
  ph_.param("robot_angular_axis", robot_angular_, 0);
  ph_.param("dead_man_switch", robot_dead_man_switch_, 2);
  ph_.param("pan_axis", pan_, 3);
  ph_.param("tilt_axis", tilt_, 4);
  ph_.param("pan_and_tilt_trigger", pan_and_tilt_trigger_, 5);
  ph_.param("arm_button", arm_button_, 5);
  ph_.param("reset_button", reset_button_, 10);
  ph_.param("brake_button", brake_button_, 1);
  ph_.param("scale_angular", a_scale_, 0.9);
  ph_.param("scale_linear", l_scale_, 0.3);

  ph_.param("tilt_speed", tilt_speed_, 0.5);
  ph_.param("lower_tilt_limit", lower_tilt_limit_, -0.5);
  ph_.param("upper_tilt_limit", upper_tilt_limit_, 0.5);

  ph_.param("pan_speed", pan_speed_, 0.5);
  ph_.param("lower_pan_limit", lower_pan_limit_, -0.5);
  ph_.param("upper_pan_limit", upper_pan_limit_, 0.5);

  ph_.param("multiplex", mux_, false);

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("husky/cmd_vel", 1);
  pan_and_tilt_pub_ = nh_.advertise<sensor_msgs::JointState>("ptu_d46/cmd", 1);
  arm_pub_ = nh_.advertise<sensor_msgs::JointState>("husky_arm/cmd", 1);
  if(mux_) vel_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 10, &FSRHuskyTeleop::velCallback, this);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &FSRHuskyTeleop::joyCallback, this);
  joint_sub_ = nh_.subscribe<sensor_msgs::JointState>("joint_states", 10, &FSRHuskyTeleop::jointCallback, this);

  dead_man_switch_pressed_ = false;
  brake_button_pressed_ = false;

  pan_and_tilt_moving_ = false;
  arm_moving_ = false;

  vel_.angular.z = 0.0;
  vel_.linear.x = 0.0;

  joints_.name.resize(4);
  joints_.position.resize(4);
  joints_.velocity.resize(4);
  joints_.name[0] = "ptu_d46_pan_joint";
  joints_.name[1] = "ptu_d46_tilt_joint";
  joints_.name[2] = "arm_vertical_axis_joint";
  joints_.name[3] = "linear_actuator_joint";

  timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&FSRHuskyTeleop::publish, this));
}

void FSRHuskyTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{ 
  if(joy->axes[robot_dead_man_switch_] == -1.0) dead_man_switch_pressed_ = true;
  else dead_man_switch_pressed_ = false;

  if(dead_man_switch_pressed_)
  {
    vel_.angular.z = a_scale_*joy->axes[robot_angular_];
    vel_.linear.x = l_scale_*joy->axes[robot_linear_];
  }

  if(joy->buttons[brake_button_] == 1) brake_button_pressed_ = true;
  else brake_button_pressed_ = false;

  sensor_msgs::JointState joint;
  joint.header.stamp = ros::Time::now();
  joint.name.resize(2);
  joint.position.resize(2);
  joint.velocity.resize(2);
  joint.name[0] = "ptu_d46_pan_joint";
  joint.position[0] = 0.0;
  joint.velocity[0] = pan_speed_;
  joint.name[1] = "ptu_d46_tilt_joint";
  joint.position[1] = 0.0;
  joint.velocity[1] = tilt_speed_;

  double publish = false;

  if(joy->axes[pan_and_tilt_trigger_] == -1.0)
  {
        if(joy->buttons[reset_button_] == 1)
        {
            pan_speed_ = 0.5;
            tilt_speed_ = 0.5;

            pan_and_tilt_moving_ = false;
            publish = true;
        }
        else
        {
            if(pan_and_tilt_moving_ && (fabs(joy->axes[pan_]) < 0.2 && fabs(joy->axes[tilt_]) < 0.2))
            {
                joint.position[0] = joints_.position[0];
                joint.position[1] = joints_.position[1];

                pan_and_tilt_moving_ = false;
                publish = true;
            }
            else if(fabs(joy->axes[pan_]) > 0.2 || fabs(joy->axes[tilt_]) > 0.2)
            {
                if(fabs(joy->axes[pan_]) > 0.2) joint.position[0] = joy->axes[pan_] > 0 ? upper_pan_limit_ : lower_pan_limit_;
                if(fabs(joy->axes[tilt_]) > 0.2) joint.position[1] = joy->axes[tilt_] < 0 ? upper_tilt_limit_ : lower_tilt_limit_;

                pan_speed_ = 0.7*fabs(joy->axes[pan_]);
                tilt_speed_ = 0.7*fabs(joy->axes[tilt_]);

                pan_and_tilt_moving_ = true;
                publish = true;
            }
        }
  }
  else if(pan_and_tilt_moving_)
  {
      joint.position[0] = joints_.position[0];
      joint.position[1] = joints_.position[1];

      pan_and_tilt_moving_ = false;
      publish = true;
  }
  if(publish) pan_and_tilt_pub_.publish(joint);

  if(joy->buttons[arm_button_] == 1)
  {
    // TODO...
  }
}

void FSRHuskyTeleop::jointCallback(const sensor_msgs::JointState::ConstPtr& joint)
{
    for(int i=0 ; i<joint->name.size() ; i++)
    {
        if(joint->name[i].compare("ptu_d46_pan_joint") == 0) joints_.position[0] = joint->position[i];
        else if(joint->name[i].compare("ptu_d46_tilt_joint") == 0) joints_.position[1] = joint->position[i];
        else if(joint->name[i].compare("arm_vertical_axis_joint") == 0) joints_.position[2] = joint->position[i];
        else if(joint->name[i].compare("linear_actuator_joint") == 0) joints_.position[3] = joint->position[i];
    }
}

void FSRHuskyTeleop::publish()
{
    //boost::mutex::scoped_lock lock(publish_mutex_);

    if((!dead_man_switch_pressed_ && (vel_.angular.z != 0.0 || vel_.linear.x != 0.0)) || brake_button_pressed_)
    {
        vel_.linear.x = 0.0;
        vel_.angular.z = 0.0;
        vel_pub_.publish(vel_);
    }
    else if(dead_man_switch_pressed_)
    {
        vel_pub_.publish(vel_);
    }
}

void FSRHuskyTeleop::velCallback(const geometry_msgs::Twist::ConstPtr& vel)
{
    //boost::mutex::scoped_lock lock(publish_mutex_);

    if(!dead_man_switch_pressed_ && !brake_button_pressed_)
    {
        vel_pub_.publish(*vel);
    }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "roomba_joystick_teleop");
  FSRHuskyTeleop roomba_teleop;

  ROS_INFO("FSR Husky Teleop Node");

  ros::spin();
}
