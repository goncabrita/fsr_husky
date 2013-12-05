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
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ptu_d46_driver/GotoAction.h>

class FSRHuskyTeleop
{
public:
  FSRHuskyTeleop();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void velCallback(const geometry_msgs::Twist::ConstPtr& vel);
  void publish();
  void goalDoneCallback(const actionlib::SimpleClientGoalState &state, const ptu_d46_driver::GotoResultConstPtr &result);

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

  double pan_increment_;
  double tilt_increment_;
  double upper_pan_limit_;
  double lower_pan_limit_;
  double upper_tilt_limit_;
  double lower_tilt_limit_;
  double pan_speed_;
  double tilt_speed_;

  int pan_action_;
  int tilt_action_;

  ros::Publisher vel_pub_;
  ros::Subscriber vel_sub_;
  ros::Subscriber joy_sub_;
  actionlib::SimpleActionClient<ptu_d46_driver::GotoAction> pan_and_tilt_ac_;

  bool dead_man_switch_pressed_;
  bool brake_button_pressed_;

  bool reset_pan_and_tilt_;
  bool reset_arm_;

  bool mux_;
  bool control_pan_and_tilt_;
  bool control_arm_;

  bool ready_for_pan_and_tilt_;
  bool ready_for_arm_;

  geometry_msgs::Twist vel_;
  ptu_d46_driver::GotoGoal goal_;

  ros::Timer timer_;
};

FSRHuskyTeleop::FSRHuskyTeleop(): ph_("~"), pan_and_tilt_ac_("ptu_d46", true)
{
  ph_.param("robot_linear_axis", robot_linear_, 1);
  ph_.param("robot_angular_axis", robot_angular_, 0);
  ph_.param("dead_man_switch", robot_dead_man_switch_, 2);
  ph_.param("scale_angular", a_scale_, 0.9);
  ph_.param("scale_linear", l_scale_, 0.3);
  ph_.param("pan_axis", pan_, 3);
  ph_.param("tilt_axis", tilt_, 4);
  ph_.param("pan_and_tilt_trigger", pan_and_tilt_trigger_, 5);
  ph_.param("arm_button", arm_button_, 5);
  ph_.param("reset_button", reset_button_, 10);
  ph_.param("brake_button", brake_button_, 1);

  ph_.param("tilt_increment", tilt_increment_, 0.1);
  ph_.param("tilt_speed", tilt_speed_, 0.8);
  ph_.param("lower_tilt_limit", lower_tilt_limit_, -0.5);
  ph_.param("upper_tilt_limit", upper_tilt_limit_, 0.5);

  ph_.param("pan_increment", pan_increment_, 0.1);
  ph_.param("pan_speed", pan_speed_, 0.8);
  ph_.param("lower_pan_limit", lower_pan_limit_, -0.5);
  ph_.param("upper_pan_limit", upper_pan_limit_, 0.5);

  ph_.param("multiplex", mux_, false);

  ph_.param("control_pan_and_tilt", control_pan_and_tilt_, false);
  if(control_pan_and_tilt_)
  {
      ROS_INFO("FSR Husky Teleop - %s - Waiting for the pan and tilt action server to start...", __FUNCTION__);
      pan_and_tilt_ac_.waitForServer();
      ROS_INFO("FSR Husky Teleop - %s - Got it!", __FUNCTION__);
  }

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/husky/cmd_vel", 1);
  if(mux_) vel_sub_ = nh_.subscribe<geometry_msgs::Twist>("cmd_vel", 10, &FSRHuskyTeleop::velCallback, this);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &FSRHuskyTeleop::joyCallback, this);

  ready_for_pan_and_tilt_ = true;
  ready_for_arm_ = true;

  dead_man_switch_pressed_ = false;
  brake_button_pressed_ = false;

  vel_.angular.z = 0.0;
  vel_.linear.x = 0.0;

  goal_.joint.header.stamp = ros::Time::now();
  goal_.joint.name.resize(2);
  goal_.joint.position.resize(2);
  goal_.joint.velocity.resize(2);
  goal_.joint.name[0] = "ptu_d46_pan_joint";
  goal_.joint.position[0] = 0.0;
  goal_.joint.velocity[0] = pan_speed_;
  goal_.joint.name[1] = "ptu_d46_tilt_joint";
  goal_.joint.position[1] = 0.0;
  goal_.joint.velocity[1] = tilt_speed_;

  timer_ = nh_.createTimer(ros::Duration(0.1), boost::bind(&RoombaTeleop::publish, this));
}

void FSRHuskyTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{ 
  if(joy->axes[robot_dead_man_switch_] == -1.0) dead_man_switch_pressed_ = true;
  else dead_man_switch_pressed_ = false;

  if(dead_man_switch_pressed_)
  {
    vel.angular.z = a_scale_*joy->axes[robot_angular_];
    vel.linear.x = l_scale_*joy->axes[robot_linear_];
  }

  if(joy->buttons[brake_button_] == 1) brake_button_pressed_ = true;
  else brake_button_pressed_ = false;

  if(control_pan_and_tilt_ && joy->axes[pan_and_tilt_trigger_] == -1.0)
  {
        if(joy->buttons[reset_button_] == 1)
        {
            goal_.joint.position[0] = 0.0;
            goal_.joint.position[1] = 0.0;
            ready_for_pan_and_tilt_ = false;
            pan_and_tilt_ac_.sendGoal(goal_, boost::bind(&FSRHuskyTeleop::goalDoneCallback, this, _1, _2));
        }
        else
        {
            pan_action_ = 0;
            tilt_action_ = 0;
            if(joy->axes[pan_] > 0.9 || joy->axes[pan_] < -0.9 || joy->axes[tilt_] > 0.9 || joy->axes[tilt_] < -0.9)
            {
                pan_action_ = joy->axes[pan_] > 0 ? 1 : -1;
                tilt_action = joy->axes[tilt_] < 0 ? 1 : -1;
            }

            if(ready_for_pan_and_tilt_)
            {
                goal_.joint.header.stamp = ros::Time::now();

                goal_.joint.position[0] += pan_action_*pan_increment_;
                goal_.joint.position[1] += tilt_action_*tilt_increment_;
                if(goal_.joint.position[0] > upper_pan_limit_) goal_.joint.position[0] = upper_pan_limit_;
                if(goal_.joint.position[0] < lower_pan_limit_) goal_.joint.position[0] = lower_pan_limit_;
                if(goal_.joint.position[1] > upper_tilt_limit_) goal_.joint.position[1] = upper_tilt_limit_;
                if(goal_.joint.position[1] < lower_tilt_limit_) goal_.joint.position[1] = lower_tilt_limit_;

                ready_for_pan_and_tilt_ = false;
                pan_and_tilt_ac_.sendGoal(goal_, boost::bind(&FSRHuskyTeleop::goalDoneCallback, this, _1, _2));
            }
        }

  }
  else if(control_arm_ && joy->buttons[arm_button_] == 1)
  {
    // TODO...
  }
}

void FSRHuskyTeleop::goalDoneCallback(const actionlib::SimpleClientGoalState &state, const ptu_d46_driver::GotoResultConstPtr &result)
{
    goal_.joint.position[0] = result->joint.position[0];
    goal_.joint.position[1] = result->joint.position[1];
    ready_for_pan_and_tilt_ = true;
}

void FSRHuskyTeleop::publish()
{
    //boost::mutex::scoped_lock lock(publish_mutex_);

    if((!dead_man_switch_pressed_ && (vel_.angular.z != 0.0 || vel_.angular.z != 0.0)) || brake_button_pressed_)
    {
        vel_.angular.z == 0.0;
        vel_.angular.z == 0.0;
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
