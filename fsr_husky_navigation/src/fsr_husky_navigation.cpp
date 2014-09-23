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
* Author: Baptiste Gil on 26/08/2014
*********************************************************************/

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/String.h"


#define ARM_LENGTH 1.06
#define HUSKY_LENGTH 1.0

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

ros::Publisher status_pub;

class Navigation
{
public:
    Navigation(ros::NodeHandle &n);

    ros::Publisher sweep_state_;

    void spinOnce(void);
    double sweep_alpha;

//    tf::TransformListener tf_;
//    ros::Subscriber ekf_pose_;


    move_base_msgs::MoveBaseGoal goal;

//   /* Corners positions */
//
//      4_____________3
//      |             |
//      |             |
//      |     MAP     |
//      |             |
//     1|_____________|2
//

    geometry_msgs::Vector3 corner_1_2_;
    geometry_msgs::Vector3 corner_1_4_;
    geometry_msgs::Vector3 corner_4_3_;

    void move_it(void);
    void start_sweeping(void);
    void stop_sweeping(void);
//    void get_robot_position(const geometry_msgs::PoseWithCovarianceStamped& msg);
};

Navigation::Navigation(ros::NodeHandle &n)
{
    sweep_state_ = n.advertise<std_msgs::String>("sweep_state", 1);

//    n.param("min", sweep_alpha, -0.9);
    n.param("max", sweep_alpha, 0.9);

    // Measurement of the distance between corners
    tf::TransformListener listener;
      /* Corner_1_2 */
      tf::Stamped<tf::Pose> corner_2(
          tf::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
          ros::Time(0), "corner_2");

      tf::Stamped<tf::Pose> transformed_corner_1_2_;
      listener.waitForTransform("corner_1", "corner_2", ros::Time(0), ros::Duration(15.0));
      listener.waitForTransform("corner_1", "corner_2", ros::Time(0), ros::Duration(15.0));
      try{
        listener.transformPose("corner_1", corner_2, transformed_corner_1_2_);
        ROS_INFO("Corner_1_2 is at x:%lf y:%lf", transformed_corner_1_2_.getOrigin().x(), transformed_corner_1_2_.getOrigin().y());
        corner_1_2_.x=transformed_corner_1_2_.getOrigin().x();
        corner_1_2_.y=transformed_corner_1_2_.getOrigin().y();

      }
      catch (tf::TransformException ex){
        ROS_ERROR("Corner_1_2 ERROR: %s",ex.what());
        ros::Duration(1.0).sleep();
      }

      /* Corner_1_4 */
      tf::Stamped<tf::Pose> corner_4(
          tf::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
          ros::Time(0), "corner_4");

      tf::Stamped<tf::Pose> transformed_corner_1_4_;
      listener.waitForTransform("corner_1", "corner_4", ros::Time(0), ros::Duration(3.0));

      try{
        listener.transformPose("corner_1", corner_4, transformed_corner_1_4_);
        ROS_INFO("Corner_1_4 is at x:%lf y:%lf", transformed_corner_1_4_.getOrigin().x(), transformed_corner_1_4_.getOrigin().y());
        corner_1_4_.x=transformed_corner_1_4_.getOrigin().x();
        corner_1_4_.y=transformed_corner_1_4_.getOrigin().y();

      }
      catch (tf::TransformException ex){
        ROS_ERROR("Corner_1_4 ERROR: %s",ex.what());
        ros::Duration(1.0).sleep();
      }

      /* Corner_2_3 */
      tf::Stamped<tf::Pose> corner_3(
          tf::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
          ros::Time(0), "corner_3");

      tf::Stamped<tf::Pose> transformed_corner_4_3_;
      listener.waitForTransform("corner_4", "corner_3", ros::Time(0), ros::Duration(3.0));

      try{
        listener.transformPose("corner_4", corner_3, transformed_corner_4_3_);
        ROS_INFO("Corner_2_3 is at x:%lf y:%lf", transformed_corner_4_3_.getOrigin().x(), transformed_corner_4_3_.getOrigin().y());
        corner_4_3_.x=transformed_corner_4_3_.getOrigin().x();
        corner_4_3_.y=transformed_corner_4_3_.getOrigin().y();

      }
      catch (tf::TransformException ex){
        ROS_ERROR("Corner_4_3 ERROR: %s",ex.what());
        ros::Duration(1.0).sleep();
      }

}

void Navigation::start_sweeping(void ){


    std_msgs::String msg;

    msg.data = "start";

    sweep_state_.publish(msg);

}

void Navigation::stop_sweeping(void ){

    std_msgs::String msg;

    msg.data = "stop";

    sweep_state_.publish(msg);



}

void Navigation::move_it(void ){

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("The robot arrived to the destination");
  else
    ROS_INFO("The robot failed to move to the destination for some reason");

}


void Navigation::spinOnce( void )
{

    bool upstroke_move_= true;      // (T) Upward movement
                                    // (F) Downward movement

    double sweep_width = 2*ARM_LENGTH*sin(sweep_alpha);

    //Prevents the swap of corners order, calculating its direction
    int y=corner_1_2_.y/fabs(corner_1_2_.y);

    stop_sweeping();

    std_msgs::String msg;
    msg.data = "running";
    status_pub.publish(msg);

    for (int num_step_move_=0; num_step_move_*sweep_width<fabs(corner_1_2_.y) && num_step_move_*sweep_width<fabs(corner_4_3_.y);num_step_move_++){

        if (upstroke_move_){
            //Prepare to the new goal
            goal.target_pose.header.frame_id = "corner_1";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.y = y*(sweep_width*num_step_move_+sweep_width/2);
            if (fabs(goal.target_pose.pose.position.y)>fabs(corner_1_2_.y))  //Prevents the robot exit of the map
                goal.target_pose.pose.position.y=corner_1_2_.y;
            goal.target_pose.pose.position.x = corner_1_2_.x/corner_1_2_.y*goal.target_pose.pose.position.y-(ARM_LENGTH+HUSKY_LENGTH/2);
            goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(atan2(corner_1_4_.y,corner_1_4_.x));

            ROS_INFO("Prepare to the new goal...");
            move_it();
            start_sweeping();

            //Give new goal
            goal.target_pose.header.frame_id = "corner_4";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.y = y*(sweep_width*num_step_move_+sweep_width/2);
            if (fabs(goal.target_pose.pose.position.y)>fabs(corner_4_3_.y))  //Prevents the robot exit of the map
                goal.target_pose.pose.position.y=corner_4_3_.y;
            goal.target_pose.pose.position.x = corner_4_3_.x/corner_4_3_.y*goal.target_pose.pose.position.y-(ARM_LENGTH+HUSKY_LENGTH/2)*cos(sweep_alpha);
            goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(atan2(corner_1_4_.y,corner_1_4_.x));

            ROS_INFO("Move to x:%lf y:%lf yaw:%lf", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, tf::getYaw(goal.target_pose.pose.orientation));
            move_it();
            stop_sweeping();

            goal.target_pose.pose.position.x = corner_4_3_.x/corner_4_3_.y*goal.target_pose.pose.position.y;
            ROS_INFO("Getting out of map");
            move_it();


            upstroke_move_=false;
        }
        else{

            //Prepare to the new goal
            goal.target_pose.header.frame_id = "corner_4";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.y = y*(sweep_width*num_step_move_+sweep_width/2);
            if (fabs(goal.target_pose.pose.position.y)>fabs(corner_4_3_.y))  //Prevents the robot exit of the map
                goal.target_pose.pose.position.y=corner_4_3_.y;
            goal.target_pose.pose.position.x = corner_4_3_.x/corner_4_3_.y*goal.target_pose.pose.position.y+ARM_LENGTH+HUSKY_LENGTH/2;
            goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(atan2(corner_1_4_.y,corner_1_4_.x)+3.14);

            ROS_INFO("Prepare to the new goal...");
            move_it();
            start_sweeping();


            //Give new goal
            goal.target_pose.header.frame_id = "corner_1";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.y = y*(sweep_width*num_step_move_+sweep_width/2);
            if (fabs(goal.target_pose.pose.position.y)>fabs(corner_1_2_.y))  //Prevents the robot exit of the map
                goal.target_pose.pose.position.y=corner_1_2_.y;
            goal.target_pose.pose.position.x = corner_1_2_.x/corner_1_2_.y*goal.target_pose.pose.position.y+(ARM_LENGTH+HUSKY_LENGTH/2)*cos(sweep_alpha);
            goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(atan2(corner_1_4_.y,corner_1_4_.x)+3.14);

            ROS_INFO("Move to x:%lf y:%lf yaw:%lf", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, tf::getYaw(goal.target_pose.pose.orientation));
            move_it();
            stop_sweeping();

            goal.target_pose.pose.position.x = corner_1_2_.x/corner_1_2_.y*goal.target_pose.pose.position.y;
            ROS_INFO("Getting out of map");
            move_it();


            upstroke_move_=true;
        }


    }

    stop_sweeping();

    ROS_INFO("End of the map");

    msg.data = "done";
    status_pub.publish(msg);

}

int main(int argc, char** argv){
  ros::init(argc, argv, "navigation_goals");
  ros::NodeHandle n;

  Navigation navigation(n);

  ros::AsyncSpinner spinner(3);
  spinner.start();

  status_pub = n.advertise<std_msgs::String>("navigation_status", 1);

  ros::Rate r(2.0);
  while(ros::ok())
  {
      navigation.spinOnce();
      r.sleep();

      break;

  }


  spinner.stop();

  return 0;
}
