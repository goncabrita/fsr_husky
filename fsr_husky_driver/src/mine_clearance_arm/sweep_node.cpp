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
* Updated: Baptiste Gil on 11/09/2014
*********************************************************************/


#include <ros/ros.h>
#include <tf/tf.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <boost/foreach.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>


#define ARM_LENGTH 1.06
#define COIL_LENGTH 0.62
#define COIL_WIDTH 0.16

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Ac;

class Sweeper
{
public:
    Sweeper(ros::NodeHandle &n, ros::NodeHandle &pn);

    void spinOnce(Ac& ac);

private:
    void calc_new_lift(double last_position);
    void pointCloudCallback(const PointCloud::ConstPtr& msg);
    void msgCallback(const std_msgs::String& msg);
    void odo_velCallback(const nav_msgs::Odometry& msg);
    void jointsCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg);
    void publishRvizMarker(void);
    ros::Subscriber cloud_sub_;
    tf::TransformListener tf_;

    ros::Publisher sweep_markers_pub_;
    ros::Publisher roi_cloud_pub_;
    ros::Publisher roi_temp_cloud_pub_;
    ros::Subscriber joints_sub_;

    ros::Subscriber msg_sub_;
    ros::Subscriber odo_vel_sub_;

    pcl::PointCloud<PointT>::Ptr cloud_filtered_;
    pcl::PointCloud<PointT>::Ptr temp_cloud_filtered_;

//    std::string base_link_frame_id_;
//    std::string metal_detector_frame_id_;
    std::string base_footprint_frame_id_;
    std::string base_front_bumper_frame_id_;

    bool got_cloud_;

    std::string lift_joint_;
    std::string sweep_joint_;

    visualization_msgs::Marker points;

    //Lift variables
    double min_lift_;
    double max_lift_;
    double max_lift_speed_;
    double hoffset;
    double increment;
    double lift_;
    double current_lift_;
    double last_lift_;

    //Sweep variables
    double sweep_speed_;
    double min_sweep_speed_;
    double min_alpha_sweep;
    double max_alpha_sweep;
    double acceleration;
    double current_sweep_;
    double sweep_;
    double last_sweep_;
    double sweep_tolerance_;
    double reference_height_lift;

    double linear_speed;
    double position;

    bool flag_sweep;
    bool sweep_speed_control;

    double max_vel_metal_det;

};

Sweeper::Sweeper(ros::NodeHandle &n, ros::NodeHandle &pn)
{
//    pn.param<std::string>("base_link_frame_id", base_link_frame_id_, "base_link");
    pn.param<std::string>("base_footprint", base_footprint_frame_id_, "base_footprint");
    pn.param<std::string>("base_footprint_front_bumper_part", base_front_bumper_frame_id_, "base_footprint_front_bumper_part");
//    pn.param<std::string>("metal_detector_frame_id", metal_detector_frame_id_, "metal_detector_antenna_link");

    msg_sub_ = n.subscribe("sweep_state", 1, &Sweeper::msgCallback,this);

    cloud_sub_ = n.subscribe<PointCloud>("/cloud", 1, &Sweeper::pointCloudCallback, this);

    sweep_markers_pub_ = n.advertise<visualization_msgs::Marker>( "sweep_markers", 100 );
    roi_cloud_pub_ = n.advertise<PointCloud>("roi_cloud", 1);
    roi_temp_cloud_pub_ = n.advertise<PointCloud>("roi_temp_cloud", 1);

    joints_sub_ = n.subscribe("/arm_controller/state", 10, &Sweeper::jointsCallback, this);

    //Sweep parameters
    pn.param("min", min_alpha_sweep, -0.9);
    pn.param("max", max_alpha_sweep, 0.9);
    pn.param("min_sweep_speed", min_sweep_speed_, 0.2);    // speed of sweep (rad/s)
    pn.param("max_vel_metal_det", max_vel_metal_det, 1.0); // maximum velocity of metal detector (m/s)
    //    pn.param("speed", sweep_speed_, 0.2);
    sweep_speed_ = min_sweep_speed_;
    pn.param("acceleration", acceleration, 2.8);
    pn.param("sweep_tolerance", sweep_tolerance_, 0.05);
    n.param("set_sweep_speed_control", sweep_speed_control, true);

    if (sweep_speed_control);
    odo_vel_sub_ = n.subscribe("encoder_corrected", 1, &Sweeper::odo_velCallback,this);

    //Lift parameters
    pn.param<std::string>("lift_joint", lift_joint_, "upper_arm_joint");
    pn.param<std::string>("sweep_joint", sweep_joint_, "arm_axel_joint");
    pn.param("increment", increment, 0.005);
    pn.param("min_lift", min_lift_, -0.20);
    pn.param("max_lift", max_lift_, 0.25);
    pn.param("max_lift_speed", max_lift_speed_, 0.8);
    pn.param("reference_height_lift", reference_height_lift, 0.235);

    cloud_filtered_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    temp_cloud_filtered_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);

    sweep_ = min_alpha_sweep;
    flag_sweep=true;
}

void Sweeper::pointCloudCallback(const PointCloud::ConstPtr& msg)
{
//    ROS_INFO("Sweeper - %s - Got a cloud msg.", __FUNCTION__);

/* FILTERING THE AREA WHERE THE METAL DETECTOR SWEEPS
*
*                      ANALYZED AREA
*      *******************************************
*      *                  _____                  *
*      *  ____           |     |          ____   *
*      * \     \         |  A  |         /    /  *
*      *  \     \  <---  |  R  |  --->  /    /   *
*      *   \     \       |  M  |       /    /    *
*      *    \     \      |_____|      /    /     *
*      *     \_____\       ||        /____/      *
*      *        \\         ||         //         *
*      **********\\********||********//***********
*       ^ X       \\       ||       //
*       |
*       |
*  <----x
*  Y      Z
*/


    double extra_size=0.3; // Security marge
    double max_height=0.5;

    geometry_msgs::PointStamped corners[8];

    corners[0].point.x = 0.0;
    corners[0].point.y = ARM_LENGTH*sin(min_alpha_sweep)-extra_size;
    corners[0].point.z = -max_height;

    corners[1].point.x = 0.0;
    corners[1].point.y = ARM_LENGTH*sin(min_alpha_sweep)-extra_size;
    corners[1].point.z = max_height;

    corners[2].point.x = 0.0;
    corners[2].point.y = ARM_LENGTH*sin(max_alpha_sweep)+extra_size;
    corners[2].point.z = -max_height;

    corners[3].point.x = 0.0;
    corners[3].point.y = ARM_LENGTH*sin(max_alpha_sweep)+extra_size;
    corners[3].point.z = max_height;

    corners[4].point.x = ARM_LENGTH + COIL_LENGTH/2 + linear_speed / (sweep_speed_/(max_alpha_sweep-min_alpha_sweep)) + extra_size;
    corners[4].point.y = ARM_LENGTH*sin(min_alpha_sweep)-extra_size;
    corners[4].point.z = -max_height;

    corners[5].point.x = ARM_LENGTH + COIL_LENGTH/2 + linear_speed / (sweep_speed_/(max_alpha_sweep-min_alpha_sweep)) + extra_size;
    corners[5].point.y = ARM_LENGTH*sin(min_alpha_sweep)-extra_size;
    corners[5].point.z = max_height;

    corners[6].point.x = ARM_LENGTH + COIL_LENGTH/2 + linear_speed / (sweep_speed_/(max_alpha_sweep-min_alpha_sweep)) + extra_size;
    corners[6].point.y = ARM_LENGTH*sin(max_alpha_sweep)+extra_size;
    corners[6].point.z = -max_height;

    corners[7].point.x = ARM_LENGTH + COIL_LENGTH/2 + linear_speed / (sweep_speed_/(max_alpha_sweep-min_alpha_sweep)) + extra_size;
    corners[7].point.y = ARM_LENGTH*sin(max_alpha_sweep)+extra_size;
    corners[7].point.z = max_height;

    pcl_conversions::fromPCL(msg->header, corners[0].header);
    corners[0].header.frame_id = base_front_bumper_frame_id_;
    tf_.waitForTransform(msg->header.frame_id, corners[0].header.frame_id, corners[0].header.stamp, ros::Duration(0.2));
    for(int i=0 ; i<8 ; i++)
    {
        corners[i].header = corners[0].header;

        try
        {
            tf_.transformPoint(msg->header.frame_id, corners[i], corners[i]);
        }
        catch(tf::TransformException &ex)
        {
            ROS_ERROR("Sweeper - %s - Error: %s", __FUNCTION__, ex.what());
            got_cloud_=false;
            return;
        }
    }

    PointT max_p, min_p;
    max_p.x = min_p.x = corners[0].point.x;
    max_p.y = min_p.y = corners[0].point.y;
    max_p.z = min_p.z = corners[0].point.z;
    for(int i=1 ; i<8 ; i++)
    {
        if(corners[i].point.x > max_p.x) max_p.x = corners[i].point.x;
        if(corners[i].point.x < min_p.x) min_p.x = corners[i].point.x;

        if(corners[i].point.y > max_p.y) max_p.y = corners[i].point.y;
        if(corners[i].point.y < min_p.y) min_p.y = corners[i].point.y;

        if(corners[i].point.z > max_p.z) max_p.z = corners[i].point.z;
        if(corners[i].point.z < min_p.z) min_p.z = corners[i].point.z;
    }

    pcl::PassThrough<PointT> filter;
    filter.setInputCloud(msg);
//    filter.setFilterFieldName("x");
//    filter.setFilterLimits(min_p.x, max_p.x);
//    filter.filter(*cloud_filtered_);
//    filter.setInputCloud(cloud_filtered_);
//    filter.setFilterFieldName("y");
//    filter.setFilterLimits(min_p.y, max_p.y);
//    filter.filter(*cloud_filtered_);
    filter.setFilterFieldName("z");
    filter.setFilterLimits(min_p.z, max_p.z);
    filter.filter(*cloud_filtered_);

    roi_cloud_pub_.publish(cloud_filtered_);

    got_cloud_ = true;
//    ROS_INFO("Sweeper - %s - Filtered the cloud!!!", __FUNCTION__);
}


void Sweeper::calc_new_lift(double last_position){

    if(!got_cloud_){
        lift_=0;
        ROS_INFO("Point Cloud empty");
        return;
    }
/* FILTERING THE AREA AROUND THE CURRENT AND THE NEXT POSITION OF THE ARM
*
*             ANALYZED AREA
*   c2 **************************  c0
*      *                  _____ *
*      *  ____           |     |*
*      * \     \         |  A  |*
*      *  \     \  <---  |  R  |*
*      *   \     \       |  M  |*
*      *    \     \      |_____|*
*      *     \_____\       ||   *
*      *        \\         ||   *
*   c3 **********\\********||****  c1
*       ^ X       \\       ||
*       |
*       |
*  <----x
*  Y      Z
*/

    ROS_INFO("pos: %lf last_p: %lf",position,last_position);

    double extra_size=0.20; // Security marge

    geometry_msgs::PointStamped corners[4];

    int sweep_direction=1;
    if(position<last_position)
        sweep_direction=-1;



    corners[0].point.x = (ARM_LENGTH+COIL_LENGTH/2+extra_size)*cos(last_position);
    corners[1].point.y = -((ARM_LENGTH+COIL_LENGTH/2+extra_size)*sin(last_position)-sweep_direction*(COIL_WIDTH/2+extra_size));
    corners[0].point.z = 0.0;

    corners[1].point.x = (ARM_LENGTH-COIL_LENGTH/2-extra_size)*cos(last_position);
    corners[1].point.y = -((ARM_LENGTH-COIL_LENGTH/2-extra_size)*sin(last_position)-sweep_direction*(COIL_WIDTH/2+extra_size));
    corners[1].point.z = 0.0;

    corners[2].point.x = (ARM_LENGTH+COIL_LENGTH/2+extra_size)*cos(position-sweep_direction*asin((COIL_WIDTH/2)/(ARM_LENGTH-COIL_LENGTH/2-extra_size)));
    corners[2].point.y = -(ARM_LENGTH+COIL_LENGTH/2+extra_size)*sin(position-sweep_direction*asin((COIL_WIDTH/2)/(ARM_LENGTH-COIL_LENGTH/2-extra_size)));
    corners[2].point.z = 0.0;

    corners[3].point.x = (ARM_LENGTH-COIL_LENGTH/2-extra_size)*cos(position-sweep_direction*asin((COIL_WIDTH/2)/(ARM_LENGTH-COIL_LENGTH/2-extra_size)));
    corners[3].point.y = -(ARM_LENGTH-COIL_LENGTH/2-extra_size)*sin(position-sweep_direction*asin((COIL_WIDTH/2)/(ARM_LENGTH-COIL_LENGTH/2-extra_size)));
    corners[3].point.z = 0.0;




    pcl_conversions::fromPCL(cloud_filtered_->header, corners[0].header);
    corners[0].header.frame_id = base_footprint_frame_id_;
    tf_.waitForTransform(cloud_filtered_->header.frame_id, corners[0].header.frame_id, corners[0].header.stamp, ros::Duration(0.2));

    for(int i=0 ; i<4 ; i++)
    {
        corners[i].header = corners[0].header;

        try
        {
            tf_.transformPoint(cloud_filtered_->header.frame_id, corners[i], corners[i]);
        }
        catch(tf::TransformException &ex)
        {
            ROS_ERROR("Sweeper - %s - Error: %s", __FUNCTION__, ex.what());
            return;
        }
    }

    PointT max_p, min_p;
    max_p.x = min_p.x = corners[0].point.x;
    max_p.y = min_p.y = corners[0].point.y;
    for(int i=1 ; i<4 ; i++)
    {
        if(corners[i].point.x > max_p.x) max_p.x = corners[i].point.x;
        if(corners[i].point.x < min_p.x) min_p.x = corners[i].point.x;

        if(corners[i].point.y > max_p.y) max_p.y = corners[i].point.y;
        if(corners[i].point.y < min_p.y) min_p.y = corners[i].point.y;
    }

    pcl::PassThrough<PointT> filter;
    filter.setInputCloud(cloud_filtered_);
    filter.setFilterFieldName("x");
    filter.setFilterLimits(min_p.x, max_p.x);
    filter.filter(*temp_cloud_filtered_);
    filter.setInputCloud(temp_cloud_filtered_);
    filter.setFilterFieldName("y");
    filter.setFilterLimits(min_p.y, max_p.y);
    filter.filter(*temp_cloud_filtered_);

    roi_temp_cloud_pub_.publish(temp_cloud_filtered_);

    lift_=temp_cloud_filtered_->points[0].z;
    for(int i=0; i<temp_cloud_filtered_->size();i++){
        if (lift_<temp_cloud_filtered_->points[i].z)
                lift_=temp_cloud_filtered_->points[i].z;
    }

    lift_-=reference_height_lift;

    ros::Duration(1.0).sleep();

//    ROS_INFO("Max_height: %lf frameid: %s", lift_, temp_cloud_filtered_->header.frame_id.c_str());

//    ROS_INFO("Sweeper - %s - Filtered the cloud!!!", __FUNCTION__);

}

void Sweeper::publishRvizMarker(void){

    geometry_msgs::Point p;
    p.x=ARM_LENGTH*cos(position);
    p.y=ARM_LENGTH*sin(position);
    p.z=lift_;
    std_msgs::ColorRGBA corPonto;
    corPonto.r = 255;
    corPonto.g = 0;
    corPonto.b = 0;
    corPonto.a = 1.0;

   //---------RVIZ Marker publication---------------
    //turn Global --- visualization_msgs::Marker points;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    points.header.frame_id = base_front_bumper_frame_id_;

    points.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    points.ns = "Sweep_points";
    points.action = visualization_msgs::Marker::ADD;

// %Tag(NS_ID)%
    points.id = 0;
// %EndTag(NS_ID)%

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
// %Tag(TYPE)%
    points.type = visualization_msgs::Marker::LINE_STRIP;
// %EndTag(TYPE)%

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
// %Tag(SCALE)%
    points.scale.x = 0.05;
    points.scale.y = 0.05;
    points.scale.z = 0.05;
// %EndTag(SCALE)%

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
// %Tag(POSE)%
    points.pose.position.x = p.x;
    points.pose.position.y = p.y;
    points.pose.position.z = p.z;
    points.pose.orientation.x = 0.0;
    points.pose.orientation.y = 0.0;
    points.pose.orientation.z = 0.0;
    points.pose.orientation.w = 1.0;
// %EndTag(POSE)%


    // Set the color -- be sure to set alpha to something non-zero!
// %Tag(COLOR)%
    // Points are initially red
    points.color.r = 1.0;
    points.color.a = 1.0;
// %EndTag(COLOR)%

// %Tag(LIFETIME)%
    points.lifetime = ros::Duration();
// %EndTag(LIFETIME)%

    points.points.push_back(p);
    points.colors.push_back(corPonto);


    // Publish the marker
    sweep_markers_pub_.publish(points);

}


void Sweeper::odo_velCallback(const nav_msgs::Odometry& msg)
{
    double alpha_sweep = max_alpha_sweep - min_alpha_sweep; //total sweep angle
    linear_speed = msg.twist.twist.linear.x;

    double max_dist_robot =  COIL_WIDTH * sin(alpha_sweep/2) + COIL_LENGTH * cos (alpha_sweep/2); //max distance robot can move

    sweep_speed_ = (linear_speed * alpha_sweep/ max_dist_robot);

//    ROS_INFO("lin_vel - %f - max_dist - %f", linear_speed, max_dist_robot);


    if( sweep_speed_ < min_sweep_speed_)
        sweep_speed_ = min_sweep_speed_;
    else
        if (sqrt(pow(linear_speed,2)+pow(sweep_speed_*ARM_LENGTH,2))> max_vel_metal_det){
            double max_robo_vel=max_vel_metal_det/(sqrt(1+pow(ARM_LENGTH * alpha_sweep/ max_dist_robot,2)));
            sweep_speed_ = sqrt(pow(max_vel_metal_det,2)-pow(max_robo_vel,2))/ARM_LENGTH;
            ROS_INFO("Sweep speed out of range!");
        }

//    ROS_INFO("Sweep speed (rad/s) - %f - Sweep (m/s) - %f", sweep_speed_, sqrt(pow(linear_speed,2)+pow(sweep_speed_*ARM_LENGTH,2)));

}

void Sweeper::msgCallback(const std_msgs::String& msg)
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


void Sweeper::jointsCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg)
{
    //ROS_INFO("Sweeper - %s - Got joints info! %s %lf %s %lf", __FUNCTION__, msg->joint_names[0].c_str(), msg->actual.positions[0], msg->joint_names[1].c_str(), msg->actual.positions[1]);

    for(int i=0 ; i<msg->joint_names.size() ; i++)
    {
        if(lift_joint_.compare(msg->joint_names[i]) == 0) current_lift_ = msg->actual.positions[i];
        else if(sweep_joint_.compare(msg->joint_names[i]) == 0) current_sweep_ = msg->actual.positions[i];
    }
}

void Sweeper::spinOnce(Ac &ac)
{
    if(!got_cloud_) return;
    if(flag_sweep){

    // Step 1. Check if we should invert the sweep direction
    if(fabs(sweep_ - current_sweep_) <= sweep_tolerance_)
    {
        if(sweep_  == min_alpha_sweep) sweep_ = max_alpha_sweep;
        else sweep_ = min_alpha_sweep;
    }

    // Step 2. Calculate the new points
        int number_of_steps=9;

        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory.header.stamp = ros::Time::now();
        goal.trajectory.points.resize(number_of_steps);
        goal.path_tolerance.resize(number_of_steps+1);
        goal.goal_tolerance.resize(number_of_steps+1);
        goal.trajectory.joint_names.push_back("upper_arm_joint");
        goal.trajectory.joint_names.push_back("arm_axel_joint");

        //Reset variables
        position=current_sweep_;
        points.points.clear();
        points.colors.clear();

        //Rviz Markers variables
        publishRvizMarker();

        for(int i=0;i<number_of_steps;i++){

            double last_position=position;
            position+=2*sweep_/number_of_steps;
//            ROS_INFO("id: %d position: %lf",i,position);

            if(position>max_alpha_sweep){
                position=max_alpha_sweep;
            }
            else
                if(position<min_alpha_sweep){
                    position=min_alpha_sweep;
                }

            // Step 3. Determine the new lift angle based on the distance to the ground
            calc_new_lift(last_position);

            double alpha_lift_=asin(lift_/ARM_LENGTH);
            alpha_lift_+=hoffset;
//            ROS_INFO("lift: %lf alpha: %lf",lift_,alpha_lift_);
            // If we are bumping into something we cannot go over, turn back!
            if(alpha_lift_>max_lift_){
                alpha_lift_=max_lift_;
//                sweep_ = sweep_ == min_alpha_sweep ? max_alpha_sweep : min_alpha_sweep;
                ROS_INFO("LIFT: too hight! - Inverted direction of sweep");
            }
            else if (alpha_lift_<min_lift_){
                alpha_lift_=min_lift_;
                ROS_INFO("LIFT: too low!");
            }
            /*if(lift_ > max_lift_ || lift_ < min_lift_  && ros::Time::now() - last_change_of_direction_ > time_between_change_of_directions_)
            {
                lift_ = current_lift_;
                sweep_ = sweep_ == min_sweep_ ? max_sweep_ : min_sweep_;
                last_change_of_direction_ = ros::Time::now();
            }*/

        //        ROS_INFO("Sweeper - %s - Min distance to the ground is %lf, delta is %lf, current h is %lf and the desired h is %lf", __FUNCTION__, min_distance, delta_h, current_h, desired_h);

            goal.trajectory.points[i].positions.push_back(alpha_lift_);
            goal.trajectory.points[i].velocities.push_back(0.0);
            goal.trajectory.points[i].accelerations.push_back(0.05);
            goal.trajectory.points[i].positions.push_back(position);
            goal.trajectory.points[i].velocities.push_back(sweep_speed_/4);

            //Rviz Markers variables
            publishRvizMarker();

    //        if(position - min_alpha_sweep <= sweep_tolerance_ || position - max_alpha_sweep <= sweep_tolerance_)
                goal.trajectory.points[i].accelerations.push_back(acceleration);
      //      else
        //        goal.trajectory.points[i].accelerations.push_back(999999*acceleration);

            goal.path_tolerance[i].name = "upper_arm_joint";
            goal.path_tolerance[i].position = -1;

            goal.goal_tolerance[i].name = "upper_arm_joint";
            goal.goal_tolerance[i].position = 0.05;
        }

        goal.goal_tolerance[number_of_steps].name = "arm_axel_joint";
        goal.goal_tolerance[number_of_steps].position = 0.05;
        goal.path_tolerance[number_of_steps].name = "arm_axel_joint";
        goal.path_tolerance[number_of_steps].position = -1;

        goal.goal_time_tolerance = ros::Duration(30.0);

        // Step 4. Send the new command to the arm
    //    if(lift_ != last_lift_ || sweep_ != last_sweep_)
        ROS_INFO("NEW GOAL");
        ac.sendGoal(goal);

        // Wait for the action to return
        bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

        if(!finished_before_timeout)
        {
            actionlib::SimpleClientGoalState state = ac.getState();
            ROS_ERROR("Sweep -- %s!", state.toString().c_str());
        }


//    ROS_INFO("Sweeper - %s - Lift c:%lf d:%lf Sweep c:%lf d:%lf", __FUNCTION__, current_lift_, lift_, current_sweep_, sweep_);

    }

    last_lift_ = lift_;
    last_sweep_ = sweep_;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sweep_node");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    Ac ac("/arm_controller/follow_joint_trajectory", true);
    ROS_INFO("Sweep -- Waiting for the action server to start...");
    ac.waitForServer();
    ROS_INFO("Sweep -- Got it!");

    Sweeper sweeper(n, pn);

    ros::AsyncSpinner spinner(3);
    spinner.start();

    ros::Rate r(2.0);
    while(ros::ok())
    {
        sweeper.spinOnce(ac);
        r.sleep();
    }

    spinner.stop();

    return 0;
}
