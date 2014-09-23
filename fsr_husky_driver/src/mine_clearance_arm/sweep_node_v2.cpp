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
#include "std_msgs/Float64.h"

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

//Voxel_Grid
#include <pcl/filters/voxel_grid.h>
//Statistical_removal
#include <pcl/filters/statistical_outlier_removal.h>
//Convexhull
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>



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
    void calc_new_lift(void);
    void pointCloudCallback(const PointCloud::ConstPtr& msg);
    void msgCallback(const std_msgs::String& msg);
    void odo_velCallback(const nav_msgs::Odometry& msg);
    void jointsCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg);
    void pointCloudVoxelGrid(void);
    void pointCloudStatisticalRemoving(void);
    void pointCloudConvexHull(void);

    ros::Subscriber cloud_sub_;
    tf::TransformListener tf_;

    ros::Publisher roi_cloud_pub_;
    ros::Publisher roi_temp_cloud_pub_;
    ros::Publisher lift_pub_;

    //Filtred Clouds publishers
    ros::Publisher cloud_voxel_filtered_pub_;
    ros::Publisher cloud_statistical_filtered_pub_;
    ros::Publisher cloud_convexhull_filtered_pub_;

    ros::Subscriber joints_sub_;
    ros::Subscriber msg_sub_;
    ros::Subscriber odo_vel_sub_;

    pcl::PointCloud<PointT>::Ptr cloud_filtered_;
    pcl::PointCloud<PointT>::Ptr last_cloud_filtered_;
    pcl::PointCloud<PointT>::Ptr temp_cloud_filtered_;

    //Filtred Clouds
    pcl::PointCloud<PointT>::Ptr cloud_voxel_filtered_;
    pcl::PointCloud<PointT>::Ptr cloud_statistical_filtered_;
    pcl::PointCloud<PointT>::Ptr cloud_convexhull_filtered_;


//    std::string base_link_frame_id_;
//    std::string metal_detector_frame_id_;
    std::string base_footprint_frame_id_;
    std::string base_front_bumper_frame_id_;
    std::string right_coil_frameid;

    bool got_cloud_;

    std::string lift_joint_;
    std::string sweep_joint_;

    ros::Time last_lift_pub;

    //Lift variables
    double min_lift_;
    double max_lift_;
    double max_lift_speed_;
    double hoffset;
    double increment;
    double lift_;
    double current_lift_;

    //Sweep variables
    double sweep_speed_;
    double min_sweep_speed_;
    double min_alpha_sweep;
    double max_alpha_sweep;
    double acceleration;
    double current_sweep_;
    double sweep_;
    double sweep_tolerance_;
    double lift_tolerance_;
    double reference_height_lift;

    double linear_speed;
    double alpha_lift_;

    bool flag_sweep;
    bool sweep_speed_control;

    double max_vel_metal_det;

};

Sweeper::Sweeper(ros::NodeHandle &n, ros::NodeHandle &pn)
{
//    pn.param<std::string>("base_link_frame_id", base_link_frame_id_, "base_link");
    pn.param<std::string>("base_footprint", base_footprint_frame_id_, "base_footprint");
    pn.param<std::string>("right_coil_frameid", right_coil_frameid, "right_coil");
    pn.param<std::string>("base_footprint_front_bumper_part", base_front_bumper_frame_id_, "base_footprint_front_bumper_part");
//    pn.param<std::string>("metal_detector_frame_id", metal_detector_frame_id_, "metal_detector_antenna_link");

    msg_sub_ = n.subscribe("sweep_state", 1, &Sweeper::msgCallback,this);

    cloud_sub_ = n.subscribe<PointCloud>("/cloud", 1, &Sweeper::pointCloudCallback, this);

    roi_cloud_pub_ = n.advertise<PointCloud>("roi_cloud", 1);
    roi_temp_cloud_pub_ = n.advertise<PointCloud>("roi_temp_cloud", 1);
    lift_pub_ = n.advertise<std_msgs::Float64>("lift_control", 1);

    cloud_voxel_filtered_pub_ = n.advertise<PointCloud>("cloud_voxel", 1);
    cloud_statistical_filtered_pub_ = n.advertise<PointCloud>("cloud_statistical", 1);
    cloud_convexhull_filtered_pub_ = n.advertise<PointCloud>("cloud_convexhull", 1);

    joints_sub_ = n.subscribe("/arm_controller/state", 10, &Sweeper::jointsCallback, this);

    //Sweep parameters
    pn.param("min", min_alpha_sweep, -0.9);
    pn.param("max", max_alpha_sweep, 0.9);
    pn.param("min_sweep_speed", min_sweep_speed_, 0.05);    // speed of sweep (rad/s)
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
    pn.param("reference_height_lift", reference_height_lift, 0.235); //Measure between the sensor and the ground when alpha_lift=0
    pn.param("lift_tolerance", lift_tolerance_, 0.005); // In meters

    cloud_filtered_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    last_cloud_filtered_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    temp_cloud_filtered_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);

    //Clouds filtred
    cloud_voxel_filtered_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    cloud_statistical_filtered_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
    cloud_convexhull_filtered_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);

    sweep_ = min_alpha_sweep;
    alpha_lift_=min_lift_;
    flag_sweep=true;
}
/************  FILTERS TEST ******************/

//******* VOXEL GRID FILTER
void Sweeper::pointCloudVoxelGrid(void)
{
    // Create the Voxel grid filtering
    pcl::VoxelGrid<pcl::PointXYZ> sor2;
    sor2.setInputCloud (cloud_statistical_filtered_);
    sor2.setLeafSize (0.10f, 0.10f, 0.30f);
//    sor2.filter (*cloud_voxel_filtered_);
    sor2.filter (*cloud_filtered_);

//    cloud_voxel_filtered_pub_.publish(cloud_voxel_filtered_);
    cloud_voxel_filtered_pub_.publish(cloud_filtered_);

}

//******* STATISTICAL REMOVING FILTER
void Sweeper::pointCloudStatisticalRemoving(void)
{
    // Create the statistical filtering
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud_filtered_);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_statistical_filtered_);

    cloud_statistical_filtered_pub_.publish(cloud_statistical_filtered_);

}

//******* CONVEX HULL FILTER
void Sweeper::pointCloudConvexHull(void)
{
    // Create the ConvexHull filtering

    // Build a filter to remove spurious NaNs
      pcl::PassThrough<pcl::PointXYZ> pass;
      pass.setInputCloud (cloud_statistical_filtered_);
      pass.setFilterFieldName ("z");
      pass.setFilterLimits (-10.0, 10.0);
      pass.filter (*cloud_convexhull_filtered_);
//      std::cerr << "PointCloud after filtering has: "
//                << cloud_convexhull_filtered_->points.size () << " data points." << std::endl;

      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      // Create the segmentation object
      pcl::SACSegmentation<pcl::PointXYZ> seg;
      // Optional
      seg.setOptimizeCoefficients (true);
      // Mandatory
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setDistanceThreshold (0.01);

      seg.setInputCloud (cloud_convexhull_filtered_);
      seg.segment (*inliers, *coefficients);
//      std::cerr << "PointCloud after segmentation has: "
//                << inliers->indices.size () << " inliers." << std::endl;

//      // Project the model inliers
//      pcl::ProjectInliers<pcl::PointXYZ> proj;
//      proj.setModelType (pcl::SACMODEL_PLANE);
//      proj.setIndices (inliers);
//      proj.setInputCloud (cloud_convexhull_filtered_);
//      proj.setModelCoefficients (coefficients);
//      proj.filter (*cloud_convexhull_filtered_);
////      std::cerr << "PointCloud after projection has: "
////                << cloud_convexhull_filtered_->points.size () << " data points." << std::endl;

      // Create a Concave Hull representation of the projected inliers
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::ConcaveHull<pcl::PointXYZ> chull;
      chull.setInputCloud (cloud_convexhull_filtered_);
      chull.setAlpha (0.1);
      chull.reconstruct (*cloud_convexhull_filtered_);

      cloud_convexhull_filtered_pub_.publish(cloud_convexhull_filtered_);

}

/************  END FILTERS TEST ******************/


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
    int num_corners=8;

    geometry_msgs::PointStamped corners[num_corners];

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


    for(int i=1 ; i<num_corners ; i++)
        corners[i].header = corners[0].header;

    for(int i=0 ; i<num_corners ; i++)
    {
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

    pcl::PointCloud<PointT> new_cloud_filtered_;
    pcl::PassThrough<PointT> filter;
    filter.setInputCloud(msg);
//    filter.setFilterFieldName("x");
//    filter.setFilterLimits(min_p.x, max_p.x);
//    filter.filter(*new_cloud_filtered_);
//    filter.setInputCloud(new_cloud_filtered_);
//    filter.setFilterFieldName("y");
//    filter.setFilterLimits(min_p.y, max_p.y);
//    filter.filter(*new_cloud_filtered_);
    filter.setFilterFieldName("z");
    filter.setFilterLimits(min_p.z, max_p.z);
    filter.filter(new_cloud_filtered_);

    if(got_cloud_){
        *cloud_filtered_=*last_cloud_filtered_;
        cloud_filtered_->operator +=(new_cloud_filtered_);
    }
    else
    *cloud_filtered_=new_cloud_filtered_;

    *last_cloud_filtered_=new_cloud_filtered_;

    pointCloudStatisticalRemoving();
    pointCloudVoxelGrid();
    pointCloudConvexHull();

    roi_cloud_pub_.publish(cloud_filtered_);

    got_cloud_ = true;

//    ROS_INFO("Sweeper - %s - Filtered the cloud!!!", __FUNCTION__);
}


void Sweeper::calc_new_lift(void){

    if(!got_cloud_){
        lift_=max_lift_;
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

    double extra_size=0.05; // Security marge

    int num_corners=4;
    geometry_msgs::PointStamped corners[num_corners];
    geometry_msgs::PointStamped corners2[num_corners];

    int sweep_direction=1;
    if(sweep_<current_sweep_)
        sweep_direction=-1;

    double next_sweep=current_sweep_+sweep_direction*sweep_speed_;

    corners[0].point.x = COIL_LENGTH+extra_size;
    corners[0].point.y = 0.0;
    corners[0].point.z = -sweep_direction*(COIL_WIDTH+extra_size);

    corners[1].point.x = -0.2-extra_size;
    corners[1].point.y = 0.0;
    corners[1].point.z = -sweep_direction*(COIL_WIDTH+extra_size);

    corners[2].point.x = corners[0].point.x + (ARM_LENGTH+COIL_LENGTH/2.0)*(cos(next_sweep)-cos(current_sweep_));
    corners[2].point.y = 0.0;
    corners[2].point.z = sweep_direction*extra_size + (ARM_LENGTH+COIL_LENGTH/2.0)*(sin(next_sweep)-sin(current_sweep_));

    corners[3].point.x = corners[1].point.x + (ARM_LENGTH-COIL_LENGTH/2.0)*(cos(next_sweep)-cos(current_sweep_));
    corners[3].point.y = 0.0;
    corners[3].point.z = sweep_direction*extra_size + (ARM_LENGTH-COIL_LENGTH/2.0)*(sin(next_sweep)-sin(current_sweep_));

    pcl_conversions::fromPCL(cloud_filtered_->header, corners[0].header);
    corners[0].header.frame_id = right_coil_frameid;
    tf_.waitForTransform(cloud_filtered_->header.frame_id, corners[0].header.frame_id, corners[0].header.stamp, ros::Duration(0.2));

    for(int i=1 ; i<num_corners ; i++)
        corners[i].header = corners[0].header;

    for(int i=0 ; i<num_corners ; i++)
    {
        try
        {
            tf_.transformPoint(cloud_filtered_->header.frame_id, corners[i], corners2[i]);
        }
        catch(tf::TransformException &ex)
        {
            ROS_ERROR("Sweeper - %s - Error: %s", __FUNCTION__, ex.what());
            return;
        }
    }

    PointT max_p, min_p;
    max_p.x = min_p.x = corners2[0].point.x;
    max_p.y = min_p.y = corners2[0].point.y;
    for(int i=1 ; i<4 ; i++)
    {
//        ROS_INFO("x: %lf y:%lf z: %lf",corners2[i].point.x,corners2[i].point.y,corners2[i].point.z);
        if(corners2[i].point.x > max_p.x) max_p.x = corners2[i].point.x;
        if(corners2[i].point.x < min_p.x) min_p.x = corners2[i].point.x;

        if(corners2[i].point.y > max_p.y) max_p.y = corners2[i].point.y;
        if(corners2[i].point.y < min_p.y) min_p.y = corners2[i].point.y;
    }

//    ROS_INFO("max_x: %lf max_y:%lf",max_p.x,max_p.y);
//    ROS_INFO("min_x: %lf min_y:%lf",min_p.x,min_p.y);

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

    alpha_lift_=asin(lift_/ARM_LENGTH);
    alpha_lift_+=hoffset;
//            ROS_INFO("lift: %lf alpha: %lf",lift_,alpha_lift_);
    // If we are bumping into something we cannot go over, turn back!
    if(alpha_lift_>max_lift_){
        alpha_lift_=max_lift_;
//                sweep_ = sweep_ == min_alpha_sweep ? max_alpha_sweep : min_alpha_sweep;
//        ROS_INFO("LIFT: too hight! - Inverted direction of sweep");
    }
    else if (alpha_lift_<min_lift_){
        alpha_lift_=min_lift_;
//        ROS_INFO("LIFT: too low!");
    }

    ros::Time time_now=ros::Time::now();
    if((fabs(alpha_lift_-current_lift_) >= lift_tolerance_)&&( time_now > last_lift_pub + ros::Duration(0.5))){
    std_msgs::Float64 lift_pub;
    lift_pub.data=alpha_lift_;
    lift_pub_.publish(lift_pub);
    last_lift_pub=time_now;
    ROS_INFO("alpha_lift_: %f current: %f", alpha_lift_, current_lift_);
    }


//    ROS_INFO("Max_height: %lf frameid: %s", lift_, temp_cloud_filtered_->header.frame_id.c_str());

//    ROS_INFO("Sweeper - %s - Filtered the cloud!!!", __FUNCTION__);

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

//    ros::Time time_now=ros::Time::now();

//    if( time_now > last_lift_pub + ros::Duration(0.5)){
        calc_new_lift();
//        last_lift_pub=time_now;
//    }

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

    // Step 2. Move to next goal
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.header.stamp = ros::Time::now();
    goal.trajectory.points.resize(1);
    goal.trajectory.joint_names.push_back("upper_arm_joint");
    goal.trajectory.points[0].positions.push_back(alpha_lift_);
    goal.trajectory.points[0].velocities.push_back(0.0);
    goal.trajectory.points[0].accelerations.push_back(0.05);
    goal.trajectory.joint_names.push_back("arm_axel_joint");
    goal.trajectory.points[0].positions.push_back(sweep_);
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


//    ROS_INFO("Sweeper - %s - Lift c:%lf d:%lf Sweep c:%lf d:%lf", __FUNCTION__, current_lift_, lift_, current_sweep_, sweep_);

    }

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
