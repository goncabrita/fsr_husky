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
*********************************************************************/

#include <ros/ros.h>
#include <tf/tf.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTrajectoryControllerState.h>

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

#define ARM_LENGTH 0.8475

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class Sweeper
{
public:
    Sweeper(ros::NodeHandle &n, ros::NodeHandle &pn);

    void spinOnce();

private:
    void pointCloudCallback(const PointCloud::ConstPtr& msg);
    void jointsCallback(const control_msgs::JointTrajectoryControllerState::ConstPtr& msg);

    ros::Subscriber cloud_sub_;
    //message_filters::Subscriber<PointCloud> cloud_sub_;
    tf::TransformListener tf_;
    //tf::MessageFilter<PointCloud> * tf_filter_;

    ros::Publisher roi_cloud_pub_;
    ros::Publisher virtual_sensors_pub_;
    ros::Publisher points_pub_;
    ros::Publisher joints_pub_;
    ros::Subscriber joints_sub_;

    pcl::PointCloud<PointT>::Ptr cloud_filtered_;

    std::string base_link_frame_id_;
    std::string metal_detector_frame_id_;

    bool got_cloud_;

    std::vector<geometry_msgs::PoseStamped> virtual_sensors_;

    std::string lift_joint_;
    std::string sweep_joint_;

    double lift_;
    double sweep_;

    double current_lift_;
    double current_sweep_;

    double last_lift_;
    double last_sweep_;

    double min_lift_;
    double max_lift_;
    double max_lift_speed_;

    double min_sweep_;
    double max_sweep_;
    double sweep_speed_;
    double sweep_tolerance_;

    double height_;

    double k_;

    double filter_box_size_;

    double distance(const PointT &p1, const PointT &p2);

    ros::Time last_change_of_direction_;
    ros::Duration time_between_change_of_directions_;
};

Sweeper::Sweeper(ros::NodeHandle &n, ros::NodeHandle &pn)
{
    pn.param<std::string>("metal_detector_frame_id", metal_detector_frame_id_, "metal_detector_antenna_link");

    // Lets load the list of virtual sensors...
    XmlRpc::XmlRpcValue list_of_virtual_sensors;
    if( pn.getParam("virtual_sensors", list_of_virtual_sensors) )
    {
        ROS_ASSERT(list_of_virtual_sensors.getType() == XmlRpc::XmlRpcValue::TypeArray);

        for(int i=0 ; i<list_of_virtual_sensors.size() ; i+=6)
        {
            for(int j=i ; j<i+6 ; j++) ROS_ASSERT(list_of_virtual_sensors[j].getType() == XmlRpc::XmlRpcValue::TypeDouble);

            geometry_msgs::PoseStamped virtual_sensor;
            virtual_sensor.header.frame_id = metal_detector_frame_id_;
            virtual_sensor.pose.position.x = static_cast<double>(list_of_virtual_sensors[i]);
            virtual_sensor.pose.position.y = static_cast<double>(list_of_virtual_sensors[i+1]);
            virtual_sensor.pose.position.z = static_cast<double>(list_of_virtual_sensors[i+2]);
            virtual_sensor.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(static_cast<double>(list_of_virtual_sensors[i+3]), static_cast<double>(list_of_virtual_sensors[i+4]), static_cast<double>(list_of_virtual_sensors[i+5]));
            virtual_sensors_.push_back(virtual_sensor);
        }

        if(virtual_sensors_.size() == 0)
        {
            ROS_FATAL("Sweeper -- Unable to parse virtual sensors...");
            ROS_BREAK();
        }
    }
    else
    // If a list of virtual sensors was not defined...
    {
        ROS_FATAL("Sweeper -- A list of virtual sensors was not provided, aborting...");
        ROS_BREAK();
    }

    pn.param<std::string>("base_link_frame_id", base_link_frame_id_, "base_link");

    cloud_sub_ = n.subscribe<PointCloud>("/cloud", 1, &Sweeper::pointCloudCallback, this);
    /*cloud_sub_.subscribe(n, "/cloud", 1);
    tf_filter_ = new tf::MessageFilter<PointCloud>(cloud_sub_, tf_, base_link_frame_id_, 10);
    tf_filter_->registerCallback( boost::bind(&Sweeper::pointCloudCallback, this, _1) );*/

    roi_cloud_pub_ = n.advertise<PointCloud>("roi_cloud", 1);
    virtual_sensors_pub_ = n.advertise<geometry_msgs::PoseStamped>("virtual_sensors/pose", 10);
    points_pub_ = n.advertise<geometry_msgs::PointStamped>("virtual_sensors/distance", 10);

    joints_pub_ = n.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command", 10);
    joints_sub_ = n.subscribe("/arm_controller/state", 10, &Sweeper::jointsCallback, this);

    pn.param<std::string>("lift_joint", lift_joint_, "upper_arm_joint");
    pn.param<std::string>("sweep_joint", sweep_joint_, "arm_axel_joint");

    pn.param("min_lift", min_lift_, -0.5);
    pn.param("max_lift", max_lift_, 0.5);
    pn.param("max_lift_speed", max_lift_speed_, 0.8);

    pn.param("min_sweep", min_sweep_, -0.6);
    pn.param("max_sweep", max_sweep_, 0.6);
    pn.param("sweep_speed", sweep_speed_, 0.8);
    pn.param("sweep_tolerance", sweep_tolerance_, 0.05);

    pn.param("height", height_, 0.10);

    pn.param("k", k_, 1.0);

    pn.param("filter_box_size", filter_box_size_, 10.0);

    double time_between_change_of_directions;
    pn.param("time_between_change_of_directions", time_between_change_of_directions, 1.0);
    time_between_change_of_directions_ = ros::Duration(time_between_change_of_directions);
    last_change_of_direction_ = ros::Time::now();

    cloud_filtered_ = pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>);
}

void Sweeper::pointCloudCallback(const PointCloud::ConstPtr& msg)
{
    ROS_INFO("Sweeper - %s - Got a cloud msg.", __FUNCTION__);

    geometry_msgs::PointStamped corners[8];

    corners[0].point.x = filter_box_size_/2.0;
    corners[0].point.y = filter_box_size_/2.0;
    corners[0].point.z = filter_box_size_/2.0;

    corners[1].point.x = filter_box_size_/-2.0;
    corners[1].point.y = filter_box_size_/2.0;
    corners[1].point.z = filter_box_size_/2.0;

    corners[2].point.x = filter_box_size_/-2.0;
    corners[2].point.y = filter_box_size_/-2.0;
    corners[2].point.z = filter_box_size_/2.0;

    corners[3].point.x = filter_box_size_/2.0;
    corners[3].point.y = filter_box_size_/-2.0;
    corners[3].point.z = filter_box_size_/2.0;

    corners[4].point.x = filter_box_size_/2.0;
    corners[4].point.y = filter_box_size_/2.0;
    corners[4].point.z = filter_box_size_/-2.0;

    corners[5].point.x = filter_box_size_/-2.0;
    corners[5].point.y = filter_box_size_/2.0;
    corners[5].point.z = filter_box_size_/-2.0;

    corners[6].point.x = filter_box_size_/-2.0;
    corners[6].point.y = filter_box_size_/-2.0;
    corners[6].point.z = filter_box_size_/-2.0;

    corners[7].point.x = filter_box_size_/2.0;
    corners[7].point.y = filter_box_size_/-2.0;
    corners[7].point.z = filter_box_size_/-2.0;

    pcl_conversions::fromPCL(msg->header, corners[0].header);
    corners[0].header.frame_id = base_link_frame_id_;
    tf_.waitForTransform(msg->header.frame_id, corners[0].header.frame_id, corners[0].header.stamp, ros::Duration(0.1));
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
    filter.setFilterFieldName("x");
    filter.setFilterLimits(min_p.x, max_p.x);
    filter.filter(*cloud_filtered_);
    filter.setInputCloud(cloud_filtered_);
    filter.setFilterFieldName("y");
    filter.setFilterLimits(min_p.y, max_p.y);
    filter.filter(*cloud_filtered_);
    filter.setFilterFieldName("z");
    filter.setFilterLimits(min_p.z, max_p.z);
    filter.filter(*cloud_filtered_);

    roi_cloud_pub_.publish(cloud_filtered_);

    got_cloud_ = true;
    ROS_INFO("Sweeper - %s - Filtered the cloud!!!", __FUNCTION__);
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

void Sweeper::spinOnce()
{
    if(!got_cloud_) return;

    // Step 1. Check if we should invert the sweep direction
    if(fabs(current_sweep_ - sweep_) <= sweep_tolerance_ && ros::Time::now() - last_change_of_direction_ > time_between_change_of_directions_)
    {
        sweep_ = sweep_ == min_sweep_ ? max_sweep_ : min_sweep_;
        last_change_of_direction_ = ros::Time::now();
    }

    // Step 2. Convert the virtual sensors into the frame of the cloud
    std::vector<geometry_msgs::PoseStamped> virtual_sensors = virtual_sensors_;
    virtual_sensors[0].header.stamp = ros::Time::now();
    tf_.waitForTransform(cloud_filtered_->header.frame_id, virtual_sensors[0].header.frame_id, virtual_sensors[0].header.stamp, ros::Duration(0.1));
    for(int i=0 ; i<virtual_sensors.size() ; i++)
    {
        virtual_sensors[i].header.stamp = virtual_sensors[0].header.stamp;

        try
        {
            tf_.transformPose(cloud_filtered_->header.frame_id, virtual_sensors[i], virtual_sensors[i]);
        }
        catch(tf::TransformException &ex)
        {
            ROS_ERROR("Sweeper - %s - Error: %s", __FUNCTION__, ex.what());
            return;
        }

        virtual_sensors_pub_.publish(virtual_sensors[i]);
    }

    // Step 3. Find the distance to the ground of all sensors a pick the closest to the ground
    std::vector<double> distances;
    for(int i=0 ; i<virtual_sensors.size() ; i++) distances.push_back(2.0);
    BOOST_FOREACH(const PointT& pt, cloud_filtered_->points)
    {
        for(int i=0 ; i<virtual_sensors.size() ; i++)
        {
            PointT sensor;
            sensor.x = virtual_sensors[i].pose.position.x;
            sensor.y = virtual_sensors[i].pose.position.y;
            sensor.z = virtual_sensors[i].pose.position.z;

            // First we convert pt into a point p along the axis on the same plane perpendicular to the axis
            tf::Quaternion q;
            tf::quaternionMsgToTF(virtual_sensors[i].pose.orientation, q);
            tf::Vector3 axis = tf::quatRotate(q, tf::Vector3(1, 0, 0));
            double t = (axis.x()*(pt.x - sensor.x) + axis.y()*(pt.y - sensor.y) + axis.z()*(pt.z - sensor.z))/(pow(axis.x(),2) + pow(axis.y(),2) + pow(axis.z(),2));

            PointT p;
            p.x = virtual_sensors[i].pose.position.x + axis.x() * t;
            p.y = virtual_sensors[i].pose.position.y + axis.y() * t;
            p.z = virtual_sensors[i].pose.position.z + axis.z() * t;

            // Then if this point is within the measurement range and is the smalles distance, its a keeper!
            if(distance(pt, p) < 0.01)
            {
                double d = distance(sensor, p);
                PointT pd;
                pd.x = sensor.x + axis.x() * d;
                pd.y = sensor.y + axis.y() * d;
                pd.z = sensor.z + axis.z() * d;
                if(distance(pd, p) > 0.01) d *= -1;
                if(d < distances[i]) distances[i] = d;
            }
        }
    }

    /*tf::Quaternion q;
    tf::quaternionMsgToTF(virtual_sensors[0].pose.orientation, q);
    tf::Vector3 axis = tf::quatRotate(q, tf::Vector3(1, 0, 0));

    geometry_msgs::PointStamped point;
    point.header.frame_id = virtual_sensors[0].header.frame_id;
    point.header.stamp = ros::Time::now();
    point.point.x = virtual_sensors[0].pose.position.x + 0.5 * axis.x();
    point.point.y = virtual_sensors[0].pose.position.y + 0.5 * axis.y();
    point.point.z = virtual_sensors[0].pose.position.z + 0.5 * axis.z();

    points_pub_.publish(point);*/

    // Step 4. Determine the new lift angle based on the distance to the ground
    double min_distance = double(*std::min_element(distances.begin(), distances.end()));
    double delta_h = height_ - min_distance;
    double current_h = ARM_LENGTH * sin(current_lift_);
    double desired_h = current_h + delta_h;
    if(desired_h > ARM_LENGTH) desired_h = ARM_LENGTH;
    else if(desired_h < -1.0*ARM_LENGTH) desired_h = -1.0*ARM_LENGTH;
    lift_ = asin(desired_h/ARM_LENGTH);

    ROS_INFO("Sweeper - %s - Min distance to the ground is %lf, delta is %lf, current h is %lf and the desired h is %lf", __FUNCTION__, min_distance, delta_h, current_h, desired_h);

    // If we are bumping into something we cannot go over, turn back!
    if(lift_ > max_lift_ || lift_ < min_lift_  && ros::Time::now() - last_change_of_direction_ > time_between_change_of_directions_)
    {
        lift_ = current_lift_;
        sweep_ = sweep_ == min_sweep_ ? max_sweep_ : min_sweep_;
        last_change_of_direction_ = ros::Time::now();
    }

    ROS_INFO("Sweeper - %s - Lift c:%lf d:%lf Sweep c:%lf d:%lf", __FUNCTION__, current_lift_, lift_, current_sweep_, sweep_);

    // Step 5. Send the new command to the arm
    trajectory_msgs::JointTrajectory msg;

    msg.header.stamp = ros::Time::now();
    msg.points.resize(1);
    msg.joint_names.push_back(lift_joint_);
    msg.points[0].positions.push_back(lift_);
    msg.points[0].velocities.push_back(max_lift_speed_);
    //msg.points[0].accelerations.push_back(0.0);
    msg.joint_names.push_back(sweep_joint_);
    msg.points[0].positions.push_back(sweep_);
    msg.points[0].velocities.push_back(sweep_speed_);
    //msg.points[0].accelerations.push_back(0.0);
    msg.points[0].time_from_start = ros::Duration(1.0);

    if(lift_ != last_lift_ || sweep_ != last_sweep_) joints_pub_.publish(msg);

    last_lift_ = lift_;
    last_sweep_ = sweep_;
}

double Sweeper::distance(const PointT &p1, const PointT &p2)
{
    return sqrt( pow(p1.x-p2.x, 2) + pow(p1.y-p2.y, 2) + pow(p1.z-p2.z, 2) );
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sweep_node");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    Sweeper sweeper(n, pn);

    ros::AsyncSpinner spinner(3);
    spinner.start();

    ros::Rate r(2.0);
    while(ros::ok())
    {
        sweeper.spinOnce();
        r.sleep();
    }

    spinner.stop();

    return 0;
}
