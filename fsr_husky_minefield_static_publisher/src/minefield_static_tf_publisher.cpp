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
* Author: Goncalo Cabrita on 21/04/2014
* Updated: Baptiste Gil on 26/08/2014
*
* This software publishes a static transform between the world frame
* and the minefield frame.
*
* The minefield frame is centered on the 4 GPS coordinates that define
* the minefield itself.
*
*   1 ------------------------------------------------------------ 4
*   |                                                              |
*   |                              y                               |
*   |                              ^                               |
*   |                              |                               |
*   |                              o-- > x                         |
*   |                            z                                 |
*   |                                                              |
*   |                                                              |
*   |                                                              |
*   2 ------------------------------------------------------------ 3
*
*********************************************************************/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <angles/angles.h>

#include <UTMConverter/UTMConverter.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "minefield_static_tf_publisher");

    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    // Convert the GPS coordinates into UTM coordinates
    UTMCoordinates utm;
    sensor_msgs::NavSatFix fix;

    if(!pn.hasParam("minefield/corner1/latitude") && !pn.hasParam("minefield/corner1/longitude") && !pn.hasParam("minefield/corner1/altitude"))
    {
        ROS_FATAL("Minefield static tf broadcaster -- Unable to start without the 4 corners of the minefield!!!");
        ROS_BREAK();
    }
    pn.getParam("minefield/corner1/latitude", fix.latitude);
    pn.getParam("minefield/corner1/longitude", fix.longitude);
    pn.getParam("minefield/corner1/altitude", fix.altitude);

    UTMConverter::latitudeAndLongitudeToUTMCoordinates(fix, utm);
    tf::Vector3 corner1 = tf::Vector3(utm.easting, utm.northing, fix.altitude);

    ROS_INFO("Minefield static tf broadcaster -- Corner1 lat:%lf long:%lf alt:%lf - x:%lf y:%lf z:%lf", fix.latitude, fix.longitude, fix.altitude, corner1.x(), corner1.y(), corner1.z());

    if(!pn.hasParam("minefield/corner2/latitude") && !pn.hasParam("minefield/corner2/longitude") && !pn.hasParam("minefield/corner2/altitude"))
    {
        ROS_FATAL("Minefield static tf broadcaster -- Unable to start without the 4 corners of the minefield!!!");
        ROS_BREAK();
    }
    pn.getParam("minefield/corner2/latitude", fix.latitude);
    pn.getParam("minefield/corner2/longitude", fix.longitude);
    pn.getParam("minefield/corner2/altitude", fix.altitude);

    UTMConverter::latitudeAndLongitudeToUTMCoordinates(fix, utm);
    tf::Vector3 corner2 = tf::Vector3(utm.easting, utm.northing, fix.altitude);

    ROS_INFO("Minefield static tf broadcaster -- Corner2 lat:%lf long:%lf alt:%lf - x:%lf y:%lf z:%lf", fix.latitude, fix.longitude, fix.altitude, corner2.x(), corner2.y(), corner2.z());

    if(!pn.hasParam("minefield/corner3/latitude") && !pn.hasParam("minefield/corner3/longitude") && !pn.hasParam("minefield/corner3/altitude"))
    {
        ROS_FATAL("Minefield static tf broadcaster -- Unable to start without the 4 corners of the minefield!!!");
        ROS_BREAK();
    }
    pn.getParam("minefield/corner3/latitude", fix.latitude);
    pn.getParam("minefield/corner3/longitude", fix.longitude);
    pn.getParam("minefield/corner3/altitude", fix.altitude);

    UTMConverter::latitudeAndLongitudeToUTMCoordinates(fix, utm);
    tf::Vector3 corner3 = tf::Vector3(utm.easting, utm.northing, fix.altitude);

    ROS_INFO("Minefield static tf broadcaster -- Corner3 lat:%lf long:%lf alt:%lf - x:%lf y:%lf z:%lf", fix.latitude, fix.longitude, fix.altitude, corner3.x(), corner3.y(), corner3.z());

    if(!pn.hasParam("minefield/corner4/latitude") && !pn.hasParam("minefield/corner4/longitude") && !pn.hasParam("minefield/corner4/altitude"))
    {
        ROS_FATAL("Minefield static tf broadcaster -- Unable to start without the 4 corners of the minefield!!!");
        ROS_BREAK();
    }
    pn.getParam("minefield/corner4/latitude", fix.latitude);
    pn.getParam("minefield/corner4/longitude", fix.longitude);
    pn.getParam("minefield/corner4/altitude", fix.altitude);

    UTMConverter::latitudeAndLongitudeToUTMCoordinates(fix, utm);
    tf::Vector3 corner4 = tf::Vector3(utm.easting, utm.northing, fix.altitude);

    ROS_INFO("Minefield static tf broadcaster -- Corner4 lat:%lf long:%lf alt:%lf - x:%lf y:%lf z:%lf", fix.latitude, fix.longitude, fix.altitude, corner4.x(), corner4.y(), corner4.z());

    double w =  ( sqrt(pow(corner4.x()-corner1.x(), 2) + pow(corner4.y()-corner1.y(), 2) + pow(corner4.z()-corner1.z(), 2)) + sqrt(pow(corner3.x()-corner2.x(), 2) + pow(corner3.y()-corner2.y(), 2) + pow(corner3.z()-corner2.z(), 2)) ) / 2.0;
    double h =  ( sqrt(pow(corner2.x()-corner1.x(), 2) + pow(corner2.y()-corner1.y(), 2) + pow(corner2.z()-corner1.z(), 2)) + sqrt(pow(corner4.x()-corner3.x(), 2) + pow(corner4.y()-corner3.y(), 2) + pow(corner4.z()-corner3.z(), 2)) ) / 2.0;

    ROS_INFO("Minefield static tf broadcaster:");
    ROS_INFO(" -- The minefield is %lfx%lf (m)", w, h);
    ROS_INFO(" -- With the origin at corner_1 ");

    double rate;
    pn.param("rate", rate, 20.0);

    std::string parent_frame;
    pn.param<std::string>("parent_frame", parent_frame, "odom");

    std::string child_frame;
    pn.param<std::string>("child_frame", child_frame, "minefield");

    ROS_INFO("Minefield static tf broadcaster -- Publishing static transform from %s to %s at %lf hz...", parent_frame.c_str(), child_frame.c_str(), rate);

    // Generate the transformation from the world frame to the minefield frame
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(corner1.x(), corner1.y(), corner1.z()) );
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, atan2((corner4.y()+corner3.y())/2.0 - (corner1.y()+corner2.y())/2.0, (corner4.x()+corner3.x())/2.0 - (corner1.x()+corner2.x())/2.0));
    transform.setRotation(q);

    tf::Transform corner_1_;
    corner_1_.setOrigin( tf::Vector3(corner1.x()-transform.getOrigin().x(), corner1.y()-transform.getOrigin().y(), corner1.z()-transform.getOrigin().z()) );
    q.setRPY(0.0, 0.0, 0.0); // Use the same as /minefield
    corner_1_.setRotation(q);
    ROS_INFO(" -- Corner_1 at minefield is x:%lf y:%lf z:%lf (m)", corner_1_.getOrigin().x(),corner_1_.getOrigin().y(),corner_1_.getOrigin().z());

    tf::Transform corner_2_;
    corner_2_.setOrigin( tf::Vector3(corner2.x()-transform.getOrigin().x(), corner2.y()-transform.getOrigin().y(), corner2.z()-transform.getOrigin().z()) );
    corner_2_.setRotation(q);
    ROS_INFO(" -- Corner_2 at minefield is x:%lf y:%lf z:%lf (m)", corner_2_.getOrigin().x(),corner_2_.getOrigin().y(),corner_2_.getOrigin().z());

    tf::Transform corner_3_;
    corner_3_.setOrigin( tf::Vector3(corner3.x()-transform.getOrigin().x(), corner3.y()-transform.getOrigin().y(), corner3.z()-transform.getOrigin().z()) );
    corner_3_.setRotation(q);
    ROS_INFO(" -- Corner_3 at minefield is x:%lf y:%lf z:%lf (m)", corner_3_.getOrigin().x(),corner_3_.getOrigin().y(),corner_3_.getOrigin().z());

    tf::Transform corner_4_;
    corner_4_.setOrigin( tf::Vector3(corner4.x()-transform.getOrigin().x(), corner4.y()-transform.getOrigin().y(), corner4.z()-transform.getOrigin().z()) );
    corner_4_.setRotation(q);
    ROS_INFO(" -- Corner_4 at minefield is x:%lf y:%lf z:%lf (m)", corner_4_.getOrigin().x(),corner_4_.getOrigin().y(),corner_4_.getOrigin().z());


    // Publish the transformation at the desired rate!
    ros::Rate loop(rate);
    while(ros::ok())
    {
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent_frame, child_frame));
        br.sendTransform(tf::StampedTransform(corner_1_, ros::Time::now(), child_frame, "corner_1"));
        br.sendTransform(tf::StampedTransform(corner_2_, ros::Time::now(), child_frame, "corner_2"));
        br.sendTransform(tf::StampedTransform(corner_3_, ros::Time::now(), child_frame, "corner_3"));
        br.sendTransform(tf::StampedTransform(corner_4_, ros::Time::now(), child_frame, "corner_4"));

        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}
