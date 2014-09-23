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
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PointStamped.h>
#include "geometry_msgs/Point.h"
#include <math.h>
#include "mine_mapping/Mines.h"

double tank_mine_size, tank_mine_height, personal_mine_size, personal_mine_height, mine_resolution;
unsigned int num_points=0;

//std::string frame_id;
sensor_msgs::PointCloud cloud;
ros::Publisher cloud_pub_;


void mine_to_cloud (double size_mine, double height_mine, geometry_msgs::Point position){

    //Resize the cloud
    int added_points=ceil(size_mine*height_mine/pow(mine_resolution,3));
    num_points+=added_points;
    cloud.points.resize(num_points);

    unsigned count=0;

    //Fills the space of the mine in the cloud
    for(int i=0; (double)(i*mine_resolution)<size_mine; i++){
        for(int j=0; (double)(j*mine_resolution)<size_mine; j++){
            for(int k=0; (double)(k*mine_resolution)<height_mine; k++){
                cloud.points[num_points-added_points+count].x = position.x-size_mine/2+i*mine_resolution;
                cloud.points[num_points-added_points+count].y = position.y-size_mine/2+j*mine_resolution;
                cloud.points[num_points-added_points+count].z = k*mine_resolution; //position.z-size_mine/2+j;
                //cloud.channels[0].values[num_points-added_points+count] = 100 + i + j;
                count++;
            }
        }
    }
}


void minesCallback(const mine_mapping::Mines& msg){

    //Clean the point cloud
    num_points=0;
//    cloud.header.stamp = ros::Time::now();
//    cloud.header.frame_id = frame_id;
    cloud.header.stamp = msg.header.stamp;
    cloud.header.frame_id = msg.header.frame_id;

    for(int mine_it= 0 ; mine_it < msg.mine.size() ; mine_it++)
//    for(std::vector<Mine>::iterator mine_it = msg.mine.begin() ; mine_it != msg.mine.end() ; ++mine_it)
    {
        geometry_msgs::Point mine_position;
        mine_position.x = msg.mine.at(mine_it).x;
        mine_position.y = msg.mine.at(mine_it).y;
//        mine_position.z = msg.mine.at(mine_it).position_zx;

        //Print this mine into the point cloud
        if(msg.type.at(mine_it)==mine_mapping::Mines::ANTI_PERSONAL)
            mine_to_cloud (personal_mine_size,personal_mine_height, mine_position);
        else if (msg.type.at(mine_it)==mine_mapping::Mines::ANTI_TANK)
            mine_to_cloud (tank_mine_size,tank_mine_height, mine_position);
        else{
            ROS_INFO("Type_error %d",(int)msg.type.at(mine_it));
            mine_to_cloud (tank_mine_size,tank_mine_height, mine_position);
        }

    }

    //Publish virtual mines point cloud
    cloud_pub_.publish(cloud);

}


int main(int argc, char** argv){

    ros::init(argc, argv, "mines_converter_subscriber");
    ros::NodeHandle n;

    n.param("tank_mine_size", tank_mine_size, 0.3);
    n.param("tank_mine_height", tank_mine_height, 0.3);
    n.param("personal_mine_size", personal_mine_size, 0.1);
    n.param("personal_mine_height", personal_mine_height, 0.1);
    n.param("mine_resolution", mine_resolution, 0.01);
//    n.param<std::string>("mines_frame_id", frame_id, "minefield");

    ros::Subscriber mines_sub_ = n.subscribe("mines", 100, minesCallback);
    cloud_pub_ = n.advertise<sensor_msgs::PointCloud>("virtual_metal_cloud", 1, true);

    ROS_INFO("ANTI_PERSONAL mine size: %lfm",personal_mine_size);
    ROS_INFO("TANK_PERSONAL mine size: %lfm",tank_mine_size);
    ROS_INFO("Fill resolution: %lfm",mine_resolution);

    while(ros::ok()){

        ros::spinOnce();

    }
}
