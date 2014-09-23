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
* Author: Baptiste Gil on 01/09/2014
*********************************************************************/

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "dynamic_reconfigure/Reconfigure.h"
#include <cstdlib>

double max_sweep_vel_;
double max_vel_;
ros::ServiceClient client;

void flag_msgCallback(const std_msgs::String& msg)
{

    ///////////////////Dynamic Reconfig
    dynamic_reconfigure::Reconfigure srv;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::Config conf;


    //Entering values using Dynamic Reconfig
    double_param.name = "max_vel_x";

  if(msg.data == "start")
  {
      ROS_INFO("Sweep initiated: max_vel=%lf",max_sweep_vel_);
      double_param.value = max_sweep_vel_;

  }
  else if(msg.data == "stop")
  {
       ROS_INFO("Sweep paused: max_vel=%lf",max_vel_);
       double_param.value = max_vel_;
  }

  conf.doubles.push_back(double_param);

  srv.request.config=conf;


   if (!client.call(srv))
     ROS_ERROR("Failed to call service max_vel_x");

}


 int main(int argc, char **argv)
 {
   fprintf(stderr,"\nVelocity control...\n");

   ros::init(argc, argv, "Velocity_controller");
   ros::NodeHandle n;
//   ros::Rate r(10);

  // Default value version
   n.param("max_sweep_vel_", max_sweep_vel_, 0.25);
   n.param("max_vel_", max_vel_, 0.5);

   client = n.serviceClient<dynamic_reconfigure::Reconfigure>("/move_base/TrajectoryPlannerROS/set_parameters");
   ros::Subscriber msg_sub_ = n.subscribe("sweep_state", 1, flag_msgCallback);

   ros::spin();

 }

