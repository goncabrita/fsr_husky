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
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <fsr_husky_driver/GotoAction.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sweep_node");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    double min;
    pn.param("min", min, -0.45);
    double max;
    pn.param("max", max, 0.45);
    double height;
    pn.param("height", height, 0.05);
    double speed;
    pn.param("speed", speed, 0.2);

    actionlib::SimpleActionClient<fsr_husky_driver::GotoAction> ac("fsr_husky_arm/goto", true);
    ROS_INFO("Sweep -- Waiting for the action server to start...");
    ac.waitForServer();
    ROS_INFO("Sweep -- Got it!");

    double position = min;

    while(ros::ok())
    {
        fsr_husky_driver::GotoGoal goal;
        goal.joint.header.stamp = ros::Time::now();
        goal.joint.name.resize(2);
        goal.joint.position.resize(2);
        goal.joint.velocity.resize(2);
        goal.joint.name[0] = "arm_linear_joint";
        goal.joint.position[0] = 0.0;
        goal.joint.velocity[0] = 0.0;
        goal.joint.name[1] = "arm_rotation_joint";
        goal.joint.position[1] = position = position == min ? max : min;
        goal.joint.velocity[1] = speed;

        ac.sendGoal(goal);

        // Wait for the action to return
        bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

        if(!finished_before_timeout)
        {
            actionlib::SimpleClientGoalState state = ac.getState();
            ROS_ERROR("Sweep -- %s!", state.toString().c_str());
        }
    }

    return 0;
}
