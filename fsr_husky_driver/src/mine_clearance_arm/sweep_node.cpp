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
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/move_group/move_group_capability.h>
#include <moveit_msgs/GetPlanningScene.h>

/*bool checkHeight(const robot_state::RobotState &kinematic_state, bool verbose)
{
    const Eigen::Affine3d &end_effector_state = kinematic_state.getGlobalLinkTransform(group.getEndEffectorLink().c_str());

}*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sweep_node");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    double max;
    pn.param("max", max, 0.8);
    double step;
    pn.param("step", step, 0.05);
    double height;
    pn.param("distance_from_ground", height, 0.1);
    double speed;
    pn.param("speed", speed, 0.8);

    moveit::planning_interface::MoveGroup group("arm");

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    planning_scene::PlanningScene planning_scene(kinematic_model);
    //planning_scene.setStateFeasibilityPredicate(checkHeight);

    ros::ServiceClient client_get_scene = n.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
    moveit_msgs::GetPlanningScene scene_srv;
    scene_srv.request.components.components = scene_srv.request.components.OCTOMAP;

    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;

    collision_request.contacts = true;
    collision_request.max_contacts = 1000;
    collision_request.group_name = "arm";

    collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();

    robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst();
    const robot_model::JointModelGroup* joint_model_group = current_state.getJointModelGroup("arm");
    std::vector<double> joint_values;
    current_state.copyJointGroupPositions(joint_model_group, joint_values);

    double position = max;
    while(ros::ok())
    {
        // Step 0. Update the planning scene
        if(!client_get_scene.call(scene_srv))
        {
            ROS_WARN("Failed to call service /get_planning_scene");
        }
        else
        {
            planning_scene.setPlanningSceneMsg(scene_srv.response.scene);
        }

        // Step 1. Define one of the sweep limits
        joint_values[0] = position = position == -1*max ? max : -1*max;

        // Step 2. Look for the first lift angle that is not in collision with the octomap
        joint_values[1] = 0.25;
        for(int i=0 ; i<10 ; i++)
        {

            joint_values[1] -= step;
            group.setJointValueTarget(joint_values);
            current_state.setJointGroupPositions(joint_model_group, joint_values);
            collision_result.clear();
            planning_scene.checkCollision(collision_request, collision_result, current_state, acm);
            ROS_INFO("Joint value %lf Collision %s", joint_values[1], collision_result.collision ? "True" : "False");
            if(!collision_result.collision)
            {
                break;
                joint_values[1] -= height;
                group.setJointValueTarget(joint_values);
            }
        }

        moveit::planning_interface::MoveGroup::Plan my_plan;
        double success = group.plan(my_plan);

        if(success) group.move();
    }

    ros::shutdown();
    return 0;
}
