#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <fsr_husky_driver/laser_tiltAction.h>
#include <laser_assembler/AssembleScans2.h>
#include <sensor_msgs/PointCloud2.h>

int main (int argc, char **argv)
{
    ros::init(argc, argv, "continuous_tilt_operation");
    ros::NodeHandle nh_;
    ros::NodeHandle pn_("~");
    double min_angle_, max_angle_, speed_, timeout_;
    pn_.param("lower_tilt", min_angle_, -0.6);
    pn_.param("upper_tilt", max_angle_, 0.0);
    pn_.param("timeout", timeout_, 5.0);
    pn_.param("tilt_speed", speed_, 1.17);        // not used yet


    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<fsr_husky_driver::laser_tiltAction> ac("laser_tilt_action_server", true);
    ros::Publisher cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("assembled_cloud", 1);
    ros::ServiceClient client = nh_.serviceClient<laser_assembler::AssembleScans2>("assemble_scans2");
    laser_assembler::AssembleScans2 assembler_srv;

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time

    ROS_INFO("Action server started, sending goal.");
    // send a goal to the action

    int move_state = 1;

    while(ros::ok())
    {
        fsr_husky_driver::laser_tiltGoal goal;

        assembler_srv.request.begin = ros::Time::now();

        if (move_state==0)
            goal.target_position = max_angle_;
        else
            goal.target_position = min_angle_;

        ac.sendGoal(goal);

        //wait for the action to return
        bool finished_before_timeout = ac.waitForResult(ros::Duration(timeout_));

        if (finished_before_timeout)
        {
//            actionlib::SimpleClientGoalState state = ac.getState();
//            ROS_INFO("Action finished: %s",state.toString().c_str());

            assembler_srv.request.end = ros::Time::now();

            if(client.call(assembler_srv))
            {
                cloud_pub.publish(assembler_srv.response.cloud);
            }
            else
            {
                ROS_WARN("PointCloud Generator - Service call failed!");
            }

            if (move_state==0)
                move_state = 1;
            else
                move_state = 0;
        }
        else
            ROS_INFO("Action did not finish before the time out.");
    }
    //exit
    return 0;
}
