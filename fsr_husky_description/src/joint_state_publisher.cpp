
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joint_state_publisher");
    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<sensor_msgs::JointState>("joint_states", 10);

    sensor_msgs::JointState joint_state;

    joint_state.name.resize(4);
    joint_state.position.resize(4);
    joint_state.velocity.resize(4);

    joint_state.name[0] = "joint_back_left_wheel";
    joint_state.position[0] = 0.0;
    joint_state.velocity[0] = 0.0;
    joint_state.name[1] = "joint_back_right_wheel";
    joint_state.position[1] = 0.0;
    joint_state.velocity[1] = 0.0;
    joint_state.name[2] = "joint_front_left_wheel";
    joint_state.position[2] = 0.0;
    joint_state.velocity[2] = 0.0;
    joint_state.name[3] = "joint_front_right_wheel";
    joint_state.position[3] = 0.0;
    joint_state.velocity[3] = 0.0;

    ros::Rate r(50.0);
    while(ros::ok())
    {
        joint_state.header.stamp = ros::Time::now();
        pub.publish(joint_state);

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
