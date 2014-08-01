#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <dynamixel_msgs/JointState.h>


ros::Publisher * joint_state_pub;

void publishStateCallback(const dynamixel_msgs::JointState::ConstPtr& msg){

        sensor_msgs::JointState joint_state;

        joint_state.header.stamp = msg->header.stamp;
        joint_state.header.frame_id = msg->header.frame_id;

        joint_state.name.push_back(msg->name);
        joint_state.position.push_back(msg->current_pos);
        joint_state.effort.push_back(msg->load);
        joint_state.velocity.push_back(msg->velocity);


        joint_state_pub->publish(joint_state);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tilt_joint_state_publisher");

    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    std::string topic;
    pn.param<std::string>("joint_topic",topic,"/ptu_d46_tilt_controller/state");

    ros::Publisher pub = n.advertise<sensor_msgs::JointState>("joint_states", 10);

    joint_state_pub=&pub;

    ros::Subscriber sub = n.subscribe<dynamixel_msgs::JointState>(topic,10,publishStateCallback);

    ros::spin();


    return 0;
}
