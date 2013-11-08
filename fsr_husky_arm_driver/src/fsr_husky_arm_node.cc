

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "jrk_driver.h"
#include "nanotec_driver.h"

class FSRHuskyArm
{
    public:
        FSRHuskyArm(ros::NodeHandle& node_handle, ros::NodeHandle& private_node_handle);
        ~FSRHuskyArm();

        void init();
        void home(double timeout=30.0);
        void setGoal(const sensor_msgs::JointState::ConstPtr& msg);
        void update();

    private:
        JRK jrk;
        Nanotec nanotec;

        ros::NodeHandle m_node;
        ros::NodeHandle m_private_node;
        ros::Publisher  m_joint_pub;
        ros::Subscriber m_joint_sub;

        std::string base_frame_id_;
        std::string linear_frame_id_;
        std::string rotation_frame_id_;

        std::string linear_joint_;
        std::string rotation_joint_;

        ros::Time last_update_;
        double last_linear_position_;
        double last_rotation_position_;
};

FSRHuskyArm::FSRHuskyArm(ros::NodeHandle& node_handle, ros::NodeHandle& private_node_handle) : jrk(), m_node(node_handle), m_private_node(private_node_handle)
{
    m_private_node.param<std::string>("base_frame_id", base_frame_id_, "arm_base_link");
    m_private_node.param<std::string>("linear_frame_id", linear_frame_id_, "arm_linear_link");
    m_private_node.param<std::string>("rotation_frame_id", rotation_frame_id_, "arm_rotation_link");

    m_private_node.param<std::string>("linear_joint", linear_joint_, "arm_linear_joint");
    m_private_node.param<std::string>("rotation_joint", rotation_joint_, "arm_rotation_joint");
}

FSRHuskyArm::~FSRHuskyArm()
{

}

void FSRHuskyArm::init()
{
    // Query for serial configuration
    std::string linear_port;
    m_private_node.param<std::string>("linear_actuator_port", linear_port, "/dev/ttyACM0");
    int linear_baudrate;
    m_private_node.param("linear_actuator_baudrate", linear_baudrate, 115200);

    std::string rotation_port;
    m_private_node.param<std::string>("rotation_actuator_port", rotation_port, "/dev/ttyACM0");
    int rotation_baudrate;
    m_private_node.param("rotation_actuator_baudrate", rotation_baudrate, 115200);

    if(!jrk.init(&linear_port, linear_baudrate))
    {
        ROS_FATAL("FSR Husky Arm - %s - Failed to initialize the linear actuator!", __FUNCTION__);
        ROS_BREAK();
    }
    if(!nanotec.init(&rotation_port, rotation_baudrate))
    {
        ROS_FATAL("FSR Husky Arm - %s - Failed to initialize the rotation actuator!", __FUNCTION__);
        ROS_BREAK();
    }

    m_joint_pub = m_node.advertise<sensor_msgs::JointState>("joint_states", 10);
    m_joint_sub = m_node.subscribe<sensor_msgs::JointState>("cmd", 10, &FSRHuskyArm::setGoal, this);
}

void FSRHuskyArm::home(double timeout)
{
    if(!nanotec.startHoming())
    {
        ROS_FATAL("FSR Husky Arm - %s - Failed to start the homing routine!", __FUNCTION__);
        ROS_BREAK();
    }

    ros::Rate r(10.0);
    ros::Time start_time = ros::Time::now();
    while(!nanotec.positionError() && ros::Time::now() - start_time < ros::Duration(timeout))
    {
        r.sleep();
    }
    if(!nanotec.positionError())
    {
        ROS_FATAL("FSR Husky Arm - %s - Timeout before the arm could complete the homing routine!", __FUNCTION__);
        ROS_BREAK();
    }

    if(!nanotec.clearPositionError())
    {
        ROS_FATAL("FSR Husky Arm - %s - Failed to complete the homing routine!", __FUNCTION__);
        ROS_BREAK();
    }
}

void FSRHuskyArm::setGoal(const sensor_msgs::JointState::ConstPtr& msg)
{
    jrk.setPosition(int(msg->position[0]));
    nanotec.setPosition(int(msg->position[1]));
}

void FSRHuskyArm::update()
{
    int linear_position;
    if(!jrk.getPosition(linear_position))
    {
        ROS_WARN("FSR Husky Arm - %s - Failed to update the position of the linear actuator!", __FUNCTION__);
        return;
    }
    ros::Duration delta_t = ros::Time::now() - last_update_;
    double linear_speed = (linear_position - last_linear_position_)/delta_t.toSec();

    int rotation_position;
    if(!nanotec.getPosition(rotation_position))
    {
        ROS_WARN("FSR Husky Arm - %s - Failed to update the position of the rotation actuator!", __FUNCTION__);
        return;
    }
    delta_t = ros::Time::now() - last_update_;
    double rotation_speed = (rotation_position - last_rotation_position_)/delta_t.toSec();

    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.header.frame_id = base_frame_id_;
    joint_state.name.resize(2);
    joint_state.position.resize(2);
    joint_state.velocity.resize(2);
    joint_state.name[0] = linear_joint_;
    joint_state.position[0] = linear_position;
    joint_state.velocity[0] = linear_speed;
    joint_state.name[1] = rotation_joint_;
    joint_state.position[1] = rotation_position;
    joint_state.velocity[1] = rotation_speed;
    m_joint_pub.publish(joint_state);

    last_update_ = ros::Time::now();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fsr_husky_arm");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    FSRHuskyArm arm(n, pn);
    arm.init();
    arm.home();

    int rate;
    pn.param("rate", rate, 20);
    ros::Rate r(rate);

    while(ros::ok())
    {
        arm.update();
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
