

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
        ros::Publisher  m_actuator_pub;
        ros::Subscriber m_joint_sub;

        std::string base_frame_id_;
        std::string linear_frame_id_;
        std::string rotation_frame_id_;

        std::string linear_joint_;
        std::string rotation_joint_;

        ros::Time last_update_;
        double last_linear_position_;
        double last_rotation_position_;

        int max_position_;

        double min_linear_position_;
        double max_linear_position_;
        double min_angular_position_;
        double max_angular_position_;
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

    m_private_node.param("min_linear_position", min_linear_position_, -0.5236);
    m_private_node.param("max_linear_position", max_linear_position_, 0.5236);

    std::string rotation_port;
    m_private_node.param<std::string>("rotation_actuator_port", rotation_port, "/dev/ttyACM1");
    int rotation_baudrate;
    m_private_node.param("rotation_actuator_baudrate", rotation_baudrate, 115200);

    m_private_node.param("min_angular_position", min_angular_position_, -0.5236);
    m_private_node.param("max_angular_position", max_angular_position_, 0.5236);

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
    m_actuator_pub = m_node.advertise<sensor_msgs::JointState>("actuator_states", 10);
    m_joint_sub = m_node.subscribe<sensor_msgs::JointState>("cmd", 10, &FSRHuskyArm::setGoal, this);
}

void FSRHuskyArm::home(double timeout)
{
    ROS_INFO("FSR Husky Arm - %s - Starting homing...", __FUNCTION__);

    if(!nanotec.startHoming())
    {
        ROS_FATAL("FSR Husky Arm - %s - Failed to start the homing routine!", __FUNCTION__);
        ROS_BREAK();
    }

    ros::Rate r(10.0);
    ros::Time start_time = ros::Time::now();
    while(nanotec.getStatus() != NANOTEC_STATUS_ZERO && ros::Time::now() - start_time < ros::Duration(timeout))
    {
        r.sleep();
    }
    if(nanotec.getStatus() != NANOTEC_STATUS_ZERO)
    {
        ROS_FATAL("FSR Husky Arm - %s - Timeout before the arm could complete the homing routine!", __FUNCTION__);
        ROS_BREAK();
    }

    ros::Duration(1.0).sleep();

    if(!nanotec.setPositionMode())
    {
        ROS_FATAL("FSR Husky Arm - %s - Failed to switch to absolute position mode!", __FUNCTION__);
        ROS_BREAK();
    }

    if(!nanotec.setPosition(5000))
    {
        ROS_FATAL("FSR Husky Arm - %s - Failed to complete the homing routine!", __FUNCTION__);
        ROS_BREAK();
    }

    start_time = ros::Time::now();
    while(nanotec.getStatus() != NANOTEC_STATUS_LIMIT && ros::Time::now() - start_time < ros::Duration(timeout))
    {
        r.sleep();
    }
    if(nanotec.getStatus() != NANOTEC_STATUS_LIMIT)
    {
        ROS_FATAL("FSR Husky Arm - %s - Timeout before the arm could complete the homing routine!", __FUNCTION__);
        ROS_BREAK();
    }

    if(!nanotec.clearPositionError())
    {
        ROS_FATAL("FSR Husky Arm - %s - Failed to complete the homing routine!", __FUNCTION__);
        ROS_BREAK();
    }

    if(!nanotec.getPosition(max_position_))
    {
        ROS_FATAL("FSR Husky Arm - %s - Failed to complete the homing routine!", __FUNCTION__);
        ROS_BREAK();
    }
    ROS_INFO("FSR Husky Arm - %s - Limit reached at %d steps.", __FUNCTION__, max_position_);

    ROS_INFO("FSR Husky Arm - %s - Homing complete!", __FUNCTION__);
}

void FSRHuskyArm::setGoal(const sensor_msgs::JointState::ConstPtr& msg)
{
    double l = msg->position[0];
    if(l < min_linear_position_) l = min_linear_position_;
    if(l > max_linear_position_) l = max_linear_position_;
    int goal_l = (l - min_linear_position_)*4048/(max_linear_position_ - min_linear_position_);

    double a = msg->position[1];
    if(a < min_angular_position_) a = min_angular_position_;
    if(a > max_angular_position_) a = max_angular_position_;
    int goal_a = (a - min_angular_position_)*max_position_/(max_angular_position_ - min_angular_position_);

    jrk.setPosition(goal_l);
    nanotec.setPosition(goal_a);
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
    double l = linear_position*(max_linear_position_ - min_linear_position_)/4048 + min_linear_position_;
    double linear_speed = (l - last_linear_position_)/delta_t.toSec();

    int rotation_position;
    if(!nanotec.getPosition(rotation_position))
    {
        ROS_WARN("FSR Husky Arm - %s - Failed to update the position of the rotation actuator!", __FUNCTION__);
        return;
    }
    delta_t = ros::Time::now() - last_update_;
    double a = rotation_position*(max_angular_position_ - min_angular_position_)/max_position_ + min_angular_position_;
    double rotation_speed = (a - last_rotation_position_)/delta_t.toSec();

    sensor_msgs::JointState joint_state;
    ros::Time now = ros::Time::now();
    joint_state.header.stamp = now;
    joint_state.header.frame_id = base_frame_id_;

    joint_state.name.resize(2);
    joint_state.position.resize(2);
    joint_state.velocity.resize(2);
    joint_state.name[0] = linear_joint_;
    joint_state.position[0] = linear_position;
    joint_state.velocity[0] = 0.0;
    joint_state.name[1] = rotation_joint_;
    joint_state.position[1] = rotation_position;
    joint_state.velocity[1] = 0.0;
    m_actuator_pub.publish(joint_state);

    joint_state.name.resize(4);
    joint_state.position.resize(4);
    joint_state.velocity.resize(4);
    joint_state.name[0] = "arm_axel_joint";
    joint_state.position[0] = a;
    joint_state.velocity[0] = rotation_speed;
    joint_state.name[1] = "upper_arm_joint";
    joint_state.position[1] = l;
    joint_state.velocity[1] = linear_speed;
    joint_state.name[2] = "lower_arm_joint";
    joint_state.position[2] = l;
    joint_state.velocity[2] = linear_speed;
    joint_state.name[3] = "metal_detector_arm_joint";
    joint_state.position[3] = -l;
    joint_state.velocity[3] = linear_speed;
    m_joint_pub.publish(joint_state);

    last_update_ = ros::Time::now();
    last_linear_position_ = l;
    last_rotation_position_ = a;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fsr_husky_arm");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    ROS_INFO("FSR Husky Arm -- ROS Hardware Driver");

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
