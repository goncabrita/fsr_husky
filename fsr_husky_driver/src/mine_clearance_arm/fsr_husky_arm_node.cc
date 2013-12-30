

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "jrk_driver.h"
#include "nanotec_driver.h"

#include <actionlib/server/simple_action_server.h>
#include <fsr_husky_driver/HomeAction.h>
#include <fsr_husky_driver/GotoAction.h>

class FSRHuskyArm
{
    public:
        FSRHuskyArm(ros::NodeHandle& node_handle, ros::NodeHandle& private_node_handle);
        ~FSRHuskyArm();

        void init();
        bool home(double timeout=30.0);
        void setGoal(const sensor_msgs::JointState::ConstPtr& msg);
        void update();

        actionlib::SimpleActionServer<fsr_husky_driver::HomeAction> m_as_home_;
        void asHomeCallback(const fsr_husky_driver::HomeGoalConstPtr &req);

        actionlib::SimpleActionServer<fsr_husky_driver::GotoAction> m_as_goto_;
        void asGotoCallback(const fsr_husky_driver::GotoGoalConstPtr &goal);

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

        double linear_position_;
        double angular_position_;
        double linear_speed_;
        double angular_speed_;

        double goal_tolerance_;
        ros::Duration timeout_;
};

FSRHuskyArm::FSRHuskyArm(ros::NodeHandle& node_handle, ros::NodeHandle& private_node_handle) : jrk(), m_node(node_handle), m_private_node(private_node_handle),
    m_as_home_(node_handle, "fsr_husky_arm/home", boost::bind(&FSRHuskyArm::asHomeCallback, this, _1), false),
    m_as_goto_(node_handle, "fsr_husky_arm/goto", boost::bind(&FSRHuskyArm::asGotoCallback, this, _1), false)
{
    m_private_node.param<std::string>("base_frame_id", base_frame_id_, "arm_base_link");
    m_private_node.param<std::string>("linear_frame_id", linear_frame_id_, "arm_linear_link");
    m_private_node.param<std::string>("rotation_frame_id", rotation_frame_id_, "arm_rotation_link");

    m_private_node.param<std::string>("linear_joint", linear_joint_, "arm_linear_joint");
    m_private_node.param<std::string>("rotation_joint", rotation_joint_, "arm_rotation_joint");

    m_private_node.param("goal_tolerance", goal_tolerance_, 0.01);
    double timeout;
    m_private_node.param("timeout", timeout, 60.0);
    timeout_ = ros::Duration(timeout);
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

    bool go_home;
    m_private_node.param("home", go_home, false);
    if(go_home)
    {
        if(!home())
        {
            ROS_FATAL("FSR Husky Arm - %s - Failed to perform the homing routine!!", __FUNCTION__);
            ROS_BREAK();
        }
    }

    m_joint_pub = m_node.advertise<sensor_msgs::JointState>("joint_states", 10);
    m_actuator_pub = m_node.advertise<sensor_msgs::JointState>("actuator_states", 10);
    m_joint_sub = m_node.subscribe<sensor_msgs::JointState>("cmd", 10, &FSRHuskyArm::setGoal, this);

    m_as_home_.start();
    m_as_goto_.start();
}

bool FSRHuskyArm::home(double timeout)
{
    ROS_INFO("FSR Husky Arm - %s - Starting homing...", __FUNCTION__);

    if(!nanotec.setDirection(NANOTEC_LEFT))
    {
        ROS_ERROR("FSR Husky Arm - %s - Failed to change the direction!", __FUNCTION__);
        return false;
    }

    if(!nanotec.startHoming())
    {
        ROS_ERROR("FSR Husky Arm - %s - Failed to start the homing routine!", __FUNCTION__);
        return false;
    }

    ros::Rate r(2.0);
    ros::Time start_time = ros::Time::now();
    nanotec.getStatus();
    while(!nanotec.zero_ && ros::Time::now() - start_time < ros::Duration(timeout) && ros::ok())
    {
        r.sleep();
        nanotec.getStatus();
    }
    if(!nanotec.zero_)
    {
        ROS_ERROR("FSR Husky Arm - %s - Timeout before the arm could complete the homing routine!", __FUNCTION__);
        return false;
    }

    ros::Duration(1.0).sleep();

    if(!nanotec.setPositionMode())
    {
        ROS_ERROR("FSR Husky Arm - %s - Failed to switch to absolute position mode!", __FUNCTION__);
        return false;
    }

    if(!nanotec.setPosition(5000))
    {
        ROS_ERROR("FSR Husky Arm - %s - Failed to complete the homing routine!", __FUNCTION__);
        return false;
    }
    ros::Duration(1.0).sleep();
    if(!nanotec.setPosition(5000))
    {
        ROS_ERROR("FSR Husky Arm - %s - Failed to complete the homing routine!", __FUNCTION__);
        return false;
    }

    start_time = ros::Time::now();
    nanotec.getStatus();
    while(!nanotec.error_ && ros::Time::now() - start_time < ros::Duration(timeout) && ros::ok())
    {
        r.sleep();
        nanotec.getStatus();
    }
    if(!nanotec.error_)
    {
        ROS_ERROR("FSR Husky Arm - %s - Timeout before the arm could complete the homing routine!", __FUNCTION__);
        return false;
    }

    ros::Duration(1.0).sleep();

    if(!nanotec.clearPositionError())
    {
        ROS_ERROR("FSR Husky Arm - %s - Failed to complete the homing routine!", __FUNCTION__);
        return false;
    }

    if(!nanotec.getPosition(max_position_))
    {
        ROS_ERROR("FSR Husky Arm - %s - Failed to complete the homing routine!", __FUNCTION__);
        return false;
    }
    ROS_INFO("FSR Husky Arm - %s - Limit reached at %d steps.", __FUNCTION__, max_position_);

    if(!nanotec.setPosition(max_position_/2))
    {
        ROS_ERROR("FSR Husky Arm - %s - Failed to complete the homing routine!", __FUNCTION__);
        return false;
    }

    ROS_INFO("FSR Husky Arm - %s - Homing complete!", __FUNCTION__);

    return true;
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
    //ROS_INFO("linear position:%d", linear_position);
    ros::Duration delta_t = ros::Time::now() - last_update_;
    linear_position_ = linear_position*(max_linear_position_ - min_linear_position_)/4048 + min_linear_position_;
    linear_speed_ = (linear_position_ - last_linear_position_)/delta_t.toSec();

    int rotation_position;
    if(!nanotec.getPosition(rotation_position))
    {
        ROS_WARN("FSR Husky Arm - %s - Failed to update the position of the rotation actuator!", __FUNCTION__);
        return;
    }
    delta_t = ros::Time::now() - last_update_;
    angular_position_ = rotation_position*(max_angular_position_ - min_angular_position_)/max_position_ + min_angular_position_;
    angular_speed_ = (angular_position_ - last_rotation_position_)/delta_t.toSec();

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
    joint_state.position[0] = angular_position_;
    joint_state.velocity[0] = angular_speed_;
    joint_state.name[1] = "upper_arm_joint";
    joint_state.position[1] = linear_position_;
    joint_state.velocity[1] = linear_speed_;
    joint_state.name[2] = "lower_arm_joint";
    joint_state.position[2] = linear_position_;
    joint_state.velocity[2] = linear_speed_;
    joint_state.name[3] = "metal_detector_arm_joint";
    joint_state.position[3] = -linear_position_;
    joint_state.velocity[3] = linear_speed_;
    m_joint_pub.publish(joint_state);

    last_update_ = now;
    last_linear_position_ = linear_position_;
    last_rotation_position_ = angular_position_;
}

void FSRHuskyArm::asHomeCallback(const fsr_husky_driver::HomeGoalConstPtr &req)
{
    if(home())
    {
         m_as_home_.setSucceeded();
    }
    else
    {
        m_as_home_.setAborted();
    }
}

void FSRHuskyArm::asGotoCallback(const fsr_husky_driver::GotoGoalConstPtr &goal)
{
    ros::Rate loop_rate(10.0);

    ros::Time start_time = ros::Time::now();

    double peed_l = goal->joint.velocity[0];
    double l = goal->joint.position[0];
    if(l < min_linear_position_) l = min_linear_position_;
    if(l > max_linear_position_) l = max_linear_position_;
    int goal_l = (l - min_linear_position_)*4048/(max_linear_position_ - min_linear_position_);

    double speed_a = goal->joint.velocity[1];
    double a = goal->joint.position[1];
    if(a < min_angular_position_) a = min_angular_position_;
    if(a > max_angular_position_) a = max_angular_position_;
    int goal_a = (a - min_angular_position_)*max_position_/(max_angular_position_ - min_angular_position_);

    jrk.setPosition(goal_l);
    nanotec.setPosition(goal_a);

    // start executing the action
    while(ros::ok())
    {
        // check that preempt has not been requested by the client
        if(m_as_goto_.isPreemptRequested())
        {
            ROS_INFO("FSR Husky Arm - %s - Goal preempted", __FUNCTION__);
            // set the action state to preempted
            m_as_goto_.setPreempted();
            break;
        }

        // Read Position & Speed
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();
        joint_state.header.frame_id = "fsr_husky_arm";
        joint_state.name.resize(2);
        joint_state.position.resize(2);
        joint_state.velocity.resize(2);
        joint_state.name[0] = linear_joint_;
        joint_state.position[0] = linear_position_;
        joint_state.velocity[0] = linear_speed_;
        joint_state.name[1] = rotation_joint_;
        joint_state.position[1] = angular_position_;
        joint_state.velocity[1] = angular_speed_;

        if(ros::Time::now() - start_time > timeout_)
        {
            fsr_husky_driver::GotoResult result;
            result.joint = joint_state;
            ROS_INFO("FSR Husky Arm - %s - Goal timeout", __FUNCTION__);
            // set the action state to failed
            m_as_goto_.setAborted();
            break;
        }

        //ROS_INFO("[DEBUG] goal:%lf position:%lf tolerance:%lf", goal->joint.position[1], angular_position_, goal_tolerance_);
        //ROS_INFO("[DEBUG] goal:%lf position:%lf tolerance:%lf", goal->joint.position[0], linear_position_, goal_tolerance_);
        if( fabs(goal->joint.position[0] - linear_position_) <= goal_tolerance_*4.0  && fabs(goal->joint.position[1] - angular_position_) <= goal_tolerance_ )
        {
            fsr_husky_driver::GotoResult result;
            result.joint = joint_state;
            // set the action state to succeeded
            m_as_goto_.setSucceeded(result);
            break;
        }

        fsr_husky_driver::GotoFeedback feedback;
        feedback.joint = joint_state;
        // publish the feedback
        m_as_goto_.publishFeedback(feedback);

        loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fsr_husky_arm");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    ROS_INFO("FSR Husky Arm -- ROS Hardware Driver");

    FSRHuskyArm arm(n, pn);
    arm.init();

    double rate;
    pn.param("rate", rate, 20.0);
    ros::Rate r(rate);

    while(ros::ok())
    {
        arm.update();
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
