
#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <actionlib/server/simple_action_server.h>
#include <fsr_husky_driver/HomeAction.h>
#include <controller_manager/controller_manager.h>

#include "jrk_driver.h"
#include "nanotec_driver.h"

class FSRHuskyArm : public hardware_interface::RobotHW
{
    public:
        FSRHuskyArm(ros::NodeHandle &nh);

        void init(ros::NodeHandle &pnh);
        bool home(double timeout=120.0);
        void write();
        void read();

        actionlib::SimpleActionServer<fsr_husky_driver::HomeAction> m_as_home_;
        void asHomeCallback(const fsr_husky_driver::HomeGoalConstPtr &req);

    private:
        hardware_interface::JointStateInterface jnt_state_interface;
        hardware_interface::PositionJointInterface jnt_pos_interface;
        //hardware_interface::VelocityJointInterface jnt_vel_interface;
        double cmd_pos[2];
        //double cmd_vel[2];
        double pos[2];
        double vel[2];
        double eff[2];

        JRK jrk;
        Nanotec nanotec;

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

        bool homing_complete_;
};

FSRHuskyArm::FSRHuskyArm(ros::NodeHandle &nh) : m_as_home_(nh, "/arm/home", boost::bind(&FSRHuskyArm::asHomeCallback, this, _1), false)
{
    homing_complete_ = false;

    // connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle_a("arm_axel_joint", &pos[1], &vel[1], &eff[1]);
    jnt_state_interface.registerHandle(state_handle_a);

    hardware_interface::JointStateHandle state_handle_l("upper_arm_joint", &pos[0], &vel[0], &eff[0]);
    jnt_state_interface.registerHandle(state_handle_l);

    registerInterface(&jnt_state_interface);

    // connect and register the joint position interface
    hardware_interface::JointHandle pos_handle_a(jnt_state_interface.getHandle("arm_axel_joint"), &cmd_pos[1]);
    jnt_pos_interface.registerHandle(pos_handle_a);

    hardware_interface::JointHandle pos_handle_l(jnt_state_interface.getHandle("upper_arm_joint"), &cmd_pos[0]);
    jnt_pos_interface.registerHandle(pos_handle_l);

    registerInterface(&jnt_pos_interface);

    // connect and register the joint velocity interface
    /*hardware_interface::JointHandle vel_handle_pan(jnt_state_interface.getHandle("arm_axel_joint"), &cmd_vel[0]);
    jnt_vel_interface.registerHandle(vel_handle_pan);

    hardware_interface::JointHandle vel_handle_tilt(jnt_state_interface.getHandle("upper_arm_joint"), &cmd_vel[1]);
    jnt_vel_interface.registerHandle(vel_handle_tilt);

    registerInterface(&jnt_vel_interface);*/
}

void FSRHuskyArm::init(ros::NodeHandle &pnh)
{
    // Query for serial configuration
    std::string linear_port;
    pnh.param<std::string>("linear_actuator_port", linear_port, "/dev/serial/by-id/usb-Pololu_Corporation_Pololu_Jrk_21v3_Motor_Controller_00042539-if00");
    int linear_baudrate;
    pnh.param("linear_actuator_baudrate", linear_baudrate, 115200);

    pnh.param("min_linear_position", min_linear_position_, -0.5236);
    pnh.param("max_linear_position", max_linear_position_, 0.5236);

    std::string rotation_port;
    pnh.param<std::string>("rotation_actuator_port", rotation_port, "/dev/serial/by-id/usb-Pololu_Corporation_Pololu_Jrk_21v3_Motor_Controller_00042539-if02");
    int rotation_baudrate;
    pnh.param("rotation_actuator_baudrate", rotation_baudrate, 115200);

    pnh.param("min_angular_position", min_angular_position_, -0.8);
    pnh.param("max_angular_position", max_angular_position_, 0.8);

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
    pnh.param("home", go_home, false);
    if(go_home)
    {
        if(!home())
        {
            ROS_FATAL("FSR Husky Arm - %s - Failed to perform the homing routine!!", __FUNCTION__);
            ROS_BREAK();
        }
    }

    m_as_home_.start();

    if(!nanotec.setSpeeds(200, 400))
    {
        ROS_FATAL("FSR Husky Arm - %s - Failed to set the rotation actuator speeds!", __FUNCTION__);
        ROS_BREAK();
    }
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
        ROS_ERROR("FSR Husky Arm - %s - Timeout before the arm could reach the first limit switch", __FUNCTION__);
        return false;
    }

    ros::Duration(3.0).sleep();

    if(!nanotec.setPositionMode())
    {
        ROS_ERROR("FSR Husky Arm - %s - Failed to switch to absolute position mode!", __FUNCTION__);
        return false;
    }

    if(!nanotec.setPosition(5000))
    {
        ROS_ERROR("FSR Husky Arm - %s - Failed to set the possition to search for the other limit switch!", __FUNCTION__);
        return false;
    }
    ros::Duration(1.0).sleep();
    if(!nanotec.setPosition(5000))
    {
        ROS_ERROR("FSR Husky Arm - %s - Failed to set the possition to search for the other limit switch again!", __FUNCTION__);
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
        ROS_ERROR("FSR Husky Arm - %s - Timeout before the arm could reach the second limit switch", __FUNCTION__);
        return false;
    }

    ros::Duration(3.0).sleep();

    if(!nanotec.clearPositionError())
    {
        ROS_ERROR("FSR Husky Arm - %s - Failed to clear the position error!", __FUNCTION__);
        return false;
    }

    if(!nanotec.getPosition(max_position_))
    {
        ROS_ERROR("FSR Husky Arm - %s - Failed to get the position!", __FUNCTION__);
        return false;
    }
    ROS_INFO("FSR Husky Arm - %s - Limit reached at %d steps.", __FUNCTION__, max_position_);

    if(!nanotec.setPosition(max_position_/2))
    {
        ROS_ERROR("FSR Husky Arm - %s - Failed to complete the homing routine!", __FUNCTION__);
        return false;
    }

    ROS_INFO("FSR Husky Arm - %s - Homing complete!", __FUNCTION__);

    homing_complete_ = true;
    return true;
}

void FSRHuskyArm::write()
{
    if(!homing_complete_) return;

    double l = cmd_pos[0];
    if(l < min_linear_position_) l = min_linear_position_;
    if(l > max_linear_position_) l = max_linear_position_;
    int goal_l = (l - min_linear_position_)*4048/(max_linear_position_ - min_linear_position_);

    double a = cmd_pos[1];
    if(a < min_angular_position_) a = min_angular_position_;
    if(a > max_angular_position_) a = max_angular_position_;
    int goal_a = (a - min_angular_position_)*max_position_/(max_angular_position_ - min_angular_position_);

    jrk.setPosition(goal_l);
    nanotec.setPosition(goal_a);
}

void FSRHuskyArm::read()
{
    if(!homing_complete_) return;

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

    pos[1] = angular_position_;
    vel[1] = angular_speed_;
    eff[1] = 0.0;

    pos[0] = linear_position_;
    vel[0] = linear_speed_;
    eff[0] = 0.0;

    last_update_ = ros::Time::now();
    last_linear_position_ = linear_position_;
    last_rotation_position_ = angular_position_;
}

void FSRHuskyArm::asHomeCallback(const fsr_husky_driver::HomeGoalConstPtr &req)
{
    ROS_INFO("FSR Husky Arm - %s - Got a request to perform homing.", __FUNCTION__);

    if(home())
    {
         m_as_home_.setSucceeded();
    }
    else
    {
        m_as_home_.setAborted();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fsr_husky_arm");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    ROS_INFO("FSR Husky Arm -- ROS Hardware Driver");

    FSRHuskyArm arm(n);
    controller_manager::ControllerManager cm(&arm);

    arm.init(pn);

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::Rate r(20.0);
    while(ros::ok())
    {
        arm.read();
        cm.update(ros::Time::now(), ros::Duration(1/20.0));
        arm.write();
        r.sleep();
    }

    spinner.stop();

    return 0;
}
