
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <fsr_husky_driver/HomeAction.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <sensor_msgs/JointState.h>

#include "jrk_driver.h"
#include "nanotec_driver/NanotecMotor.h"

//prado - para controlar em separado o jrk, a qq momento
#include <sensor_msgs/Joy.h>
#include "std_msgs/Float64.h"





class FSRHuskyArm
{
    public:
        FSRHuskyArm(ros::NodeHandle &nh);

        void init(ros::NodeHandle &pnh);
        void spinOnce();

        void jointStateSpinner();
        void stop_Motor();

    private:
        ros::Publisher  m_joint_state_pub_;
        ros::Publisher  m_joint_pub_;
        ros::Subscriber m_joint_sub_;

        void setGoal(const trajectory_msgs::JointTrajectory::ConstPtr& msg);

        actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> m_as_goal_;
        bool as_active_;
        void actionServerCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);

        bool home(double timeout=120.0);
        actionlib::SimpleActionServer<fsr_husky_driver::HomeAction> m_as_home_;
        void asHomeCallback(const fsr_husky_driver::HomeGoalConstPtr &req);
        bool homing_complete_;

        bool checkJointNames(const std::vector<std::string> *joint_names);
        bool checkGoal(const std::vector<trajectory_msgs::JointTrajectoryPoint> *points);

        void write(double lift, double lift_speed, double sweep, double sweep_speed, double sweep_acceleration);

        //prado - para controlar em separado o jrk, a qq momento
        ros::Subscriber joy_sub_2;
        ros::Subscriber lift_sub_;

        double hoffset;
        double increment;
        double height;

        void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
        void liftCallback(const std_msgs::Float64::ConstPtr& lift);

        std::string lift_joint_;
        std::string sweep_joint_;

        ros::Time start_time_;
        ros::Time goal_start_time_;
        std::list<trajectory_msgs::JointTrajectoryPoint> trajectory_;
        std::vector<control_msgs::JointTolerance> path_tolerance_;
        std::vector<control_msgs::JointTolerance> goal_tolerance_;
        ros::Duration goal_time_tolerance_;

        JRK jrk;
        nanotec::NanotecPort port;
        nanotec::NanotecMotor motor;

        ros::Time last_update_;
        double last_lift_;
        double last_sweep_;

        double lift_;
        double sweep_;
        double lift_speed_;
        double sweep_speed_;
        double min_lift_;
        double max_lift_;
        double max_lift_speed_;
        double min_sweep_;
        double max_sweep_;
        double max_sweep_speed_;
        double lift_tolerance_;
        double sweep_tolerance_;
        ros::Duration time_tolerance_;

        int lift_index_;
        int sweep_index_;

        int max_sweep_steps_;

        double joint_state_rate_;

        bool got_new_goal_;

//        double acceleration_coeficient_;
        double speed_coeficient_;
};

FSRHuskyArm::FSRHuskyArm(ros::NodeHandle &nh) :
    port(),
    motor(&port),
    m_as_home_(nh, "/arm/home", boost::bind(&FSRHuskyArm::asHomeCallback, this, _1), false),
    m_as_goal_(nh, "/arm_controller/follow_joint_trajectory", boost::bind(&FSRHuskyArm::actionServerCallback, this, _1), false)

{
    // Publishers : Only publish the most recent reading
    m_joint_pub_ = nh.advertise<control_msgs::JointTrajectoryControllerState>("/arm_controller/state", 1);

    // Subscribers : Only subscribe to the most recent instructions
    m_joint_sub_ = nh.subscribe("/arm_controller/command", 1, &FSRHuskyArm::setGoal, this);

    m_joint_state_pub_ = nh.advertise<sensor_msgs::JointState>("/joint_states", 200);

    //prado - para controlar em separado o jrk, a qq momento
    joy_sub_2 = nh.subscribe<sensor_msgs::Joy>("joy", 10, &FSRHuskyArm::joyCallback,this);
    lift_sub_ = nh.subscribe<std_msgs::Float64>("lift_control", 1, &FSRHuskyArm::liftCallback,this);

    homing_complete_ = false;
    got_new_goal_ = false;
}


//prado ****
void FSRHuskyArm::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

    bool flag_joy=false;
    int actual_goal;
    jrk.getPosition(actual_goal);
    hoffset=-(actual_goal*(max_lift_ - min_lift_)/4048+min_lift_);

    if(joy->buttons[0] == 1) //A pressed new
    {
        hoffset += increment; //meio centimetro
        ROS_INFO("A pressed new - go UP");
        flag_joy=true;
    }else if(joy->buttons[2] == 1) //X pressed new
    {
        hoffset -= increment;
        ROS_INFO("X pressed new - go DOWN");
        flag_joy=true;
    }
    else if(joy->buttons[3] == 1) //Y pressed new
    {
        if(hoffset<-height)
            hoffset = -height;
        else
            hoffset += increment; //meio centimetro
        ROS_INFO("Y pressed new - EMERGENCY UP");
        flag_joy=true;
    }

    if(flag_joy){
        double l = -1.0*hoffset;
        if(l < min_lift_) l = min_lift_;
        if(l > max_lift_) l = max_lift_;
        int goal_l = (l - min_lift_)*4048/(max_lift_ - min_lift_);

        jrk.setPosition(goal_l);
    }

}

void FSRHuskyArm::liftCallback(const std_msgs::Float64::ConstPtr& lift)
{
    double l = -1.0*lift->data;
    if(l < min_lift_) l = min_lift_;
    if(l > max_lift_) l = max_lift_;
    int goal_l = (l - min_lift_)*4048/(max_lift_ - min_lift_);

    jrk.setPosition(goal_l);
}

//************


void FSRHuskyArm::init(ros::NodeHandle &pnh)
{
    // Query for serial configuration
    std::string linear_port;
    pnh.param<std::string>("linear_actuator_port", linear_port, "/dev/serial/by-id/usb-Pololu_Corporation_Pololu_Jrk_21v3_Motor_Controller_00076021-if00");
    int linear_baudrate;
    pnh.param("linear_actuator_baudrate", linear_baudrate, 115200);

    std::string rotation_port;
    pnh.param<std::string>("rotation_actuator_port", rotation_port, "/dev/serial/by-id/usb-Pololu_Corporation_Pololu_Jrk_21v3_Motor_Controller_00076021-if02");
    int rotation_baudrate;
    pnh.param("rotation_actuator_baudrate", rotation_baudrate, 115200);

    if(!jrk.init(&linear_port, linear_baudrate))
    {
        ROS_FATAL("FSR Husky Arm - %s - Failed to initialize the linear actuator!", __FUNCTION__);
        ROS_BREAK();
    }
    if(!port.openPort(rotation_port, rotation_baudrate))
    {
        ROS_FATAL("FSR Husky Arm - %s - Failed to initialize the rotation actuator!", __FUNCTION__);
        ROS_BREAK();
    }

    pnh.param<std::string>("lift_joint", lift_joint_, "upper_arm_joint");
    pnh.param<std::string>("sweep_joint", sweep_joint_, "arm_axel_joint");

    pnh.param("min_lift", min_lift_, -0.25);
    pnh.param("max_lift", max_lift_, 0.25);
    pnh.param("max_lift_speed", max_lift_speed_, 1.0);

    pnh.param("min_sweep", min_sweep_, -1.5);
    pnh.param("max_sweep", max_sweep_, 1.5);
    pnh.param("max_sweep_speed", max_sweep_speed_, 1.0);

    pnh.param("lift_tolerance", lift_tolerance_, 0.05);
    pnh.param("sweep_tolerance", sweep_tolerance_, 0.05);

    //prado - para controlar em separado o jrk, a qq momento
    pnh.param("increment", increment, 0.01);
    pnh.param("height", height, -0.20);

    double time_tolerance;
    pnh.param("time_tolerance", time_tolerance, 0.5);
    time_tolerance_ = ros::Duration(time_tolerance);

    pnh.param("joint_state_rate", joint_state_rate_, 50.0);

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

    m_as_goal_.start();
    as_active_ = false;

    try{ motor.setMinimumFrequency(200); }
    catch(nanotec::Exception& e)
    {
        ROS_FATAL("FSR Husky Arm - %s - Failed to set the rotation actuator speeds!", __FUNCTION__);
        ROS_BREAK();
    }

    try{ motor.setMaximumFrequency(400); }
    catch(nanotec::Exception& e)
    {
        ROS_FATAL("FSR Husky Arm - %s - Failed to set the rotation actuator speeds!", __FUNCTION__);
        ROS_BREAK();
    }

    try{ motor.setRampType(0); }
    catch(nanotec::Exception& e)
    {
        ROS_FATAL("FSR Husky Arm - %s - Failed to set the ramp type!", __FUNCTION__);
        ROS_BREAK();
    }

    //    try{ motor.setMaximumJerk(2); }
    //    catch(nanotec::Exception& e)
    //    {
    //        ROS_FATAL("FSR Husky Arm - %s - Failed to set the ramp type!", __FUNCTION__);
    //        ROS_BREAK();
    //    }


    speed_coeficient_=motor.getStepMode()*15*1.67;
//    acceleration_coeficient_=speed_coeficient_;//1.0*max_sweep_steps_*15*1.67/3.14/3;

    ROS_INFO("speed_coeficient:%lf - get:%d",speed_coeficient_,motor.getStepMode());

}

bool FSRHuskyArm::home(double timeout)
{
    ROS_INFO("FSR Husky Arm - %s - Starting homing...", __FUNCTION__);

    jrk.setPosition(4048/2);

    try{ motor.setDirection(NANOTEC_DIRECTION_LEFT); }
    catch(nanotec::Exception& e)
    {
        ROS_ERROR("FSR Husky Arm - %s - Failed to change the direction!", __FUNCTION__);
        return false;
    }

    try{ motor.setPositionMode(NANOTEC_POSITION_MODE_EXTERNAL_REF_RUN); }
    catch(nanotec::Exception& e)
    {
        ROS_ERROR("FSR Husky Arm - %s - Failed to start the homing routine!", __FUNCTION__);
        return false;
    }
    try{ motor.startMotor(); }
    catch(nanotec::Exception& e)
    {
        ROS_ERROR("FSR Husky Arm - %s - Failed to start the homing routine!", __FUNCTION__);
        return false;
    }

    nanotec::NanotecStatus motor_status;

    ros::Rate r(2.0);
    ros::Time start_time = ros::Time::now();
    try{ motor.getStatus(motor_status); }
    catch(nanotec::Exception& e)
    {
        ROS_WARN("FSR Husky Arm - %s - Failed to get motor status!", __FUNCTION__);
    }
    while(!motor_status.zero_position_reached && ros::Time::now() - start_time < ros::Duration(timeout) && ros::ok())
    {
        r.sleep();
        try{ motor.getStatus(motor_status);}
        catch(nanotec::Exception& e)
        {
            ROS_WARN("FSR Husky Arm - %s - Failed to get motor status!", __FUNCTION__);
        }
    }
    if(!motor_status.zero_position_reached)
    {
        ROS_ERROR("FSR Husky Arm - %s - Timeout before the arm could reach the first limit switch", __FUNCTION__);
        return false;
    }

    ros::Duration(3.0).sleep();

    try{ motor.setPositionMode(NANOTEC_POSITION_MODE_ABSOLUTE); }
    catch(nanotec::Exception& e)
    {
        ROS_ERROR("FSR Husky Arm - %s - Failed to switch to absolute position mode!", __FUNCTION__);
        return false;
    }
    try{ motor.setTravelDistance(5000); }
    catch(nanotec::Exception& e)
    {
        ROS_ERROR("FSR Husky Arm - %s - Failed to set the possition to search for the other limit switch!", __FUNCTION__);
        return false;
    }
    try{ motor.startMotor(); }
    catch(nanotec::Exception& e)
    {
        ROS_ERROR("FSR Husky Arm - %s - Failed to set the possition to search for the other limit switch!", __FUNCTION__);
        return false;
    }

    start_time = ros::Time::now();
    try{ motor.getStatus(motor_status);}
    catch(nanotec::Exception& e)
    {
        ROS_ERROR("FSR Husky Arm - %s - Failed to get motor status!", __FUNCTION__);
        return false;
    }
    while(!motor_status.controller_ready && ros::Time::now() - start_time < ros::Duration(timeout) && ros::ok())
    {
        r.sleep();
        try{ motor.getStatus(motor_status);}
        catch(nanotec::Exception& e)
        {
            ROS_ERROR("FSR Husky Arm - %s - Failed to get motor status!", __FUNCTION__);
            return false;
        }
    }
    if(!motor_status.controller_ready)
    {
        ROS_ERROR("FSR Husky Arm - %s - Timeout before the arm was ready to move!", __FUNCTION__);
        return false;
    }

    try{ motor.startMotor(); }
    catch(nanotec::Exception& e)
    {
        ROS_ERROR("FSR Husky Arm - %s - Failed to set the possition to search for the other limit switch!", __FUNCTION__);
        return false;
    }

    start_time = ros::Time::now();
    try{ motor.getStatus(motor_status);}
    catch(nanotec::Exception& e)
    {
        ROS_ERROR("FSR Husky Arm - %s - Failed to get motor status!", __FUNCTION__);
        return false;
    }
    while(!motor_status.position_error && ros::Time::now() - start_time < ros::Duration(timeout) && ros::ok())
    {
        r.sleep();
        try{ motor.getStatus(motor_status);}
        catch(nanotec::Exception& e)
        {
            ROS_ERROR("FSR Husky Arm - %s - Failed to get motor status!", __FUNCTION__);
            return false;
        }
    }
    if(!motor_status.position_error)
    {
        ROS_ERROR("FSR Husky Arm - %s - Timeout before the arm could reach the second limit switch", __FUNCTION__);
        return false;
    }

    ros::Duration(3.0).sleep();

    try{ motor.resetPositionError(); }
    catch(nanotec::Exception& e)
    {
        ROS_ERROR("FSR Husky Arm - %s - Failed to clear the position error!", __FUNCTION__);
        return false;
    }

    try{ max_sweep_steps_ = motor.getPosition(); }
    catch(nanotec::Exception& e)
    {
        ROS_ERROR("FSR Husky Arm - %s - Failed to get the position!", __FUNCTION__);
        return false;
    }
    ROS_INFO("FSR Husky Arm - %s - Limit reached at %d steps.", __FUNCTION__, max_sweep_steps_);

    try{ motor.setTravelDistance(max_sweep_steps_/2); }
    catch(nanotec::Exception& e)
    {
        ROS_ERROR("FSR Husky Arm - %s - Failed to complete the homing routine!", __FUNCTION__);
        return false;
    }
    try{ motor.startMotor(); }
    catch(nanotec::Exception& e)
    {
        ROS_ERROR("FSR Husky Arm - %s - Failed to complete the homing routine!", __FUNCTION__);
        return false;
    }

    ROS_INFO("FSR Husky Arm - %s - Homing complete!", __FUNCTION__);

    homing_complete_ = true;
    return true;
}

void FSRHuskyArm::write(double lift, double lift_speed, double sweep, double sweep_speed, double sweep_acceleration)
{
    if(!homing_complete_) return;

    double l = -1.0*lift;
    if(l < min_lift_) l = min_lift_;
    if(l > max_lift_) l = max_lift_;
    int goal_l = (l - min_lift_)*4048/(max_lift_ - min_lift_);

    double a = sweep;
    if(a < min_sweep_) a = min_sweep_;
    if(a > max_sweep_) a = max_sweep_;
    int goal_a = (a - min_sweep_)*max_sweep_steps_/(max_sweep_ - min_sweep_);

    //ROS_INFO("FSR Husky Arm - %s - Goal lift %lf %d sweep %lf %d", __FUNCTION__, lift, goal_l, sweep, goal_a);

    jrk.setPosition(goal_l);

    try{ motor.setRampType(2); }
    catch(nanotec::Exception& e)
    {
        ROS_FATAL("FSR Husky Arm - %s - Failed to set the ramp type!", __FUNCTION__);
        ROS_BREAK();
    }

        try{ motor.setMaximumJerk(10000000); }
        catch(nanotec::Exception& e)
        {
            ROS_FATAL("FSR Husky Arm - %s - Failed to set the ramp type!", __FUNCTION__);
            ROS_BREAK();
        }

//    try{ motor.setAccelerationRamp((int)(acceleration_coeficient_*sweep_acceleration)); }
    try{ motor.setAccelerationRamp2((int)(pow(3000/(11.7+sweep_acceleration),2))); }
        catch(nanotec::Exception& e)
    {
        ROS_ERROR("FSR Husky Arm - %s - Failed to set the acceleration!", __FUNCTION__);
        return;
    }
    try{ motor.setMaximumFrequency((int)(speed_coeficient_*sweep_speed)); }
    catch(nanotec::Exception& e)
    {
        ROS_ERROR("FSR Husky Arm - %s - Failed to set velocity!", __FUNCTION__);
        return;
    }
    try{ motor.setTravelDistance(goal_a); }
    catch(nanotec::Exception& e)
    {
        ROS_ERROR("FSR Husky Arm - %s - Failed to set new goal!", __FUNCTION__);
        return;
    }
    try{ motor.startMotor(); }
    catch(nanotec::Exception& e)
    {
        ROS_ERROR("FSR Husky Arm - %s - Failed to start the motor!", __FUNCTION__);
        return;
    }
}

bool FSRHuskyArm::checkJointNames(const std::vector<std::string> *joint_names)
{
    int found_joint = 0;
    for(int i=0 ; i<joint_names->size() ; i++)
    {
        if(lift_joint_.compare(joint_names->at(i)) == 0)
        {
            lift_index_ = i;
            found_joint++;
        }
        else if(sweep_joint_.compare(joint_names->at(i)) == 0)
        {
            sweep_index_ = i;
            found_joint++;
        }
    }
    return (joint_names->size() == found_joint);
}

bool FSRHuskyArm::checkGoal(const std::vector<trajectory_msgs::JointTrajectoryPoint> *points)
{
    for(int i=0 ; i<points->size() ; i++)
    {
        if(     points->at(i).positions[lift_index_] < min_lift_ ||
                points->at(i).positions[lift_index_] > max_lift_ ||
                points->at(i).positions[sweep_index_] < min_sweep_ ||
                points->at(i).positions[sweep_index_] > max_sweep_ ||
                points->at(i).velocities[lift_index_] > max_lift_speed_ ||
                points->at(i).velocities[sweep_index_] > max_sweep_speed_) return false;
    }
    return true;
}

void FSRHuskyArm::jointStateSpinner()
{
    ros::Rate r(joint_state_rate_);
    while(ros::ok())
    {
        sensor_msgs::JointState msg;

        msg.header.stamp = ros::Time::now();
        msg.name.push_back(lift_joint_);
        msg.position.push_back(lift_);
        msg.velocity.push_back(lift_speed_);
        msg.effort.push_back(0.0);
        msg.name.push_back(sweep_joint_);
        msg.position.push_back(sweep_);
        msg.velocity.push_back(sweep_speed_);
        msg.effort.push_back(0.0);
        msg.name.push_back("lower_arm_joint");
        msg.position.push_back(lift_);
        msg.velocity.push_back(lift_speed_);
        msg.effort.push_back(0.0);
        msg.name.push_back("metal_detector_arm_joint");
        msg.position.push_back(-1.0*lift_);
        msg.velocity.push_back(-1.0*lift_speed_);
        msg.effort.push_back(0.0);

        m_joint_state_pub_.publish(msg);
        r.sleep();
    }
}

void FSRHuskyArm::spinOnce()
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
    lift_ = -1.0*(linear_position*(max_lift_ - min_lift_)/4048 + min_lift_);
    lift_speed_ = (lift_ - last_lift_)/delta_t.toSec();

    int rotation_position;
    try{ rotation_position = motor.getPosition(); }
    catch(nanotec::Exception& e)
    {
        ROS_WARN("FSR Husky Arm - %s - Failed to update the position of the rotation actuator!", __FUNCTION__);
        return;
    }

    delta_t = ros::Time::now() - last_update_;
    sweep_ = rotation_position*(max_sweep_ - min_sweep_)/max_sweep_steps_ + min_sweep_;
    sweep_speed_ = (sweep_ - last_sweep_)/delta_t.toSec();

    // Publish Position & Speed
    control_msgs::JointTrajectoryControllerState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.joint_names.push_back(lift_joint_);
    joint_state.actual.positions.push_back(lift_);
    joint_state.actual.velocities.push_back(lift_speed_);
    joint_state.joint_names.push_back(sweep_joint_);
    joint_state.actual.positions.push_back(sweep_);
    joint_state.actual.velocities.push_back(sweep_speed_);
    if(trajectory_.size() > 0)
    {
        joint_state.desired.positions.push_back(trajectory_.front().positions[lift_index_]);
        joint_state.desired.velocities.push_back(trajectory_.front().velocities[lift_index_]);
        joint_state.error.positions.push_back(trajectory_.front().positions[lift_index_] - lift_);
        joint_state.error.velocities.push_back(trajectory_.front().velocities[lift_index_] - lift_speed_);
        joint_state.desired.positions.push_back(trajectory_.front().positions[sweep_index_]);
        joint_state.desired.velocities.push_back(trajectory_.front().velocities[sweep_index_]);
        joint_state.error.positions.push_back(trajectory_.front().positions[sweep_index_] - sweep_);
        joint_state.error.velocities.push_back(trajectory_.front().velocities[sweep_index_] - sweep_speed_);
        joint_state.desired.time_from_start = trajectory_.front().time_from_start;
        joint_state.actual.time_from_start = ros::Time::now() - start_time_;
        joint_state.error.time_from_start = ros::Time::now() - start_time_ - trajectory_.front().time_from_start;
    }
    m_joint_pub_.publish(joint_state);

    last_update_ = ros::Time::now();
    last_lift_ = lift_;
    last_sweep_ = sweep_;

    // If there is stuff on the queue deal with it
    if(trajectory_.size() > 0)
    {
        if(got_new_goal_)
        {
            write(trajectory_.front().positions[lift_index_], trajectory_.front().velocities[lift_index_], trajectory_.front().positions[sweep_index_], trajectory_.front().velocities[sweep_index_], trajectory_.front().accelerations[sweep_index_]);
            got_new_goal_ = false;
        }

        double lift_tolerance = lift_tolerance_;
        double sweep_tolerance = sweep_tolerance_;
        ros::Duration time_tolerance = time_tolerance_;

        if(as_active_)
        {
            if(trajectory_.size() == 1 && goal_tolerance_[lift_index_].position > 0 && goal_tolerance_[sweep_index_].position > 0)
            {
                lift_tolerance = goal_tolerance_[lift_index_].position;
                sweep_tolerance = goal_tolerance_[sweep_index_].position;
            }
            else if(path_tolerance_[lift_index_].position > 0 && path_tolerance_[sweep_index_].position > 0)
            {
                lift_tolerance = path_tolerance_[lift_index_].position;
                sweep_tolerance = path_tolerance_[sweep_index_].position;
            }

            if(goal_time_tolerance_ > time_tolerance) time_tolerance = goal_time_tolerance_;
        }

        if(fabs(trajectory_.front().positions[lift_index_] - lift_) <= lift_tolerance && fabs(trajectory_.front().positions[sweep_index_] - sweep_) <= sweep_tolerance)
        {
            // Remove it from the queue
            trajectory_.pop_front();
            if(as_active_ && trajectory_.size() == 0)
            {
                // And set the AS to successful!
                control_msgs::FollowJointTrajectoryResult result;
                result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
                m_as_goal_.setSucceeded(result);
                as_active_ = false;
            }
            else if(trajectory_.size() > 0)
            {
                // And keep on going!
                write(trajectory_.front().positions[lift_index_], trajectory_.front().velocities[lift_index_], trajectory_.front().positions[sweep_index_], trajectory_.front().velocities[sweep_index_], trajectory_.front().accelerations[sweep_index_]);
            }
        }
        // If we timed out...
        else if(as_active_ && ros::Time::now() > goal_start_time_ + goal_time_tolerance_)
        {
            // Clear the trajectory queue
            trajectory_.clear();
            // And abort the AS.
            control_msgs::FollowJointTrajectoryResult result;
            result.error_code = control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED;
            m_as_goal_.setAborted(result);
            as_active_ = false;
        }
        else if(ros::Time::now() > start_time_ + time_tolerance)
        {
            if(as_active_ && path_tolerance_[lift_index_].position != -1 && path_tolerance_[sweep_index_].position != -1)
            {
                trajectory_.clear();

                // And abort the AS.
                control_msgs::FollowJointTrajectoryResult result;
                result.error_code = control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
                m_as_goal_.setAborted(result);
                as_active_ = false;
            }
            else if(!as_active_)
            {
                trajectory_.clear();
            }
        }
    }
}

void FSRHuskyArm::setGoal(const trajectory_msgs::JointTrajectory::ConstPtr& msg)
{
    if(!homing_complete_ || !checkJointNames(&msg->joint_names) || !checkGoal(&msg->points))
        return;

    if(as_active_)
    {
        ROS_WARN("FSR Husky Arm - %s - Goal preempted.", __FUNCTION__);
        // set the action state to preempted
        m_as_goal_.setPreempted();
        as_active_ = false;
    }

    trajectory_.clear();
    for(int i=0 ; i<msg->points.size() ; i++)
    {
        trajectory_.push_back(msg->points[i]);
    }
    got_new_goal_ = true;
    //write(trajectory_.front().positions[lift_index_], trajectory_.front().velocities[lift_index_], trajectory_.front().positions[sweep_index_], trajectory_.front().velocities[sweep_index_]);
}

void FSRHuskyArm::actionServerCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
{
    if(!homing_complete_)
    {
        control_msgs::FollowJointTrajectoryResult result;
        result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
        m_as_goal_.setAborted(result);
        ROS_ERROR("FSR Husky Arm - %s - The homing routine was not performed yet!", __FUNCTION__);
        return;
    }

    if(!checkJointNames(&goal->trajectory.joint_names))
    {
        control_msgs::FollowJointTrajectoryResult result;
        result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
        m_as_goal_.setAborted(result);
        ROS_ERROR("FSR Husky Arm - %s - Invalid joints!", __FUNCTION__);
        return;
    }

    if(!checkGoal(&goal->trajectory.points))
    {
        control_msgs::FollowJointTrajectoryResult result;
        result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
        m_as_goal_.setAborted(result);
        ROS_ERROR("FSR Husky Arm - %s - Invalid goal!", __FUNCTION__);
        return;
    }

    as_active_ = true;

    trajectory_.clear();
    for(int i=0 ; i<goal->trajectory.points.size() ; i++)
    {
        trajectory_.push_back(goal->trajectory.points[i]);
    }

    path_tolerance_ = goal->path_tolerance;
    goal_tolerance_ = goal->goal_tolerance;
    goal_time_tolerance_ = goal->goal_time_tolerance;
    got_new_goal_ = true;
    //write(trajectory_.front().positions[lift_index_], trajectory_.front().velocities[lift_index_], trajectory_.front().positions[sweep_index_], trajectory_.front().velocities[sweep_index_]);

    goal_start_time_ = ros::Time::now();

    ros::Rate r(10.0);
    // start executing the action
    while(ros::ok() && as_active_)
    {
        // check that preempt has not been requested by the client
        if(m_as_goal_.isPreemptRequested())
        {
            ROS_WARN("FSR Husky Arm - %s - Goal preempted.", __FUNCTION__);
            // set the action state to preempted
            m_as_goal_.setPreempted();
            as_active_ = false;
            break;
        }

        // Read Position & Speed
        control_msgs::FollowJointTrajectoryFeedback feedback;
        feedback.header.stamp = ros::Time::now();
        feedback.joint_names.push_back(lift_joint_);
        feedback.desired.positions.push_back(trajectory_.front().positions[lift_index_]);
        feedback.desired.velocities.push_back(trajectory_.front().velocities[lift_index_]);
        feedback.actual.positions.push_back(lift_);
        feedback.actual.velocities.push_back(lift_speed_);
        feedback.error.positions.push_back(trajectory_.front().positions[lift_index_] - lift_);
        feedback.error.velocities.push_back(trajectory_.front().velocities[lift_index_] - lift_speed_);
        feedback.joint_names.push_back(sweep_joint_);
        feedback.desired.positions.push_back(trajectory_.front().positions[sweep_index_]);
        feedback.desired.velocities.push_back(trajectory_.front().velocities[sweep_index_]);
        feedback.actual.positions.push_back(sweep_);
        feedback.actual.velocities.push_back(sweep_speed_);
        feedback.error.positions.push_back(trajectory_.front().positions[sweep_index_] - sweep_);
        feedback.error.velocities.push_back(trajectory_.front().velocities[sweep_index_] - sweep_speed_);
        feedback.desired.time_from_start = trajectory_.front().time_from_start;
        feedback.actual.time_from_start = ros::Time::now() - start_time_;
        feedback.error.time_from_start = ros::Time::now() - start_time_ - trajectory_.front().time_from_start;
        // publish the feedback
        m_as_goal_.publishFeedback(feedback);

        r.sleep();
    }

    as_active_ = false;
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


void FSRHuskyArm::stop_Motor()
{
    try{ FSRHuskyArm::motor.stopMotor(false); }
    catch(nanotec::Exception& e)
    {
        ROS_ERROR("FSR Husky Arm - %s - Failed to stop motor!", __FUNCTION__);
    }
    ROS_INFO("FSR Husky Arm - Motor stopped!");

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fsr_husky_arm");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    ROS_INFO("FSR Husky Arm v2.0 -- ROS Hardware Driver");

    FSRHuskyArm arm(n);
    arm.init(pn);

    ros::AsyncSpinner spinner(5);
    spinner.start();

    boost::thread joint_state_thread(&FSRHuskyArm::jointStateSpinner, &arm);

    ros::Rate r(10.0);
    while(ros::ok())
    {
        arm.spinOnce();
        r.sleep();
    }

    arm.stop_Motor();

    joint_state_thread.join();

    spinner.stop();

    return 0;
}
