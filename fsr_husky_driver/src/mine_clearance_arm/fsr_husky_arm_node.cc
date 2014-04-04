
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <fsr_husky_driver/HomeAction.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <sensor_msgs/JointState.h>

#include "jrk_driver.h"
#include "nanotec_driver.h"

class FSRHuskyArm
{
    public:
        FSRHuskyArm(ros::NodeHandle &nh);

        void init(ros::NodeHandle &pnh);
        void spinOnce();

        void jointStateSpinner();

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

        void write(double lift, double lift_speed, double sweep, double sweep_speed);

        std::string lift_joint_;
        std::string sweep_joint_;

        ros::Time start_time_;
        ros::Time goal_start_time_;
        std::list<trajectory_msgs::JointTrajectoryPoint> trajectory_;
        std::vector<control_msgs::JointTolerance> path_tolerance_;
        std::vector<control_msgs::JointTolerance> goal_tolerance_;
        ros::Duration goal_time_tolerance_;

        JRK jrk;
        Nanotec nanotec;

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
};

FSRHuskyArm::FSRHuskyArm(ros::NodeHandle &nh) :
    m_as_home_(nh, "/arm/home", boost::bind(&FSRHuskyArm::asHomeCallback, this, _1), false),
    m_as_goal_(nh, "/arm_controller/follow_joint_trajectory", boost::bind(&FSRHuskyArm::actionServerCallback, this, _1), false)

{
    // Publishers : Only publish the most recent reading
    m_joint_pub_ = nh.advertise<control_msgs::JointTrajectoryControllerState>("/arm_controller/state", 1);

    // Subscribers : Only subscribe to the most recent instructions
    m_joint_sub_ = nh.subscribe("/arm_controller/command", 1, &FSRHuskyArm::setGoal, this);

    m_joint_state_pub_ = nh.advertise<sensor_msgs::JointState>("/joint_states", 200);

    homing_complete_ = false;
}

void FSRHuskyArm::init(ros::NodeHandle &pnh)
{
    // Query for serial configuration
    std::string linear_port;
    pnh.param<std::string>("linear_actuator_port", linear_port, "/dev/serial/by-id/usb-Pololu_Corporation_Pololu_Jrk_21v3_Motor_Controller_00042539-if00");
    int linear_baudrate;
    pnh.param("linear_actuator_baudrate", linear_baudrate, 115200);

    std::string rotation_port;
    pnh.param<std::string>("rotation_actuator_port", rotation_port, "/dev/serial/by-id/usb-Pololu_Corporation_Pololu_Jrk_21v3_Motor_Controller_00042539-if02");
    int rotation_baudrate;
    pnh.param("rotation_actuator_baudrate", rotation_baudrate, 115200);

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

    double time_tolerance;
    pnh.param("time_tolerance", time_tolerance, 0.5);
    time_tolerance_ = ros::Duration(time_tolerance);

    pnh.param("joint_state_rate", joint_state_rate_, 100.0);

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

    if(!nanotec.getPosition(max_sweep_steps_))
    {
        ROS_ERROR("FSR Husky Arm - %s - Failed to get the position!", __FUNCTION__);
        return false;
    }
    ROS_INFO("FSR Husky Arm - %s - Limit reached at %d steps.", __FUNCTION__, max_sweep_steps_);

    if(!nanotec.setPosition(max_sweep_steps_/2))
    {
        ROS_ERROR("FSR Husky Arm - %s - Failed to complete the homing routine!", __FUNCTION__);
        return false;
    }

    ROS_INFO("FSR Husky Arm - %s - Homing complete!", __FUNCTION__);

    homing_complete_ = true;
    return true;
}

void FSRHuskyArm::write(double lift, double lift_speed, double sweep, double sweep_speed)
{
    if(!homing_complete_) return;

    double l = lift;
    if(l < min_lift_) l = min_lift_;
    if(l > max_lift_) l = max_lift_;
    int goal_l = -1.0*(l - min_lift_)*4048/(max_lift_ - min_lift_);

    double a = sweep;
    if(a < min_sweep_) a = min_sweep_;
    if(a > max_sweep_) a = max_sweep_;
    int goal_a = (a - min_sweep_)*max_sweep_steps_/(max_sweep_ - min_sweep_);

    jrk.setPosition(goal_l);
    nanotec.setPosition(goal_a);
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
    lift_ = -1.0*linear_position*(max_lift_ - min_lift_)/4048 + min_lift_;
    lift_speed_ = (lift_ - last_lift_)/delta_t.toSec();

    int rotation_position;
    if(!nanotec.getPosition(rotation_position))
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
                write(trajectory_.front().positions[lift_index_], trajectory_.front().velocities[lift_index_], trajectory_.front().positions[sweep_index_], trajectory_.front().velocities[sweep_index_]);
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
            if(as_active_ && goal_tolerance_[lift_index_].position != -1 && goal_tolerance_[sweep_index_].position != -1)
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

    write(trajectory_.front().positions[lift_index_], trajectory_.front().velocities[lift_index_], trajectory_.front().positions[sweep_index_], trajectory_.front().velocities[sweep_index_]);
}

void FSRHuskyArm::actionServerCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
{
    if(!homing_complete_)
    {
        control_msgs::FollowJointTrajectoryResult result;
        result.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
        m_as_goal_.setAborted(result);
        ROS_ERROR("FSR Husky Arm - %s - The homing routine was not performed yer!", __FUNCTION__);
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
    write(trajectory_.front().positions[lift_index_], trajectory_.front().velocities[lift_index_], trajectory_.front().positions[sweep_index_], trajectory_.front().velocities[sweep_index_]);

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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fsr_husky_arm");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    ROS_INFO("FSR Husky Arm -- ROS Hardware Driver");

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

    joint_state_thread.join();

    spinner.stop();

    return 0;
}
