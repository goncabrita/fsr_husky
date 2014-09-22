#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <fsr_husky_driver/laser_tiltAction.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <math.h>

class LaserTiltAction
{
protected:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    // NodeHandle instance must be created before this line. Otherwise strange error may occur.
    actionlib::SimpleActionServer<fsr_husky_driver::laser_tiltAction> as_;
    std::string action_name_;
    // create messages that are used to published feedback/result
    fsr_husky_driver::laser_tiltFeedback feedback_;
    fsr_husky_driver::laser_tiltResult result_;
    sensor_msgs::JointState last_tilt_joint;

    ros::Subscriber joint_states_sub;
    ros::Publisher tilt_command_pub_;
    double threshold_;


public:
    LaserTiltAction(std::string name) :
        as_(nh_,name, boost::bind(&LaserTiltAction::executeAction, this, _1), false),
        action_name_(name), nh_private_("~")
    {
        nh_private_.param("threshold",threshold_,0.02);
        as_.start();
        joint_states_sub = nh_.subscribe<sensor_msgs::JointState>("/joint_states", 10, &LaserTiltAction::jointStateCallback, this);
        tilt_command_pub_ = nh_.advertise<std_msgs::Float64>("ptu_d46_tilt_controller/command",1);
        last_tilt_joint.position.resize(1);
        last_tilt_joint.velocity.resize(1);
        last_tilt_joint.effort.resize(1);
    }

    ~LaserTiltAction(void)
    {
    }

    void executeAction(const fsr_husky_driver::laser_tiltGoalConstPtr &goal)
    {
        ros::Rate r(5);
        bool success = true;
        bool reached_position = false;

        while(!reached_position)
        {

            if(as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                // set the action state to preempted
                as_.setPreempted();
                success = false;
                break;
            }

            feedback_.current_position = last_tilt_joint.position[0];

            ROS_INFO("feedback = %f, goal = %f, cond %f", feedback_.current_position, goal->target_position,fabs(feedback_.current_position - goal->target_position));

            if(fabs(feedback_.current_position - goal->target_position) < threshold_)
            {
                reached_position = true;
            }
            else
            {
                std_msgs::Float64 cmd;
                cmd.data = goal->target_position;
                tilt_command_pub_.publish(cmd);
            }
            as_.publishFeedback(feedback_);

            r.sleep();
        }
        if(success)
        {
            result_.position = feedback_.current_position;
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            // set the action state to succeeded
            as_.setSucceeded(result_);
        }
    }

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& states)
    {
        for(int i = 0; i < states->name.size(); i++)
        {
            if((states->name[i]).compare("ptu_d46_tilt_joint") == 0)
            {
                last_tilt_joint.header = states.get()->header;
                last_tilt_joint.position[0] = states->position[i];
                last_tilt_joint.velocity[0] = states->velocity[i];
                last_tilt_joint.effort[0] = states->effort[i];
            }
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_tilt_action_server");

    LaserTiltAction dynamixel_tilt(ros::this_node::getName());
    ros::spin();

    return 0;
}
