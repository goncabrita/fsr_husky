#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/Odometry.h>

static ros::Publisher odom_pub;
std::string frame_id, child_frame_id;

void callback(const nav_msgs::Odometry& odom) {

  if (odom_pub) {
      nav_msgs::Odometry odom_corrected=odom;

      if(odom_corrected.twist.twist.angular.z==0.0 && odom_corrected.twist.twist.linear.x!=0.0){ //Andar sem rotacao
          odom_corrected.pose.covariance.at(0) = 0.0025;  //x
          odom_corrected.pose.covariance.at(7) = 0.000000001;  //y
      }
      else if(odom_corrected.twist.twist.angular.z!=0.0 && odom_corrected.twist.twist.linear.x!=0.0){ //Andar com rotacao
          odom_corrected.pose.covariance.at(0) = pow(odom_corrected.twist.twist.angular.z,2)/10;  //x
          odom_corrected.pose.covariance.at(7) = pow(odom_corrected.twist.twist.angular.z,2)/10;  //y
      }
      else if(odom_corrected.twist.twist.angular.z!=0.0 && odom_corrected.twist.twist.linear.x==0.0){ //Rotacao em torno de si
          odom_corrected.pose.covariance.at(0) = 0.007;  //x (variance ~ 0.25m)
          odom_corrected.pose.covariance.at(7) = 0.007;  //y (variance ~ 0.25m)
      }
      odom_pub.publish(odom_corrected);
  }
}


int main (int argc, char **argv) {
  ros::init(argc, argv, "odometry_covariance_correction_node");
  ros::NodeHandle n;


  odom_pub = n.advertise<nav_msgs::Odometry>("encoder_corrected", 10);

  ros::Subscriber sub_odo = n.subscribe("encoder", 10, callback);

  ros::spin();
}

