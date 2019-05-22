#include "ros/ros.h"
#include "tf2_msgs/TFMessage.h"
#include <iostream>


class Localizer
{
public:
  Localizer(ros::NodeHandle& nh)
  {
      //last_msg_->detections[0].pose.pose.pose.position.z=0;
      ar_sub_ = nh.subscribe<tf2_msgs::TFMessage>("tf", 1,
      &Localizer::number_callback, this);
      //ROS_INFO("1Vision node starting");
      
      
  }
  void number_callback(const tf2_msgs::TFMessage::ConstPtr& msg)
  {
       last_msg_ = msg;
       //if (last_msg_->detections[0].pose.pose.pose.position.z!=0) {
       //last_msg_ = msg;
       ROS_INFO("1Vision node starting");
       ROS_INFO_STREAM(last_msg_->transforms[0].transform);
       //if(last_msg_->detections[0])
       ROS_INFO("2Vision node starting");
       //ar_sub_.shutdown();
  }

  ros::Subscriber ar_sub_;
  
  tf2_msgs::TFMessageConstPtr last_msg_;
};


int main(int argc, char **argv)
{
  
    int i=0;
    ros::init(argc, argv,"apriltag_detector_subscriber");
    ros::NodeHandle node_obj;

    Localizer localizer(node_obj);
    //ros::Subscriber number_subscriber = node_obj.subscribe("tag_detections",1,number_callback);
    ROS_INFO("3Vision node starting");
    ros::spin();
    ros::Duration(.5).sleep();
 
    //return 0;
}
