#include "ros/ros.h"
#include "apriltags2_ros/AprilTagDetectionArray.h"
#include <apriltags2_node/LocalizePart.h>
#include <iostream>

#include <tf/transform_listener.h>

class Localizer
{
public:
  Localizer(ros::NodeHandle& nh)
  {
      ar_sub_ = nh.subscribe<apriltags2_ros::AprilTagDetectionArray>("tag_detections", 1,
      &Localizer::number_callback, this);
      server_ = nh.advertiseService("localize_part", &Localizer::localizePart, this); //将服务公布给ROS主服务器
  }

  bool localizePart(apriltags2_node::LocalizePart::Request& req,
                  apriltags2_node::LocalizePart::Response& res)
  {
     // Read last message
      apriltags2_ros::AprilTagDetectionArrayConstPtr p = last_msg_;  
      if (!p) 
        return false;

      //res.pose = p->detections[0].pose.pose.pose;
      //ROS_INFO_STREAM("frame_id: " << p->detections[0].pose.header.frame_id);
      
      //将线上格式geometry_msgs::Pose转换为：tf::Transform object
      //tf::Transform target_to_cam;
      tf::Transform cam_to_target;
      tf::poseMsgToTF(p->detections[0].pose.pose.pose, cam_to_target);//可以换成target to cam
      //cam_to_target=target_to_cam;//可以换成target to cam

      //使用侦听器对象request.base_frame从ARMarker消息（应该是“camera_frame”）查找和参考帧之间的最新转换：lookupTransform（）可以获得两个坐标系的转换-旋转的平移
      tf::StampedTransform req_to_cam;   //定义存放坐标变换的变量
    //  try{
     //      listener_.lookupTransform(req.base_frame, p->detections[0].pose.header.frame_id, ros::Time(0), req_to_cam);
     //  }
        try{
        ros::Time now = ros::Time::now();
        listener_.waitForTransform(req.base_frame, p->detections[0].pose.header.frame_id, 
                              now, ros::Duration(1.0));
        listener_.lookupTransform(req.base_frame, p->detections[0].pose.header.frame_id,
                             now, req_to_cam);
            }
       catch (tf::TransformException &ex) {
      	   ROS_ERROR("%s",ex.what());
      	   ros::Duration(1.0).sleep();
      	   //continue;
      }
      //对象姿势转换为目标帧,定义存放转换信息（平移，转动）的变量
      tf::Transform req_to_target;
      req_to_target = req_to_cam * cam_to_target;
      //在服务响应中返回转换后的姿势
      tf::poseTFToMsg(req_to_target, res.pose);

      return true;
  }
  void number_callback(const apriltags2_ros::AprilTagDetectionArray::ConstPtr& msg)
  {
      last_msg_ = msg;
      //ROS_INFO_STREAM(last_msg_->detections[0].pose.pose.pose);
  }

  ros::Subscriber ar_sub_;
  apriltags2_ros::AprilTagDetectionArrayConstPtr last_msg_;
  ros::ServiceServer server_;
  //定义监听器
  tf::TransformListener listener_; 
};


int main(int argc, char **argv)
{
    ros::init(argc, argv,"apriltag_detector_subscriber");
    ros::NodeHandle node_obj;
    Localizer localizer(node_obj);
    //ros::Subscriber number_subscriber = node_obj.subscribe("tag_detections",1,number_callback);
    ROS_INFO("Vision node starting");
    ros::spin();
    //return 0;
}
