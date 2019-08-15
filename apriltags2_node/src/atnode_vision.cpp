#include "ros/ros.h"
#include "tf2_msgs/TFMessage.h"
#include <apriltags2_node/LocalizePart.h>
#include <iostream>

#include <tf/transform_listener.h>

class Localizer
{
public:
  Localizer(ros::NodeHandle& nh)
  {
       
      ar_sub_ = nh.subscribe<tf2_msgs::TFMessage>("tf", 1, &Localizer::number_callback, this);
      ROS_INFO("Vision node starting1");
      server_ = nh.advertiseService("localize_part", &Localizer::localizePart, this); //将服务公布给ROS主服务器
  }

  bool localizePart(apriltags2_node::LocalizePart::Request& req,
                  apriltags2_node::LocalizePart::Response& res)
  {
     // Read last message
      tf2_msgs::TFMessageConstPtr p = last_msg_;  
      //将线上格式geometry_msgs::Pose转换为：tf::Transform object
      tf::Transform cam_to_target;

      tf::transformMsgToTF(p->transforms[0].transform, cam_to_target);

      //使用侦听器对象request.base_frame从ARMarker消息（应该是“camera_frame”）查找和参考帧之间的最新转换：lookupTransform（）可以获得两个坐标系的转换-旋转的平移
      tf::StampedTransform req_to_cam;   //定义存放坐标变换的变量

        try{
        ros::Time now = ros::Time::now();
        //查询是否存在req.base_frame（世界坐标系）到相机的坐标变换矩阵
        listener_.waitForTransform(req.base_frame, p->transforms[0].header.frame_id, 
                              now, ros::Duration(1.0));
        //获取存在的req.base_frame（世界坐标系）到相机的坐标变换矩阵
        listener_.lookupTransform(req.base_frame, p->transforms[0].header.frame_id,
                             now, req_to_cam);
            }
       catch (tf::TransformException &ex) {
      	   ROS_ERROR("%s",ex.what());
      	   ros::Duration(1.0).sleep();
      	   //continue;
      }
      //ROS_INFO_STREAM("req_to_cam: " << req_to_cam);
      //对象姿势转换为目标帧,定义存放转换信息（平移，转动）的变量
      tf::Transform req_to_target;

  /*    tf::Transform transform;
      transform.setOrigin(tf::Vector3(0, 0, 0));//3. 设置坐标原点，（0.1，0，0.2）为子坐标系激光坐标系base_laser在父坐标系小车base_link坐标系中的坐标，
      tf::Quaternion q;// 4.定义旋转
      q.setRPY(90, 0, 0);//（0，0，0）为base_laser在base_link坐标系下的roll(绕X轴)，pitch(绕Y轴)，yaw(绕Z轴) 的旋转度数，现在都是0度
      transform.setRotation(q);
*/
      req_to_target =  req_to_cam * cam_to_target;
      //req_to_target = transform * req_to_target;

      //在服务响应中返回转换后的姿势
      tf::poseTFToMsg(req_to_target , res.pose);

      return true;
  }
  void number_callback(const tf2_msgs::TFMessage::ConstPtr& msg)
  {
      if(msg->transforms[0].header.frame_id=="camera")
         last_msg_ = msg;
      //ROS_INFO_STREAM(last_msg_->detections[0].pose.pose.pose);
  }

  ros::Subscriber ar_sub_;
  tf2_msgs::TFMessageConstPtr last_msg_;
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
