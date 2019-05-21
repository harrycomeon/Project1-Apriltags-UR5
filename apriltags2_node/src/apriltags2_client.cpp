#include <ros/ros.h>
#include <apriltags2_node/LocalizePart.h>
#include <tf/tf.h>

#include <moveit/move_group_interface/move_group_interface.h>

class ScanNPlan
{
public:
  ScanNPlan(ros::NodeHandle& nh)
  {
    vision_client_ = nh.serviceClient<apriltags2_node::LocalizePart>("localize_part");
  }

  void start(const std::string& base_frame)
  {
    ROS_INFO("Attempting to localize part");
    // Localize the part
    apriltags2_node::LocalizePart srv;

    srv.request.base_frame = base_frame;
    ROS_INFO_STREAM("Requesting pose in base frame: " << base_frame);

    //if (!vision_client_.call(srv))
    if(srv.response.pose.position.z)
    {
      ROS_ERROR("Could not localize part");
      //ROS_INFO_STREAM("part localized: " << move_target);
      return;
    }

    ROS_INFO_STREAM("part localized: " << srv.response);
    //ROS_INFO_STREAM("frame_id: " << srv.request.base_frame);
    //LocalizePart服务的响应后来初始化一个新move_target变量
    move_target = srv.response.pose;
    //创建一个对象，move_group（）构造函数定义计划组名称
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");

    // Plan for robot to move to part
    //使用move_group对象的setPoseTarget功能设置所需的笛卡尔目标位置
    move_group.setPoseReferenceFrame(base_frame);
    move_group.setPoseTarget(move_target);
    move_group.move();

  }
  
private:
  // Planning components
  ros::ServiceClient vision_client_;
  geometry_msgs::Pose move_target;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "apriltags2_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_node_handle ("~");

  //如所描述的在这里，所述move_group.move()命令需要使用“异步”旋转器，以允许阻塞期间ROS消息的处理move()命令
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();

  while(ros::ok())
  {
    ROS_INFO("ScanNPlan node has been initialized");
    std::string base_frame;
    private_node_handle.param<std::string>("base_frame", base_frame, "world"); // parameter name, string object reference, default value

    ScanNPlan app(nh);
    ros::Duration(.5).sleep();  // wait for the class to initialize
    app.start(base_frame);

    //ros::waitForShutdown();
    //ros::spin();
  }
  return 0;
}
