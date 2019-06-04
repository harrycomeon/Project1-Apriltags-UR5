#include <ros/ros.h>
#include <apriltags2_node/LocalizePart.h>
#include <tf/tf.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <apriltags2_node/PlanCartesianPath.h>


class ScanNPlan
{
public:
  ScanNPlan(ros::NodeHandle& nh) : ac_("arm_controller/follow_joint_trajectory", true)//joint_trajectory_action
  {
    vision_client_ = nh.serviceClient<apriltags2_node::LocalizePart>("localize_part");
    cartesian_client_ = nh.serviceClient<apriltags2_node::PlanCartesianPath>("plan_path");
  }

  void start(const std::string& base_frame)
  {
    ROS_INFO("Attempting to localize part");
    // Localize the part
    apriltags2_node::LocalizePart srv;

    srv.request.base_frame = base_frame;
    ROS_INFO_STREAM("Requesting pose in base frame: " << base_frame);
    //如果服务回调不成功
    if(!vision_client_.call(srv))
    {
      ROS_ERROR("Could not localize part");
      return ;
    }

    ROS_INFO_STREAM("part localized: " << srv.response);
    //ROS_INFO_STREAM("frame_id: " << srv.request.base_frame);
    //LocalizePart服务的响应后来初始化一个新move_target变量
    //ROS_INFO_STREAM("part localized: " << srv.response.pose.orientation);
    //srv.response.pose.orientation.z=(-1)*srv.response.pose.orientation.z;
    //srv.response.pose.orientation.y=(-1)*srv.response.pose.orientation.y;
    //srv.response.pose.orientation.x=(-1)*srv.response.pose.orientation.x;
    //ROS_INFO_STREAM("part localized: " << srv.response.pose.orientation);
    //交换了y，z轴，实现了机械手朝下
    move_target = srv.response.pose;
    move_target1=srv.response.pose;
    move_target.orientation.y=move_target1.orientation.z;
    move_target.orientation.z=move_target1.orientation.y;
 
    //ROS_INFO_STREAM("part localized: " << move_target.orientation);
    //创建一个对象，move_group（）构造函数定义计划组名称
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");

    // Plan for robot to move to part
    //使用move_group对象的setPoseTarget功能设置所需的笛卡尔目标位置
    move_group.setPoseReferenceFrame(base_frame);
    move_group.setPoseTarget(move_target); 
    move_group.move();


    // Plan cartesian path
    apriltags2_node::PlanCartesianPath cartesian_srv;
    cartesian_srv.request.pose = move_target;
    if (!cartesian_client_.call(cartesian_srv))
    {
      ROS_ERROR("Could not plan for path");
      return;
    }

    // Execute descartes-planned path directly (bypassing MoveIt)
    ROS_INFO("Got cart path, executing");
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory = cartesian_srv.response.trajectory;
    ac_.sendGoal(goal);
    ac_.waitForResult();
    ROS_INFO("Done");

  }
  
private:
  // Planning components
  ros::ServiceClient vision_client_;
  geometry_msgs::Pose move_target;
  geometry_msgs::Pose move_target1;
  ros::ServiceClient cartesian_client_;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_;
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
    ros::Duration(.1000).sleep();
    ros::Duration(.1000).sleep();
    //ros::waitForShutdown();
    //ros::spinOnce();
    ros::Duration(.5).sleep();
  }
  return 0;
}
