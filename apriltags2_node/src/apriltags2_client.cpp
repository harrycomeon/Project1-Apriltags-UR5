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
    //ROS_INFO_STREAM("Requesting pose in base frame: " << base_frame);
    //如果服务回调不成功
    if(!vision_client_.call(srv))
    {
      ROS_ERROR("Could not localize part");
      return ;
    }

   // ROS_INFO_STREAM("part localized: " << srv.response);

    move_target = srv.response.pose;
    move_target1 = srv.response.pose;
   // move_target.position.z=move_target1.position.z+0.15;
   // move_target.position.y=move_target1.position.y+0.045;
   // move_target.position.x=move_target1.position.x-0.030;

   //绕x轴逆时针变化90度
  /*
    move_target.orientation.x=0.70710678119*move_target1.orientation.x+0.70710678119*move_target1.orientation.w;
    move_target.orientation.y=0.70710678119*move_target1.orientation.y-0.70710678119*move_target1.orientation.z;
    move_target.orientation.z=0.70710678119*move_target1.orientation.z+0.70710678119*move_target1.orientation.y;
    move_target.orientation.w=-0.70710678119*move_target1.orientation.x+0.70710678119*move_target1.orientation.w;
 */

    //ROS_INFO_STREAM("move_target: " << move_target);

    //创建一个对象，move_group（）构造函数定义计划组名称
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");

    moveit::planning_interface::MoveItErrorCode success;
    moveit::planning_interface::MoveGroupInterface::Plan plan;

     //设置初始位置
    move_group.setStartState(*move_group.getCurrentState());

    // Plan for robot to move to part
    //使用move_group对象的setPoseTarget功能设置所需的笛卡尔目标位置
    //Get the name of the end-effector link
    //end_effector_link =move_group.getEndEffectorLink();
    //Allow replanning to increase the odds of a solution
    //move_group.allowReplanning(True);
    // Allow some leeway in position (meters) and orientation (radians)

    //move_group.setGoalPositionTolerance(0.01);
    //move_group.setGoalOrientationTolerance(0.05);
    move_group.setPoseReferenceFrame(base_frame);
    //move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(move_target); 
    //move_group.setPlannerId("RRTConnectkConfigDefault");
/*
    move_group.setPlanningTime(10);
    //进行运动规划
    success = move_group.plan(plan);   //运动规划输出
    ROS_INFO("Visualizing plan (stateCatch pose) %s",success == moveit_msgs::MoveItErrorCodes::SUCCESS ? "SUCCESS" : "FAILED");
    if (success  == moveit_msgs::MoveItErrorCodes::SUCCESS)  
    move_group.execute(plan);
    move_group.setStartState(*move_group.getCurrentState());

*/    move_group.move();
  }
  
private:
  // Planning components
  ros::ServiceClient vision_client_;
  geometry_msgs::Pose move_target;
  geometry_msgs::Pose move_target1;
  std::string end_effector_link;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "apriltags2_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_node_handle ("~");
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();

  //ros::Rate r(10); // 10 hz,它的功能就是先设定一个频率，然后通过睡眠度过一个循环中剩下的时间，来达到该设定频率。
    ROS_INFO("ScanNPlan node has been initialized");
    std::string base_frame;
    private_node_handle.param<std::string>("base_frame", base_frame, "world"); // world parameter name, string object reference, default value
    ScanNPlan app(nh);
    ros::Duration(.5).sleep();  // wait for the class to initialize
  while(ros::ok())
  {
    
    app.start(base_frame);
    //ros::waitForShutdown();
  }
  return 0;
}
