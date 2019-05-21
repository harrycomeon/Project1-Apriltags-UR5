# UR5_AprilTags
To complete positioning base upon UR5 and AprilTags 

1,
Firstly，we should create workroom .
Secondly，install universal_robot package and industrial_core package （this includes industrial_robot_simulator package ）
Thirdly，install ur_modern_driver when your ur5 vesion >= 3.0 
Finally,catkn_make and source.
详细步骤：
1，首先添加source /opt/ros/kinetic/setup.bash到.bashrc工作目录
因为You will need to run this command on every new shell you open to have access to the ROS commands,（第一次创建时）
2，echo $ROS_PACKAGE_PATH 
（当有其它工作目录时，显示当前工作目录）
3，创建工作空间及下载包
mkdir -p ~/universal_robot/src
cd ~/universal_robot/src
git clone https://github.com/ros-industrial/universal_robot.git
git clone https://github.com/ros-industrial/industrial_core.git	  

4，下载ur_modern_driver驱动
cd universal_robot
git clone https://github.com/beta-robots/ur_modern_driver.git

5，修改驱动文件
gedit ~/universal_robot/src/universal_robot/ur_modern_driver/config/ur5_controllers.yaml

    在文件下添加
controller_list:
 - name: /vel_based_pos_traj_controller #or /pos_based_pos_traj_controller
   action_ns: follow_joint_trajectory
   type: FollowJointTrajectory
   default: true
   joints:
      - shoulder_pan_joint
      - shoulder_lift_joint
      - elbow_joint
      - wrist_1_joint
      - wrist_2_joint
      - wrist_3_joint

6，编译
cd ~/universal_robot
catkin_make

7，指定新建的工作目录
    修改.bashrc文件在最后添加
source /home/comeon-harry/universal_robot/devel/setup.bash

    在终端下输入    
 . /home/comeon-harry/universal_robot/devel/setup.bash  （对应自己的文件名）（.后带空格）

     输入echo $ROS_PACKAGE_PATH，发现工作目录添加上
     
