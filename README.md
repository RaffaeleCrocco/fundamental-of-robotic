# Installation

After downloading all the files, build using catkin build

  ```
  cd fundamental-of-robotic
  catkin build
  source devel/setup.bash
  ```
To launch the the gazebo simulation

  ```
  roslaunch ur5_gazebo ur5_lego.launch
  ```
Unpause the simulation. You can send commands to the joints

  ```
  rosrun ur5_gazebo send_joint.py
  ```
  
You can send commands to the gripper (value between 0.0 and 0.8)

  ```
  rosrun ur5_gazebo send_gripper.py --value 0.5
  ```
