# Download

  ```
  git clone https://github.com/RaffaeleCrocco/fundamental-of-robotic.git
  ```

# Installation

After downloading all the files, build using catkin build

  ```
  cd fundamental-of-robotic
  source /opt/ros/noetic/setup.bash
  catkin build
  source devel/setup.bash
  ```
To launch the gazebo simulation

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
You can visualize the camera feed

 ```
 rqt
 ```
Go to plugins -> visualization -> image view and then select /camera/color/image_raw or /camera/depth/image_raw
