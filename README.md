# Download

  ```
  git clone git@github.com:RaffaeleCrocco/fundamental-of-robotic.git
  cd src
  git clone https://github.com/JenniferBuehler/gazebo-pkgs.git
  git clone https://github.com/JenniferBuehler/gazebo-pkgs.git
  ```

# Installation

After downloading all the files, build using catkin build

  ```
  cd fundamental-of-robotic
  source /opt/ros/noetic/setup.bash
  catkin build
  source devel/setup.bash
  ```
Give execution permission to the scripts

  ```
  chmod +x spawner/spawner_1.py
  
  chmod +x src/ur5/ur5_gazebo/scripts/send_joints.py
  ```

You can launch specific scenarios

  ```
  roslaunch ur5_gazebo ur5_world_1.launch
  ```
Unpause the simulation. You can spawn random legos in random position using the spawner script

  ```
  ./spawner/spawner_1.py
  ```

You can send commands to the joints

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
