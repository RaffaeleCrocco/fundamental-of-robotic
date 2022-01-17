## Download

  ```
  git clone git@github.com:RaffaeleCrocco/fundamental-of-robotic.git
  cd src
  git clone https://github.com/JenniferBuehler/gazebo-pkgs.git
  git clone https://github.com/JenniferBuehler/general-message-pkgs.git
  git clone https://github.com/pal-robotics/gazebo_ros_link_attacher.git
  ```

## Installation

After downloading all the files, build using `catkin build`

  ```
  cd fundamental-of-robotic
  source /opt/ros/noetic/setup.bash
  catkin build
  source devel/setup.bash
  ```
Give execution permission to the scripts

  ```
  chmod +x spawner/spawner_1.py
  chmod +x spawner/spawner_2.py

  chmod +x src/ur5/ur5_gazebo/scripts/send_joints.py
  chmod +x src/ur5/ur5_gazebo/scripts/vision.py
  ```

## Usage

Launch the gazebo world

  ```
  roslaunch ur5_gazebo ur5_world.launch
  ```
Unpause the simulation. Launch vision script

  ```
  rosrun ur5_gazebo vision.py
  ```
You can spawn random legos in random position using the `spawner_1` or `spawner_2` script

  ```
  ./spawner/spawner_1.py # Spawn only one random block at the time
  ./spawner/spawner_2.py # Spawn all 11 blocks at the same time
  ```

After you spawned lego blocks you can tell the `ur5` to pick them up

  ```
  rosrun ur5_gazebo send_joint.py
  ```

## Conclusion

We were able to satisfy the first two requests:
 - Assignment n. 1 *There is only one object in the initial stand, which is positioned with its base “naturally” in contact with the ground. The object can be of any of the classes specified by the project. Each class has an assigned position on the final stand, which is marked by a coloured shape representing the silhouette of the object. KPI 1-1 time to detect the position of the object KPI 1-2 time to move the object between its initial and its final positions, counting from the instant in which both of them have been identified.*
 - Assignment n. 2: *There are multiple objects on the initial stand, one for each class. There is no specific order in the initial configuration, except that the base of the object is “naturally” in contact with the ground. Each object has to be picked up and stored in the position prescribed for its class and marked by the object’s silhouette. KPI 2-1: Total time to move all the objects from their initial to their final positions.*



### Lessons learned
Building this project we learn how to use succesfully tools as:
  - ROS: a set of software libraries and tools that help you build robot applications.
  - Gazebo: open-source 3D robotics simulator.
  - Yolo: is a family of object detection architectures.
  - OpenCV: provides a real-time optimized Computer Vision library, tools, and hardware.
We inevitably faced a lot of challenge getting this project done.
The huge work load expected immediatly forced us to define roles inside the team: each of us worked on the whole project but mainly
directed one specific aspect of it.  

### Authors and main field

- [@Tobic0](https://github.com/Tobic0) : Arm
- [@RaffaeleCrocco](https://github.com/RaffaeleCrocco) : Computer vision
- [@elrich2610](https://github.com/elrich2610) : Gripper

