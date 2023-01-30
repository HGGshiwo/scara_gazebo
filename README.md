# scara_gazebo

## install

first clone the package into catkin workspace
```
cd ~/catkin_ws/src
git clone https://github.com/HGGshiwo/scara_gazebo.git
```

build package
```
cd ~/catkin_ws/
catkin_make
```

## test
```
source ~/catkin_ws/devel/setup.bash
roslaunch scara_gazebo scara_gazebo.launch
```
that will launch an empty world with two scara robot carring red cylinders.

![image](https://user-images.githubusercontent.com/68278678/214795488-2e5f8afc-6089-4699-b495-00ba5fc92c07.png)

![image](https://user-images.githubusercontent.com/68278678/214795564-f43b23d0-9b65-497b-8c46-13456bce3b8a.png)


## how to use
create a controller in scara_gazebo/src/scripts, then inherit the class ScaraInterface. methos can be used:
|class methed       |description|
|-                  |-|
|```move_to(pose)```|move the gripper to the given pose|
|```move_up()```    |move the gripper up|
|```move_down()```  |move the gripper down|
|```grasp()```      |grasp the gripper|
|```release()```    |release the gripper|   
|```update()```     |update joint position and velocity, should be called in main loop.

for using more than one robot:
1. just change the ```robot_name``` and ```controller_name``` in ```Scara_interface.__init__()``` on creating instances.
2. spawn the model in launch file with the relatively robot_name as the arg of -model

ScaraController is an example of how to controll the robot.

for more information, check: [https://blog.csdn.net/HGGshiwo/article/details/128739982](https://blog.csdn.net/HGGshiwo/article/details/128739982)