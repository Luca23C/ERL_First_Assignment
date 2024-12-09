# First assignment of Experimental Robot Laboratory   
This assignment works with ROS2 foxy distribution.   
There are two main package: `robot_urdf` and `ros2_aruco`.   

## robot_urdf package
The environment for the simulation is in the **worlds** folder, where it's possible fo find a file called *aruco.world* in which is specified the displacement of the aruco markers in the environment.   
Inside the **config** folder the file called *config_motors.yaml* contain the information related to the joint and the controller manager of the camera.
Finally within the **urdf** folder there is the file *robot4.xacro* in which there are the parameters of the camera.
---

## How to run the simulation

In the first terminal paste the following line:   
```bash
ros2 launch robot_urdf gazebo.launch.py
```
This command starts the simulation environment.

In a second terminal launch this command:
```bash
ros2 run ros2_aruco aruco_node
```
This line starts the node related to the detection of the markers into the environment. This node allow us to detect the id of each marker and then once all marker ids are detected this node sort them and crate a list.   
Once this list is ready the robot will publish on a custom topic, by following the list order, the image of the aruco in which is present a circle around it.   

To verify this you should go on Rviz and add the topic called: `/output/image/circle`.   

In a third terminal you should run this following line, which allow the motion. It's important to point out that there are to ways for analyze the environment:   
a) By moving the robot controlling its angular velocity:   
```bash
ros2 run robot_urdf control_robot_vel
```   
b) By controlling the rotation of the camera:   
```bash
ros2 run robot_urdf control_camera
```  

Lastly, in a fourth terminal you can check the status of the operation. When the robot will publish all the markers on the custom topic, it will be publish on the topic called `/status` that the operation was done.   
Additionally in the simulation it's possibly to see that the robot stop its movement.
