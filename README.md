# ROS Pozyx Simulation
This package is to simulate the UWB range measurements in ROS environment. Generated data published to “uwb_data_topic” 

![](https://raw.githubusercontent.com/bekirbostanci/ros_pozyx_simulation/master/docs/1.png)

## UWB Anchors Add
The file that contains the UWB anchor information is in "launch/uwb_anchors_set.launch" file. It is possible to add or remove the UWB anchors as you like. </br>
Note : uwb tag frame name should be as follows uwb_anchor_0, uwb_anchor_1, uwb_anchor_2 ...

![](https://raw.githubusercontent.com/bekirbostanci/ros_pozyx_simulation/master/docs/2.png)


## UWB Tag information
Location of the tag has been taken from the robot position so to use this information "modelstate_index" parameter (for turtlebot3 modelstate_index =2) which is in "launch/uwb_simulation_initializing.launch" file must set correctly depending on the robot model used. It is possible to find your own robot parameter in "gazebo/model_states" topic.


## Initialize
Use the command below in terminal to start it with default settings

roslaunch pozyx_simulation uwb_simulation_initializing.launch


## Publish Topic
Name of the publisher topic is "uwb_data_topic". You can check it by using the command below in terminal

rostopic echo /uwb_data_topic

</br>
Message type consists of 3 different arrays <br>
1. anchors name => int64[] destination_id</br>
2. anchors distance to robot => float64[] distance</br>
3. time stamp => time[] stamp</br>

</br>

## Map and Rviz
If you want to start manually with custom maps. You can change map and map configuration in maps folder and start manually with the codes below.

rosrun map_server map_server map.yaml
rosrun rviz rviz
rosrun pozyx_simulation uwb_simulation.py

## Noise 
Noise has been added to the every UWB ranging data </br> 

np.random.normal(0, uwb_dist*0.015,1)
