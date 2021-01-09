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

## How to use 
Firstly run gazebo simulation after than run rviz
1. run rqt 
2. in rqt window "Plugins" => "Topics" => "Topic Monitor" 
3. in that window find /gazebo/model_states/ topic and checked that line 
4. control robot pose in which parameter  for example ['ground_plane','robot_symbol','turtlebot_waffle'] in thise case index 2 
5. go repo folder /launch/uwb_manually_initializing.launch file and change modelstate_index parameter 
 ```
  <node pkg="pozyx_simulation" name="pozyx_simulation"  type="uwb_simulation.py" output="screen">
        <param name="modelstate_index" value="2" />
```
For this process get robot or drone real position afther that to place uwb anchor in maps. 
1. go repo folder /launch/uwb_anchors_set.launch and open 
2. for each uwb anchor's set a name and position for example 
```
<launch>
   <node pkg="tf" type="static_transform_publisher" name="uwb_anchor_0x6e36" args="1.4 2.0 0.0 0 0 0 1 map uwb_anchor_0 100" />
   <node pkg="tf" type="static_transform_publisher" name="uwb_anchor_0x6e33" args="-1.4 2.0 0.0 0 0 0 1 map uwb_anchor_1 100" />
   <node pkg="tf" type="static_transform_publisher" name="uwb_anchor_0x6e49" args="1.4 -2.0 0.0 0 0 0 1 map uwb_anchor_2 100" />   
   <node pkg="tf" type="static_transform_publisher" name="uwb_anchor_0x6e30" args="-1.4 -2.0 0.0 0 0 0 1 map uwb_anchor_3 100" />
</launch>
```
3. go /src/uwb_simulation.py file and add anchors 

```
def uwb_simulate(sensor_pos):
    while not rospy.is_shutdown():
        time.sleep(0.1)
        all_distance = [] 
        all_destination_id = []

        for i in range(len(sensor_pos)):
            #calculate distance uwb to robot for all anchors 
            dist = calculate_distance(sensor_pos[i])   
            all_distance.append(dist) 
        
        #uwb_anchors_set.launch same order (not important for simulation)
        all_destination_id.append(0x694b)
        all_destination_id.append(0x6948)
        all_destination_id.append(0x694f)
        all_destination_id.append(0x694a)
            
        #publish data with ROS             
        publish_data(all_destination_id , all_distance)  
```

Now pack is ready to run.  
1. Firstly call anchors 
`roslaunch pozyx_simulation uwb_anchors_set.launch `
2. After that run main calculation script 
`roslaunch pozyx_simulation uwb_manually_initializing.launch`

For check all pack is working  
`rostopic echo /uwb_data_topic` 

This process give robot to uwb_anchor distance for example 
```
destination_id: [26955, 26952, 26959, 26954]
distance: [4192.039813830616, 2564.0268225317145, 3703.2810901751322, 1621.13755039097]
stamp: 
  - 
    secs: 1039
    nsecs: 440000000
  - 
    secs: 1039
    nsecs: 440000000
  - 
    secs: 1039
    nsecs: 440000000
---
```

This simulation give only robot to anchors distance. For calculate robot position use ieuagv_localization repo
https://github.com/bekirbostanci/ieuagv_localization


## Run
```
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch 

export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_navigation turtlebot3_navigation.launch 

roslaunch pozyx_simulation uwb_anchors_set.launch 

rosrun pozyx_simulation uwb_simulation.py 

rosrun advoard_localization kalman_filter_localization.py 
```
