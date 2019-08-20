# ROS Pozyx Simulation
This project can use with ros platform for simulating uwb sensors. In this project's main aim is publishing simulating uwb anchors and tag distance and  visualizing uwb sensors position. This project tested with turtlebot3 simulation.  

![](https://raw.githubusercontent.com/bekirbostanci/ros_pozyx_simulation/master/docs/1.png)

## UWB Anchors Add
In this file "launch/uwb_anchors_set.launch" contain uwb anchors. You can add and remove uwb anchors in this file.</br>
*Note : uwb tag frame name should be like this uwb_anchor_0, uwb_anchor_1, uwb_anchor_2 ...*

![](https://raw.githubusercontent.com/bekirbostanci/ros_pozyx_simulation/master/docs/2.png)


## UWB Tag Add 
This project tag is simuating robot orginal position so that you have to instert "launch/uwb_simulation_initializing.launch" "modelstate_index" parameter (for turtlebot3 modelstate_index = 2). To find its own robot parameter in "gazebo/model_states" topics.


## Initialize
Start with default settings 
```
roslaunch pozyx_simulation uwb_simulation_initializing.launch
```

## Publish Topic
Publisher topic name is "uwb_data_topic". You can check with this command
```
rostopic echo /uwb_data_topic
```
</br>
Message has created 3 different arrays <br>
1. anchors name => int64[] destination_id</br>
2. anchors distance to robot => float64[] distance</br>
3. time stamp => time[] stamp</br>

</br>

## Map and Rviz
If the start manually in the rviz you can add tf components. You can change map and map configuration in maps folder. Start manually you can use this order.
```
rosrun map_server map_server map.yaml
rosrun rviz rviz
rosrun pozyx_simulation uwb_simulation.py
```
## Noise 
The uwb noise is added for each uwb sensor </br> 
```
np.random.normal(0, uwb_dist*0.015,1)  
```
