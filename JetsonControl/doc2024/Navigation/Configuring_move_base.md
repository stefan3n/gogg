### [< back](../GuideForDocumentation.md)
# Configuring move_base

## URDF file
### It is essential to properly configure the robot's URDF. This file describes the spatial relationship between different components (it can also provide information for simulating the robot). This file contains **links** and **joints**. Links are the objects themselves and joints describe how they are attached to eachother.
### Generally in ROS, the robot's center is the link called *base_link* and all other components are connected to this link.
### The coordinate system is like this:
- ### X axis - positive is forward
- ### Y axis - positive is to the left
- ### Z axis - positive pointing up
### Measure the distances and specify them in the joints.
### For IMU, it throws an error if it is not centered on base_link. Even if our IMU was offset from the center, we just put *0 0 0* in the joint, to not get the error. Altough, we haven't had any localization issues, even with this discrepancy.

## Velocities and acceleration
### We found that it all worked well when we had low speeds. It's also important to have managable acceleration values. We set acceleration something smaller than the speed ranges, but we haven't tweaked this parameter much; we just stuck with something what worked.

## Obstacle avoidance
### To use LIDAR data for obstacle avoidance, you have to create an ObstacleLayer in the local_costmap and properly set all the parameters (a possible configuration is found in our *local_costmap_params.yaml*).
### You also have to set a footprint of the car in the *costmap_common_params.yaml* with either one of these parameters:
```YAML
footprint: [[1.4,0],[0.4,0.32],[-0.4,0.32],[-0.4,-0.32],[0.4,-0.32]] # sequence of points to describe a polygon centered on the robot
footprint_padding: 0.3 # enlarges the footprint by this much, as a saftey cushion
```
### or
```YAML
robot_radius: 1.5 # a circle centered on the robot
```

## Possible mistakes
### If, for example, you want to configure the local costmap, you could create a file *local_costmap_params.yaml*:
```YAML
local_costmap:
    param1: value
    param2: value
```
### If doing it like this, **DO NOT** set namespace in \<rosparam\> tag in launch file. It will make it set parameters for something like: *local_costmap.local_costmap*, and it will not work.
<pre><code class="lang-XML"><del>&lt;rosparam ... ns="local_costmap" /></del></code></pre>