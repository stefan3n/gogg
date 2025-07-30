### [< back](../GuideForDocumentation.md)
# Cartographer - SLAM ROS package

### To install it follow the instructions on [this page](https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html).

### When you run the *rosdep* command:
```Bash
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
```
### you will  get an error about *libabsl*. To fix this, go into *src/cartographer/package.xml* and delete this line:
```XML
<depend>libabsl-dev</depend>
```



## Adding dead spots to LIDAR
### If you have parts of the robot that are always visible to the LIDAR, you may want remove the corresponding angles' reading, as to not have false-positive obstacles appear on the local costmap.
### To do this you need to start a *scan_to_scan_filter_chain* node, to which you add a *laser_filters/LaserScanAngularBoundsFilterInPlace* layer.
### *laser_filter.yaml*
```YAML
scan_filter_chain:
        - name: angle
          type: laser_filters/LaserScanAngularBoundsFilterInPlace
          params:
                  lower_angle: -0.785
                  upper_angle: 0.785
```
### *launchFile.launch*
```XML
<node pkg="laser_filters" type="scan_to_scan_filter_chain" name="laser_filter"> <rosparam command="load" file="$(find navigation)/config/laser_filter.yaml" />
</node>
```
### This node listens to */scan* and publishes to */scan_filtered*. Because of this you have to add this to any node that uses */scan* (if you can't change it explicitly):
```XML
<remap from="scan" to="scan_filtered" />
```
### Only values between *-pi* and *pi* are allowed, and lower_angle must be smaller than upper angle.

### Note: the difference between *LaserScanAngularBoundsFilterInPlace* and *LaserScanAngularBoundsFilter*.
### *InPlace* removes readings between lower_angle and upper_angle. The other one keeps only the reading between those values.

## Default RPLidar A1 axes
### Positive angles are counter-clockwise and negative ones clockwise. The angle 0 is at the X axis.
- ### X positive points towards the motor
- ### Y positive points to the right (if we consider the front of the LIDAR to be the side opposite of the motor)


# EDIT
### Run this before installing cartographer:
```sudo pip3 install sphinx --upgrade```
### Add ```-DPYTHON_EXECUTABLE=(path to python3)``` when building in case of cartographer finding python2 instead of python3