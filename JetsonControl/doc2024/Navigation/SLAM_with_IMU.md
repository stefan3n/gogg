### [< back](../GuideForDocumentation.md)
# SLAM with IMU

### If localization is poor with just Lidar data, one solution would be to add another sensor. This could be wheel encoders, IMU or any other odometry. We decided to try out an IMU (inertial measurement unit) which provides 3-axis acceleration and angular velocity.

### The SLAM package which we used, [Cartographer](https://github.com/ros2/cartographer_ros), explicitly enabled the use of IMU data to improve localization (it should also have ways to do sensor fusion with any odometry).

## How to setup IMU
### The process is detailed [here](./IMU.md). An important step is to rename the */imu/data_raw* topic to */imu*, because that's what Cartographer is expecting. This should normally be done with a **\<remap/>** tag inside the launch file, but it didn't work for us. To fix this we went into the driver's source file **mpu6050_node.cpp** and changed the output topic by modifing the 39th line:
```C++
mpu_data_pub_ = nh_.advertise<sensor_msgs::Imu>("imu", 1);
```

## Modifying the .launch files
### We used [this package](https://github.com/Andrew-rw/gbot_core) to run the Cartographer node. Inside the file *gbot_lidar_2d.lua* we enabled the use of imu data:
```LUA
TRAJECTORY_BUILDER_2D.use_imu_data = true
```
### Inside the *head_2d.urdf* file we also added:
```XML
<link name="imu">
<visual>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <geometry>
        <sphere radius="0.02"/>
    </geometry>
    <material name="gray"/>
</visual>
<inertial>
    <mass value="0.1"/>
    <inertia ixx="0.001" ixy="0.000" ixz = "0.000" iyy="0.001" iyz="0.000" izz="0.001"/>
</inertial>
</link>

<joint name="imu_link_joint" type="fixed">
    <parent link="base_link" />
    <child link="imu" />
    <origin xyz="0 0 0" />
</joint>
```

### Now, by launching all the nodes (gbot_core and mpu6050_driver), SLAM with IMU should be working. You can see that by moving the IMU the pose updates, as it is using both Lidar data and IMU data.