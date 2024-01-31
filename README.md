# RTABMAP Essentials

This repo will document the essentials to run rtabmap with a realsense camera (D455) on Ubuntu 22.04. At the time of this activity, RTABMAP had only ROS1 docker image and didnt have a ROS2 docker image (at least not well tested). I have an Ubuntu 22.04 (Jammy) and it doesnt support running ROS1. So I decided to run ROS1 versions of the Real sense camera and rtabmap packages in docker containers.  Please follow the below steps to run rtabmap with the setup described.

**Step 1: Buid Docker Image for ROS1 Real Sense**

- Download the Dockerfile provided into a folder say ``/home/<user>/custom_docker_images/realsense_ros``
- From the directory with the Dockerfile build the realsense ros docker image by running the command ``sudo docker build -t realsense_noetic .``


**Step2: Launch the Real Sense Camera Node**

- Run the docker image ``sudo docker run --rm -it -v "/dev:/dev" -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --privileged --network=host realsense_noetic bash``
- Launch the realsense camera node in the container by running ``roslaunch realsense2_camera rs_camera.launch align_depth:=true``

**Step 3: Check RealSense is Working**

- Open another terminal in the container associated with the realsense camera ny running ``sudo docker exec -it <container_name> bash``
- Run rviz ``rviz rviz``
- Add camera and pointcloud2 to the rviz display
- Change the fixed frame from ``map`` to ``camera_color_frame``
- Assign topics on which the camera and pointcloud2 are listening to for data streams

  You should be able to see both the pointcloud and camera images at this point.

**Step 4: Launch RTABMAP**

- In another terminal, run the rtabmap docker image ``sudo docker run -it --rm --network=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix introlab3it/rtabmap_ros:noetic-latest bash``
- Edit the topics on which the data stream and info of the RGBD sensor is published
  - Run the command ``gedit /opt/ros/noetic/share/rtabmap_ros/launch/rtabmap.launch`` and inspect the file and make sure the topics matches what is given below
  ```
    <!-- RGB-D related topics -->
  <arg name="rgb_topic"               default="/camera/color/image_raw" />
  <arg name="depth_topic"             default="/camera/aligned_depth_to_color/image_raw" />
  <arg name="camera_info_topic"       default="/camera/color/camera_info" />
  <arg name="depth_camera_info_topic" default="$(arg camera_info_topic)" />
  ```
- Launch RTABMAP by running ``roslaunch rtabmap_ros rtabmap.launch``

**Note**

Hold the camera as still as you can in the air before launching the rtabmap node. Once, launched move the camera around slowly to build the map. Successful map building would show up similar to the below text
```
[ INFO] [1706713057.374798269]: Odom: quality=161, std dev=0.003923m|0.019480rad, update time=0.123411s
[ INFO] [1706713057.462672670]: Odom: quality=105, std dev=0.004594m|0.017144rad, update time=0.085584s
[ INFO] [1706713057.575339302]: Odom: quality=191, std dev=0.001772m|0.014651rad, update time=0.110076s
[ INFO] [1706713057.663582753]: Odom: quality=230, std dev=0.001890m|0.022044rad, update time=0.085466s
[ INFO] [1706713057.757163689]: Odom: quality=275, std dev=0.001224m|0.013634rad, update time=0.087272s
[ INFO] [1706713057.841264953]: Odom: quality=231, std dev=0.004162m|0.017944rad, update time=0.078874s
[ INFO] [1706713057.953144521]: Odom: quality=173, std dev=0.002362m|0.016542rad, update time=0.108902s
[ INFO] [1706713058.036762876]: Odom: quality=241, std dev=0.002255m|0.013634rad, update time=0.079593s
[ INFO] [1706713058.119652165]: Odom: quality=279, std dev=0.002755m|0.018188rad, update time=0.077965s
[ INFO] [1706713058.235901496]: Odom: quality=278, std dev=0.001870m|0.013634rad, update time=0.111718s
[ INFO] [1706713058.328899046]: Odom: quality=291, std dev=0.002175m|0.013634rad, update time=0.087778s
```

Also, the rtabmap_viz visualizer will show the 3D map getting built against a dark/black background. When odometry is lost (due to fast camera movement etc.) the background turns red in 3D map portion of the visualizer. The text will look similar to what is give below
```
[ WARN] [1706713063.312298277]: Could not get transform from odom to camera_link after 0.200000 seconds (for stamp=1706713062.592324)! Error="Lookup would require extrapolation 0.499859572s into the future.  Requested time 1706713062.592323542 but the latest data is at time 1706713062.092463970, when looking up transform from frame [camera_link] to frame [odom]. canTransform returned after 0.201453 timeout was 0.2.".
[ WARN] (2024-01-31 14:57:43.392) OdometryF2M.cpp:562::computeTransform() Registration failed: "Not enough inliers 0/20 (matches=54) between -1 and 624"
[ INFO] [1706713063.394758067]: Odom: quality=0, std dev=0.000000m|0.000000rad, update time=0.097652s
[ WARN] (2024-01-31 14:57:43.503) OdometryF2M.cpp:562::computeTransform() Registration failed: "Not enough inliers 0/20 (matches=52) between -1 and 625"
[ INFO] [1706713063.505905196]: Odom: quality=0, std dev=0.000000m|0.000000rad, update time=0.108904s
[ERROR] (2024-01-31 14:57:43.513) Rtabmap.cpp:1348::process() RGB-D SLAM mode is enabled, memory is incremental but no odometry is provided. Image 3436 is ignored!
[ INFO] [1706713063.513574266]: rtabmap (129): Rate=1.00s, Limit=0.000s, Conversion=0.0032s, RTAB-Map=0.0001s, Maps update=0.0000s pub=0.0000s (local map=30, WM=42)
[ WARN] [1706713063.544015055]: Could not get transform from odom to camera_link after 0.200000 seconds (for stamp=1706713062.692291)! Error="Lookup would require extrapolation 0.599826575s into the future.  Requested time 1706713062.692290545 but the latest data is at time 1706713062.092463970, when looking up transform from frame [camera_link] to frame [odom]. canTransform returned after 0.201584 timeout was 0.2.".
[ WARN] (2024-01-31 14:57:43.608) OdometryF2M.cpp:562::computeTransform() Registration failed: "Not enough inliers 0/20 (matches=50) between -1 and 626"
```


  
