# RTABMAP Essentials

This repo will document the essentials to run rtabmap with a realsense camera (D455) on Ubuntu 22.04. At the time of this activity, RTABMAP had only ROS1 docker image and didnt have a ROS2 docker image (at least not well tested). I have an Ubuntu 22.04 (Jammy) and it doesnt support running ROS1. So I decided to run ROS1 versions of the Real sense camera and rtabmap packages in docker containers.  Please follow the below steps to run rtabmap with the setup described.

**Step 1: Buid Docker Image for ROS1 Real Sense**

- Download the Dockerfile provided into a folder say ``/home/<user>/custom_docker_images/realsense_ros``
- From the directory with the Dockerfile build the realsense ros docker image by running the command ``sudo docker build -t realsense_noetic .``


**Step2: Launch the Real Sense Camera Node**

- Run the docker image ``sudo docker run --rm -it -v "/dev:/dev" -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --privileged --network=host realsense_noetic bash``
- Launch the realsense camera node in the container by running ``roslaunch realsense2_camera rs_camera.launch align_depth:=true``

**Step 3: Check RealSense is Working (optional)**

- Open another terminal in the container associated with the realsense camera ny running ``sudo docker exec -it <container_name> bash``
- Run rviz ``rviz rviz``
- Add camera and pointcloud2 to the rviz display
- Change the fixed frame from ``map`` to ``camera_color_frame``
- Assign topics on which the camera and pointcloud2 are listening to for data streams

  You should be able to see both the pointcloud and camera images at this point.

**Step 4: Launch RTABMAP**

- In another terminal, run the rtabmap docker image ``sudo docker run -it --rm --network=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix introlab3it/rtabmap_ros:noetic-latest bash``
- Inspect the topics on which the data streams and info of the RGBD sensor are published
  - Run the command ``gedit /opt/ros/noetic/share/rtabmap_ros/launch/rtabmap.launch`` and inspect the file. The topics need to match what is given below
  ```
    <!-- RGB-D related topics -->
  <arg name="rgb_topic"               default="/camera/color/image_raw" />
  <arg name="depth_topic"             default="/camera/aligned_depth_to_color/image_raw" />
  <arg name="camera_info_topic"       default="/camera/color/camera_info" />
  <arg name="depth_camera_info_topic" default="$(arg camera_info_topic)" />
  ```
  - Edit the topic names in the launch file (will not work when building from source). A better way to do it would be to pass them on as launch arguments as given below.
  
- Launch RTABMAP by running ``roslaunch rtabmap_ros rtabmap.launch rgb_topic:=/camera/color/image_raw depth_topic:=/camera/aligned_depth_to_color/image_raw camera_info_topic:=/camera/color/camera_info``

**Note 1**

- The topic names keep changing as and when newer versions of the real-sense ros node get developed and shipped. You will need to check that the real-sense node publishes data on topics that match what rtabmap subscribes to.
- Depth maps need to be aligned to the rgb images. Hence, the need for the argument ``align_depth:=true`` when launching the real-sense ros node.

**Note 2**

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

## Building From Source

If you want to modify rtabmap (or even put in your own code to better understand rtabmap) you will need to build rtabmap from source. Follow the below steps to build rtabmap from source.

**Step 1:**

- Spin the container from the docker file provided by running ``sudo docker -it blahy blah``
- Run the following commands in the container.

```
cd /rtabmap/build
cmake ..
make -j6
make install
cd ~/catkin_ws
git clone https://github.com/introlab/rtabmap_ros.git src/rtabmap_ros
catkin_make -j4
```

**Step 2:**

- Exit the container and find the name of the container by running ``sudo docker ps -a``.
- Build the image from the updated container by running ``docker commit <container_name> <image_name>``.


Note that putting the set of commands shown in step 1 inside the dockerfile was not working. The docker build seemed to get hung indefinitely when executing the ``cmake ..`` command (processing a particular line item in the CMakelists.txt seemed to be the problem). Hence, the need to run them manually and save the image using docker commit.


**Step 3:**

Spin a container of the saved image by running the command
``sudo docker run -it --rm --network=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix <image_name> bash``

**Step 4 (Optional):**

Follow this step whenever you modify the rtabmap code.

- The rtabmap code resides in ``/rtabmap/app`` and ``/rtabmap/corelib`` folders. Let us for example modify the file ``Rtabmap.cpp`` inside the ``/rtabmap/corelib/src`` folder.
- Open the file in a text editor by running the command ``gedit /rtabmap/corelib/src/Rtabmap.cpp`` and then make your changes, save and close the text editor.
- Build the Rtabmap package by running the following commands (you need to ensure you are in the /rtabmap/build folder before starting to build it by running the cmake commands).

```
cd /rtabmap/build
cmake ..
make -j6
make install
```

**Step 5:**

Navigate into the catkin_ws directory and launch the rtabmap_ros node by running the command ``roslaunch rtabmap_ros rtabmap.launch rgb_topic:=/camera/color/image_raw depth_topic:=/camera/aligned_depth_to_color/image_raw camera_info_topic:=/camera/color/camera_info``

**Note 1** 

The rtabmap libraries get installed in ``/usr/local/`` folder when built from source. The rtabmap github page recommends installing ``ros-ROS_DISTRO-rtabmap*`` as an easy way to resolve dependency issues. However, I found that this resulted in the rtabmap libraries/binaries getting stored in the library folder somewhere inside the ``/opt/ros/noetic/`` folder. When you run the ``catkin_make -j4`` command to build ``rtabmap_ros`` package, catkin was linking against the libraries stored in ``/opt/ros/noetic/``. Hence, avoid the installing ``ros-ROS_DISTRO-rtabmap`` package. In the absence of this package, if you have are using the ``ros-noetic-desktop-full`` package, the only missing package is ``costmap_2d`` which can be resolved by installing the ''ros-ROS_DISTRO-navigation`` package.

**Note 2**

You might want to fork the rtabmap github repo and work off of that repo. Any changes can be pushed into the repo, and pulled/cloned from inside the containers everytime you want to work with the latest code. This will help avoid saving the docker image from containers again and again.


References:
1) https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy
2) https://github.com/introlab/rtabmap_ros/tree/master
3) https://github.com/IntelRealSense/realsense-ros/issues/861
4) https://askubuntu.com/questions/520963/how-come-the-locate-command-doesnt-find-obvious-files
5) https://github.com/introlab/rtabmap/wiki/Kinect-mapping#lostodometry
6) https://www.theconstructsim.com/ros-5-minutes-001-create-ros-catkin-workspace/
