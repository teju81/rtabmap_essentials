# RTABMAP Essentials

This repo will document the essentials to run rtabmap with a realsense camera (D455) on Ubuntu 22.04. Ubuntu 22.04 (Jammy) doesnt support running ROS1. So you will need to use docker to run the algorithm.  Please follow the below steps to run the algorithm.

- Download the Dockerfile provided into a folder say ``/home/<user>/custom_docker_images/realsense_ros``
- From the directory with the Dockerfile build the realsense ros docker image by running the command ``sudo docker build -t realsense_noetic .``
- Run the docker image ``sudo docker run --rm -it -v "/dev:/dev" --privileged --network=host realsense_noetic bash``
- Launch the realsense camera node in the container by running ``roslaunch realsense2_camera rs_camera.launch``
- In another terminal, run the rtabmap docker image ``sudo docker run -it --rm --network=host -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix introlab3it/rtabmap_ros:noetic-latest bash``
