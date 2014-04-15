lab2
====
* To install do:
* cd ~/catkin_ws/src/
* git clone https://github.com/aliunwala/lab2.git
* cd ~/catkin_ws/
* catkin_make


* To run do:
* plug in a kinect                       // To both the wall jack and the computer. Alternatively just use one of the segbots. depending on what method you use the topic name changes. It defaults to the on on the robot kinect.
* roslaunch segbot_navigation navigation.launch - real robot
* OR
* roslaunch openni_launch openni.launch  // starts publishing kinect data- not real robot just labtop with kinect connected.
* rosrun lab2 kinect_print_to_screen     // takes the weirdness of PointCloud2 and publishes an PointXYZ instead. Which is much easier to reason about.
