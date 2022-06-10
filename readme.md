# Motion Planning for DSC 190 SP22

## Summary

The aim of the package is to return the fastest and most efficient path to overtake objects ahead safely and follow the optimal global raceline. This package was developed for the Winter 2022 edition of the DSC 190 (Intro to Robot Perception &amp; Navigation) class at UC San Diego. The package runs on python3 and is tested on a simulation of the 2.5 mile long Indiana Motor Speedway.

## Overview &amp; Framework

### Inputs

-Raceline - The optimal raceline also called the global raceline is the fastest and most efficient path to follow in a race. This is received as input to the package in a .csv format.

-Opponent Detection - Other race cars on the track are detected by fusing LiDAR depth perception and image recognition models. We get the following parameters for each opponent detected:

- X: x-coord on map
- Y: y-coord on map
- Theta: heading (which way the car is facing), 0.0 = North, 3.14/2 = West, etc.
- Type: should always be &#39;physical&#39; or &#39;car&#39;
- Form: basic shape of the object
- v\_x : velocity

![](RackMultipart20220610-1-ov6p19_html_dabcc1ebd67b7398.png)

Behavior - Behavior is the state of action that a car is currently in or would like to take. There are 4 different types of behavior executable by our package: Left, Right, Straight and Follow. It is calculated using the relative positions and relative velocity of opponent cars.

Output

Using the behavior selection algorithm, a behavior is selected. A trajector\_set with keys corresponding to action and values corresponding to trajectories is returned. The optimal behavior is selected from the trajectory\_set and published.

The following output is achieved, in this case the &#39;follow&#39; behavior was chosen from the trajectory set :

![](sensor_fusion.png)

![](RackMultipart20220610-1-ov6p19_html_a0b5e941f99bf693.png)

INSTALLATION

Any dependencies not already installed can be found in &quot;requirements.txt&quot; and installed in similar fashion as other python libraries. For example &quot;pip install [package name]&quot;

This ROS package can be installed in the following way:

First you should navigate to the source folder of your catkin\_ws or ros\_ws.

For example: cd ~/ros\_ws/src.

Run:

git clone [alexmak001/motion-path-planning-ros2 (github.com)](https://github.com/alexmak001/motion-path-planning-ros2)

cd ..

colcon build

The motion planning package should be now installed to your computer and you will be able to use it after sourcing your workspace.

source /opt/ros/foxy/setup.bash

You can run the motion planning package by running:

ros2 launch motion\_plan\_pkg motion\_plan\_launch\_file.launch.py

ROS API

Subscribed Topics

TrackedObjects(sensor\_msgs/tracked\_objects) - This would have been the custom sensor fusion topic we were going to subscribe to. For independent testing and usage purposes, our package uses dummy sensor data

Published Topics

This node can publish a variety of topics but the final configuration depends on the user. By default the majority of the topics are disabled and they should be enabled through the launch file configuration.

datmo/marker\_array (visualization\_msgs/MarkerArray) - In this topic a variety of Rviz markers are published, which can facilitate in understanding the inner workings of the program.

datmo/box\_kf (datmo/TrackArray) - In this topic the output of a Kalman Filter with a Constant Velocity model, which tracks the center of the box that surrounds the clusters is published.\

Note: In case that the marker\_array topic is published from a robot and visualized in computer, which has a different version of ROS installed (kinetic, melodic, ...), the msgs will not be published and the datmo node will crash with an md5sum error. To mitigate this, you should install on your robot the visualization msgs package of the ROS installation that runs on your computer.

Custom Messages

This package uses two custom msgs types datmo/Track and datmo/TrackArray to facilitate the publishing of its results. To my knowledge, at the time of developement, there was no standard ROS messages that accomplishes the same task.

The datmo/Track message has the following structure:

int32 id - object ID, so it is possible to differentiate between different objects during tracking

float32 length - Estimated length of the object

float32 width Estimated width of the object

nav\_msgs/Odometry odom - Estimated pose of the object

The datmo/TrackArray message is an array that contains multiple datmo/Track messages, with the goal of efficient publishing.

Rviz markers

In case that the pub\_markers flag is set to true, this package publishes visualization messages, which can be displayed in Rviz. The following messages are published:

closest\_corner - The closest corner point of surrounding vehicles is visualized with a black rectangle.

bounding\_box\_center - The center of the bounding box is visualized with a yellow rectangle.

velocities - The velocities of the tracked objects are represented with an arrow.\

PARAMETERS

&quot;lidar\_frame&quot; (&quot;string&quot;, default: &quot;laser&quot;) - Name of the transformation frame (frame\_id) of the LaserScan msgs

&quot;world\_frame&quot; (&quot;string&quot;, default: &quot;world&quot;) - Name of the world coordinate frame, if it is not available, this value can be set equal to the lidar\_frame

&quot;threshold\_distance&quot; (&quot;double&quot;, default: &quot;0.2&quot;) - This value sets the distance that is used by the clustering algorithm

&quot;euclidean\_distance&quot; (&quot;double&quot;, default: &quot;0.25&quot;) - This value sets the distance that is used by the euclidean distasnce data association algorithm

&quot;pub\_markers&quot; (&quot;bool&quot;, default: &quot;false&quot;) - publish of the the vizualization markers

CHALLENGES FACED

One of the major challenges that we faced as a group was communication. It was sometimes a difficult task to share what you worked on or what you were able to accomplish with other team members and teams. We were sometimes only able to truly share information when we met in person, because remote meetings were not as effective. Also many people had to miss class due to COVID and made working in a group slightly more of a challenge, but was not a major issue.

Another challenge that we faced relating to communication was having specification and final deliverable changes. In the last week of the project, we decided that we would only combine with only two other teams. This forced us to be flexible, but also having to spend a lot of time in the lab in order to get our project working.

The most crucial part of this project was the selection of which algorithm we were going to use. We needed to find one that takes in the right format for the data and outputs in the format we need. It also has to be very efficient and quick to do calculations on the fly. After looking through a few, we found the one that we believed would work best and attempted to implement it in ROS2. However, the algorithm was huge and had a lot of moving parts. It took a while to figure out how it works, what we need from it, and how to implement it in ROS2.

The last major challenge we faced was not having a great way to test until the very last week of the project. Throughout the implementation, we could only debug with print statements to see that there is information flowing, but we did not know if it was correct. When we finally set up the simulation in the lab, we were able to visualize our outputs, but they were not the correct ones. Also, we found out that the sample data we had been using for our algorithm was not reflective of what we will be receiving during the race, so we had to figure out how to quickly format them.

Development

  - Week 6
    - Began exploring possible options for motion plan algorithms.
  - Week 7
    - Found motion plan algorithm and determined its inputs/outputs
    - Started figuring out ROS2 integration
  - Week 8
    - Implementing our algorithm into ROS2 and setting up subscriber and publisher nodes
  - Week 9
    - Integrating with other teams
  - Week 10
    - Debugging and testing in a simulation
