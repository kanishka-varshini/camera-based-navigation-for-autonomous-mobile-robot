# Camera-based Navigation for Autonomous Mobile Robot

## The Problem : 

Autonomous Mobile Robots (AMRs) are becoming increasingly important in various industries, from manufacturing and warehousing to agriculture and urban mobility. These robots must navigate complex environments autonomously, avoiding obstacles, identifying paths, and reaching designated targets without human intervention. Traditional navigation systems often rely on predefined maps or GPS, which can be limiting in dynamic or unstructured environments. To overcome these challenges, vision-based navigation, utilizing image data from cameras, offers a flexible and robust solution.



## The Solution :

We require an autonomous mobile robot that uses image data from cameras to navigate its surroundings.

### Functionalities

The mobile robot would be able to perform the following:
<li>Path Planning</li>
<li>Object Detection</li>
<li>Obstacle Avoidance</li>


### Hardware

The hardware would make up a 3 wheeled land based differentially-steered mobile robot. The major components would include the following:

<li>Servo motors: 2 for the rear wheels</li>
<li>LiPo Battery: 1</li>
<li>Motor driver: 1</li>
<li>RGB camera modules: 2 to get depth of field</li>
<li>Raspberry Pi 4: 1</li>
<li>IMU (MPU 9250): 1</li>
https://docs.google.com/spreadsheets/d/1V6J0Lie2oz4PbgF_GIoCCh01dERQ7iK8KUERm-KVPIc/edit?gid=0#gid=0


### Software

The software would involve two systems, the navigation system and the control system.
The navigation system would take in raw image data from the cameras and determine the required movement. It converts the dual image data into a depth field and performs SLAM. The generated map is then used to perform path planning. The control system would determine the speed at which each motor should rotate to achieve the desired movement and follow the planned path.

SLAM reference paper : http://arxiv.org/pdf/1911.04063
Path planning algorithms performance comparison : https://www.frontiersin.org/journals/neurorobotics/articles/10.3389/fnbot.2020.00063/full 

<img src="https://github.com/kanishka-varshini/camera-based-navigation-for-autonomous-mobile-robot/blob/main/AMR.png" alt="Flow Chart"/>
<img src="https://github.com/kanishka-varshini/camera-based-navigation-for-autonomous-mobile-robot/blob/main/Components.png" alt="Rough Sketch"/>

## Depth map generation :

### Stereo block matching
Local Optimization for each block.

Suitable for SLAM purposes due to faster speed of computation.

<img src="https://github.com/kanishka-varshini/camera-based-navigation-for-autonomous-mobile-robot/blob/main/stereobm.png" alt="StereoBM output"/>

### Stereo semi-global block matching
Global constraints give smoothness and coherence.

Computationally expensive.

<img src="https://github.com/kanishka-varshini/camera-based-navigation-for-autonomous-mobile-robot/blob/main/stereosgbm.png" alt="StereoSGBM output"/>


* Stereo algorithms: https://www.cse.psu.edu/~rtc12/CSE486/lecture09.pdf


### Monocular SLAM
Slow processing.

## Depth map to Laser Scan:

Depth map-
<img src="https://github.com/kanishka-varshini/camera-based-navigation-for-autonomous-mobile-robot/blob/main/depthmap.png" alt="depth map"/>

Converting to laser scan at 3/4th of the height of the depth image generated-
<img src="https://github.com/kanishka-varshini/camera-based-navigation-for-autonomous-mobile-robot/blob/main/laserscan%20polar%20plot.png" alt="laser scan"/>


## Testing Method :

Upon given a destination, the AMR must navigate through stationary and moving obstacles to reach the destination.

To test the system’s performance, path planning efficiency, object detection accuracy, and obstacle avoidance effectiveness will be analysed. This will be done by conducting multiple trials on the hardware using different algorithms to arrive at the most efficient one.

1-Having the bot move in a straight line with obstacles placed along the path. (static and then, dynamic added)
2-Given a global path, the bot should be able to avoid any stationary obstacle while sticking to the path.
3-Dynamic obstacles.


## Discussions :

Try weighted average of different algorithms.


## Updates:
Trying to figure out the OS on which final development needs to be done. Due to the fact that i am using two Raspberry Pi Camera modules im restricted to use raspberry pi OS which supports the underlying libcamera module libraries that are ONLY available in Pi OS, so i cant use Ubuntu through which i could use a lot of functions/modules/ROS workbench that others have developed, but im forced to develop all this in Raspberry pi OS now. 


## Local path planning
For testing method 1, obstacle avoidance using a follow the gap method is used.
Other methods include-DWA, VFH, APF, RRT, SLAM-based, Costmap-based. 
VHF and DWA are both suitable for stereo cameras, for complex environments-RRT or SLAM-based. 
Things that can be done- Comparison between these methods. Measure deviation from straight path under the same conditions and positions of obstacle.


