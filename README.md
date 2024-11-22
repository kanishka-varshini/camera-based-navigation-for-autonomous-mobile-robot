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


### Software

The software would involve two systems, the navigation system and the control system.
The navigation system would take in raw image data from the cameras and determine the required movement. It converts the dual image data into a depth field and performs SLAM. The generated map is then used to perform path planning. The control system would determine the speed at which each motor should rotate to achieve the desired movement and follow the planned path.

<img src="https://github.com/kanishka-varshini/camera-based-navigation-for-autonomous-mobile-robot/blob/main/images/flowchart_final.png" alt="Flow Chart"/>
<img src="https://github.com/kanishka-varshini/camera-based-navigation-for-autonomous-mobile-robot/blob/main/images/mechatronic_architecture.png" alt="Mechatronic Architecture"/>

## Depth map generation :

### Stereo block matching
Local Optimization for each block.

Suitable for SLAM purposes due to faster speed of computation.

<img src="https://github.com/kanishka-varshini/camera-based-navigation-for-autonomous-mobile-robot/blob/main/stereobm.png" alt="StereoBM output"/>

### Stereo semi-global block matching
Global constraints give smoothness and coherence.

Computationally expensive.

<img src="https://github.com/kanishka-varshini/camera-based-navigation-for-autonomous-mobile-robot/blob/main/stereosgbm.png" alt="StereoSGBM output"/>

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

* Having the bot move in a straight line with static obstacles placed along the path.
* Given a global path, the bot should be able to avoid any stationary obstacle while sticking to the path.
* Dynamic obstacles.



## Obstacle Avoidance
For testing method 1, obstacle avoidance using A*:

<img src="https://github.com/kanishka-varshini/camera-based-navigation-for-autonomous-mobile-robot/blob/main/github_1.jpg"/>
<img src="https://github.com/kanishka-varshini/camera-based-navigation-for-autonomous-mobile-robot/blob/main/github_3.png"/>

Other methods include-DWA, VFH, APF, RRT, SLAM-based, Costmap-based. 
VHF and DWA are both suitable for stereo cameras and dynamic environment.

## Results
The AMR was able to generate a noise-free and accurate disparity map, and generate A* paths around the obstacles.


* Demonstration: <img src="https://github.com/kanishka-varshini/camera-based-navigation-for-autonomous-mobile-robot/blob/main/images/demo1.png"/>
  
## Future Scope
* Incorporation of Simultaneous Localization and Mapping (SLAM): For improved global navigation and mapping capabilities. The RPi on its own wasn't able to run SLAM as its computational power wasn't enough.
* Developing on ROS2: Enables easier communication between different modules of the stereo vision. Can make laptop-RPi communications simpler. The pre-eisting libraries for SLAM, navigationa and control are robust and also allow for any modifications if needed.
* DWA: https://github.com/estshorter/dwa/blob/master/dwa.py
* Path planning algorithms performance comparison : https://www.frontiersin.org/journals/neurorobotics/articles/10.3389/fnbot.2020.00063/full 

## References

* Stereo- rectification and disparity map generation: https://learnopencv.com/making-a-low-cost-stereo-camera-using-opencv/
* Stereo algorithms: https://www.cse.psu.edu/~rtc12/CSE486/lecture09.pdf

All codes were edited, integrated and debugged with the help of ChatGPT. 


