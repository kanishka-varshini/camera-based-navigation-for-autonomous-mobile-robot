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


### Software

The software would involve two systems, the navigation system and the control system.
The navigation system would take in raw image data from the cameras and determine the required movement. It converts the dual image data into a depth field and performs SLAM. The generated map is then used to perform path planning. The control system would determine the speed at which each motor should rotate to achieve the desired movement and follow the planned path.


<img src="https://github.com/kanishka-varshini/camera-based-navigation-for-autonomous-mobile-robot/blob/main/Flowchart.png" alt="Flow Chart"/>
<img src="https://github.com/kanishka-varshini/camera-based-navigation-for-autonomous-mobile-robot/blob/main/IMG-20240821-WA0013.jpg" alt="Rough Sketch"/>


## Testing Method :

Upon given a destination, the AMR must navigate through stationary and moving obstacles to reach the destination.

To test the system’s performance, path planning efficiency, object detection accuracy, and obstacle avoidance effectiveness will be analysed. This will be done by conducting multiple trials on the hardware using different algorithms to arrive at the most efficient one.
