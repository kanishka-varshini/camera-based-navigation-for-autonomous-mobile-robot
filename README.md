# Camera-based Navigation for Autonomous Mobile Robot

## The Problem : 

Autonomous Mobile Robots (AMRs) are becoming increasingly important in various industries, from manufacturing and warehousing to agriculture and urban mobility. These robots must navigate complex environments autonomously, avoiding obstacles, identifying paths, and reaching designated targets without human intervention. Traditional navigation systems often rely on predefined maps or GPS, which can be limiting in dynamic or unstructured environments. To overcome these challenges, vision-based navigation, utilizing image data from cameras, offers a flexible and robust solution.



## The Solution :

We require an autonomous mobile robot that uses image data from cameras to navigate its surroundings.

### Hardware

The hardware would make up a 3 wheeled land based differentially-steered mobile robot. The major components would include the following:

<li>DC motors (with encoders): 2 for the rear wheels.</li>
<li>Motor driver</li>
<li>2 RGB camera modules (to get depth of field)</li>
<li>1 Raspberry Pi 4</li>
<li>1 IMU (MPU 9250)</li>


### Software

The software would involve two systems, the navigation system and the control system.
The navigation system would take in image data from the cameras and determine the required movement. The control system would determine the speed at which each motor should rotate to achieve the desired movement.


<img src="[https://github.com/kanishka-varshini/camera-based-navigation-for-autonomous-mobile-robot/blob/main/Flowchart.png]" alt="Flow Chart"/>
