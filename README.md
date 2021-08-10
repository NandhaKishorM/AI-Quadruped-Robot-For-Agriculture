# AI-Quadruped-Robot-For-Agriculture
## Problem statement
Massive investment on agro-chemicals from the farmers’ end on crops for protection against crop health issues, diseases and pest attacks, leading to high chemical residue content in the final produce, thus reducing the quality of the produce, productivity and polluting the farm-environment.

## Solution
For the farmers cultivating crops, an **OAK-D enabled quadruped robot** with **robotic arm** to effectively spray **water/pesticides/insecticides** on the crops by identifying the **crop or the pests**, thus saving farmers investment on agrochemicals and water, utilizing the depth capability to detect the pest and analyze the pest infection in the process.

# Software Installation - Agribot
We followed the GitHub repo by Mike4192 for this version. Go to this [link](https://github.com/mike4192/spotMicro) to learn more.
The default implementation is on **Raspberry Pi 3 Model B and ROS kinetic**, since we used **Nvidia Jetson Nano** we invested some time on migrating to **ROS melodic**.
After installing ROS melodic on Nvidia Jetson Nano, run,
```sh
git clone https://github.com/kishorkuttan/AI-Quadruped-Robot-For-Agriculture.git
source /opt/ros/kinetic/setup.bash 
mkdir -p ~/catkin_temp  
cp -a /path/to/AI-Quadruped-Robot-For-Agriculture/src/ ~/catkin_temp/
catkin build
source devel/setup.bash
```
If you find errors during catkin build install necessary libraries or directly copy the libraries to **/src/** folder and re run catkin_build

## 3D prints
### Robot
Go to this [link](https://www.thingiverse.com/thing:3445283) for spot micro 3D print files. We made slight modification due to stability and structural issues in the default print.

### Arm
The files are under the folder /Arm 3D print stl files/

## Connection
![at text](https://github.com/kishorkuttan/AI-Quadruped-Robot-For-Agriculture/blob/master/schematics.png)
## The Robotic Arm And Sprayer System
Connect as per the diagram and control the Raspberry Pi Zero W through VNC and run,
```sh
python arm_and_spray.py
```
In this script it calculates angles for servo motors using IK and also starts the DC motor pump after getting the x,y,z location data from OAK-D.

<p float="left">
  <img src="/arm_doc/1622900229364.jpeg" width="480" />
  <img src="/arm_doc/1622899821254.jpeg" width="480" /> 
  <img src="/arm_doc/1620652142487.jpeg" width="480" />
    <img src="/arm_doc/inverse_kinematics.jpg" width="480" />
</p>


### Test OAK-D
connect the OAK-D to the host
```sh
python spatial_mobilenet.py

```
## Demo

https://user-images.githubusercontent.com/48623612/127535646-96b2c777-e888-43cb-af1a-77d663f8487b.mp4



https://user-images.githubusercontent.com/48623612/127535906-9ffb2053-86a3-4153-b496-3118d2ea1060.mp4



https://user-images.githubusercontent.com/48623612/128687662-2845c66a-45d7-49c7-8cf9-19cea187af5c.mp4

### Pest detection
```sh
python spatial_mobilenet_custom.py
```


https://user-images.githubusercontent.com/48623612/128688979-6633d1d9-b441-4c8c-9f94-d753bce6091c.mp4


## Inbuilt OAK-D IMU utilization
Run

```sh
python gyro visualization
```


https://user-images.githubusercontent.com/48623612/127535856-a1a527a5-7e4c-46dc-a450-e32995b7074e.mp4



