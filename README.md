# HRS Lecture Workspace

# Setup
- copy the https link in the top right under "clone"
- clone with git into local folder: ```$ git clone <address>```
- open with "Visual Studio Code"
- vs code should have the "Remote Development" extension installed
- there should be a pop-up asking if you want to reopen in devcontainer -> click yes
- wait for the devcontainer to be set up (you can click on the log printout)
- open a terminal in vs code and source ROS: ```$ source /opt/ros/kinetic/setup.bash```

# Settings
- change the NAO_IP in the "Dockerfile" to the one your robot tells you 

# Recommendations
- use the ROS extension in vs code for more features
- Catkin Tools are preinstalled so you can build faster with ```$ catkin build``` instead of ```$ catkin-make```
- if you want to install packages or do other permanent changes in your container, test them first in the running devcontainer terminal, and if it works, add the change to the bottom of the "Dockerfile" so that they persist over devvontainer rebuilds

# Setting up Display Output

## Ubuntu
for rendering pass-through to work, you need to tell the xserver on the ubuntu host to accept connections from the devcontainer with: ```$ xhost local:root``` (in a host terminal)

## Windows and Mac Users Requirements
You need a XServer program that is able to host the video output for you. Recommendations:
- Win: https://sourceforge.net/projects/vcxsrv/ 
- Mac: https://www.xquartz.org/ 

## Windows Users
In the "devcontainer.json" do the following:
- comment out the mapping of the X11 temp folder
- set the DISPLAY variable from ```${localEnv:DISPLAY}``` to ```:0```
- disable access control in vcxsrv
