To connect to a ROS Kinetic Master node running inside a Docker container from outside the container on the same computer, you need to set up network communication between the host machine and the Docker container. Here are the steps you can follow:

1. **Expose ROS Master Port:**
   In your Docker container, make sure that the port used by the ROS Master (default is 11311) is exposed. You can do this by either adding the `-p` option when running the container or by specifying it in the Dockerfile.

   Example using the `-p` option:
   ```bash
   docker run -p 11311:11311 your_ros_kinetic_container
   ```

   Example in Dockerfile:
   ```dockerfile
   EXPOSE 11311
   ```

2. **Find the Host IP:**
   Determine the IP address of your host machine. You can use the following command to find the IP address:

   ```bash
   ifconfig
   ```

   Look for the IP address associated with the network interface you are using.

3. **Set ROS Master URI:**
   Set the `ROS_MASTER_URI` environment variable on your ROS Noetic node to point to the ROS Kinetic Master node inside the Docker container. Replace `<HOST_IP>` with the actual IP address of your host machine.

   ```bash
   export ROS_MASTER_URI=http://<HOST_IP>:11311
   ```

   You may want to add this command to your `.bashrc` or a launch script to set the variable automatically.

4. **Check Connection:**
   After setting up the `ROS_MASTER_URI`, you should be able to communicate with the ROS Kinetic Master node from your ROS Noetic node. Test this by running a ROS command, such as:

   ```bash
   rostopic list
   ```

   You should see the list of topics published by the ROS Kinetic Master node.

Keep in mind that both ROS Kinetic and ROS Noetic nodes need to be compatible for proper communication. Ensure that your nodes are using the same ROS version and that any message types being exchanged are compatible between the two versions.

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
