# NAO U NO
![.](https://github.com/davidmartinez13/Nao_ws/Screenshots/NAO_UNO.png)
This repository contains code for integrating a NAO robot into an UNO card game environment using ROS (Robot Operating System). The system enables the robot to recognize UNO cards and interact with the game accordingly.
# How to run this code:
1. Clone the repository to your local machine:
   ```
   git clone https://github.com/davidmartinez13/Nao_ws
   cd Nao_ws
   ```

2. Initialize the NAO robot by sourcing the setup file in every terminal:
   ```
   source devel/setup.bash
   ```

3. Launch the necessary ROS nodes in different terminals:
   - Launch NAO robot:
     ```
     roslaunch nao_bringup nao_full_py.launch
     ```
   - Launch tactile sensors:
     ```
     roslaunch nao_apps tactile.launch 
     ```
   - Launch speech capabilities:
     ```
     roslaunch nao_apps speech.launch
     ```

4. Initialize the vision system:
   - Navigate to the vision module:
     ```
     cd mmdetection
     ```
   - Run the card detection script. The weights can be downloaded from [here](https://drive.google.com/file/d/1Wq8c8nYg18KqsWm90MILd3BvicDumCY_/view?usp=drive_link): .Adjust paths for model weights if necessary:
     ```
     python3 demo/nao_card_detection.py /configs/yolox/yolox_s_8xb8-300e_coco_UNO.py /work_dirs/yolox_s_8xb8-300e_coco_UNO/20240124_013947/epoch_50.pth
     ```

5. Initialize the UNO playing engine:
   ```
   rosrun pick_place_card uno-engine-motion.py
   ```

### Usage
- Ensure that the NAO robot is properly connected and configured with ROS.
- Adjust paths and configurations as needed, especially for model weights and ROS launch files.
- Monitor the terminal outputs for any errors or warnings during initialization.
- Interact with the robot and observe its behavior during UNO gameplay.

# Container Setup
- open with "Visual Studio Code"
- vs code should have the "Remote Development" extension installed
- there should be a pop-up asking if you want to reopen in devcontainer -> click yes
- wait for the devcontainer to be set up (you can click on the log printout)
- open a terminal in vs code and source ROS: ```$ source /opt/ros/kinetic/setup.bash```

## Settings
- change the NAO_IP in the "Dockerfile" to the one your robot tells you 

## Recommendations
- use the ROS extension in vs code for more features
- Catkin Tools are preinstalled so you can build faster with ```$ catkin build``` instead of ```$ catkin-make```
- if you want to install packages or do other permanent changes in your container, test them first in the running devcontainer terminal, and if it works, add the change to the bottom of the "Dockerfile" so that they persist over devvontainer rebuilds

## Setting up Display Output

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

# Network setup
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


### Additional Notes
- The system may require calibration and adjustments based on the specific environment and hardware configurations.
- Refer to the official NAO robot documentation and ROS documentation for detailed information on setup and configuration.

### Contributors
For any questions or issues, please contact the project contributors. Thank you for using the UNO Robot Integration system!

- Maria Luna Ghanime @marialunaghanime
- David S. Martinez @davidmartinez13
- Jorge Villasante Meza @jorgevillasante
- Efe Oztufan @eeffee

