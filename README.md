# espeleo_localization

## Instructions 
1. Install [Peter Corke Robotics Toolbox for MATLAB](http://petercorke.com/wordpress/toolboxes/robotics-toolbox)
2. Follow the instructions available on [this link](https://www.mathworks.com/matlabcentral/answers/329662-unable-to-access-rosbag-topics), summrized below.
The `custom_messages` folder is already within this repo.
- Download and install [this toolbox](https://www.mathworks.com/matlabcentral/fileexchange/49810-robotics-system-toolbox-interface-for-ros-custom-messages)
 - Create a new folder in a writable location, e.g. "custom_messages" (already done)
 - Execute "rosgenmsg('custom_messages')"
 - Once it finishes successfully, "rosgenmsg" will display three steps (modify javaclasspath, change MATLAB path, restart MATLAB). Please follow these steps.
 - After restarting MATLAB, you should be able to read the rosbag and extract the messages.

3. Remember to change the bagfiles path at the beginning of `main.m`
4. At the section tf#3 , I'm loading a tf_aux.mat which contains the transformation between `axis_front_optical_frame` and `base_link`.
If you want, uncomment this line and put your own transformation as a `ROS TransformStamped` message
