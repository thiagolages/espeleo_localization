# espeleo_localization

## Instructions 
- 1 - Install Peter Corke Robotics Toolbox for MATLAB
- 2 - Follow the instructions on: https://www.mathworks.com/matlabcentral/answers/329662-unable-to-access-rosbag-topics . 
The `custom_messages` folder is already within this repo
- 3 - Remember to change the bagfiles path at the beginning of `main.m`
- 4 - At the section tf#3 , I'm loading a tf_aux.mat which contains the transformation between `axis_front_optical_frame` and `base_link`.
If you want, uncomment this line and put your own transformation as a `ROS TransformStamped` message
