%icra18 main
% I- copy to terminal
    % $ rostopic echo /odom >> odom.txt
    % $ rostopic echo /mobile_base/sensors/imu_data >> odom.txt
    % $ rosbag play <rosbag_name.bag>
    
% II- run the matlab script writeOdomMeas.m

% III- in a new terminal
    % $ cd catkin_ws/
    % $ catkin_make
    % $ source devel/setup.bash
    % $ rosrun depth_extraction extract_depth_images.py
    
% III- in a different terminal
    % $ roscore
    % $ rosbag play <rosbag_name.bag>

% IV-
rgbImagesPath =  '/home/mina/Downloads/icra18/images/rgb/';
depthImagesPath =  '/home/mina/Downloads//icra18/images/depth/';
[pointsMeasurements,pointsLabels,pointsTurtlebotID] = ...
    extract3DPoints(rgbImagesPath,depthImagesPath);