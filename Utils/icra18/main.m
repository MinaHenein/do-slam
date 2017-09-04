%icra18 main
% I- copy to terminal
    % $ rostopic echo /odom >> odom.txt
    % $ rostopic echo /mobile_base/sensors/imu_data >> odom.txt
    % $ rosbag play <rosbag_name.bag>
    
% II- run the matlab script writeOdomMeas.m

% III- in a new terminal
    %   $ cd catkin_ws/
    %   $ catkin_make
    %   $ source devel/setup.bash
    %   $ rosrun depth_extraction extract_depth_images.py 

% IV-
rgbImagesPath =  '/home/mina/workspace/src/Git/do-slam/Utils/icra18/rosbags/images/rosbag1/rgb/';
depthImagesPath =  '/home/mina/workspace/src/Git/do-slam/Utils/icra18/rosbags/images/rosbag1/depth/';
[pointsMeasurements,pointsLabels,pointsTurtlebotID] = ...
    extract3DPoints(rgbImagesPath,depthImagesPath);