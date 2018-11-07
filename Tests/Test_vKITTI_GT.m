%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 30/10/2018
% Testing vKITTI dataset - Sequence0001 frames 00334-00426
%--------------------------------------------------------------------------
% Test 1 - rgb and depth images
%  1.1 Project features on an image into 3D, then project onto next frame, and
%  then project back into 3D and to the previous frame. 
%--------------------------------------------------------------------------
% Test 2 - camera intrinsic and extrinsic parameters
%  2.1 Generate 3D point cloud.
%--------------------------------------------------------------------------
% Test 3 - object poses
%  3.1 Track features on dynamic objects.
%  3.2 Plot object motion.
%--------------------------------------------------------------------------
%% 1.1
%setup
tol = 1e-10;
dir = '/home/mina/Downloads/vKitti/';
display = 1;
i = randi([334 426],1); %random image index
cameraExtrinsicsFile = strcat(dir,'vkitti_1.3.1_extrinsicsgt/0001_clone.txt');
% camera intrinsics
K = [725,      0,     620.5;
       0,    725,     187.0;
       0,      0,        1];
% read rgb image
rgbI = imread(strcat(dir,'rgbImages/00',num2str(i),'.png'));
% read depth image
depthI = imread(strcat(dir,'depthImages/00',num2str(i),'.png'));
% simple check of a close object and a far object
lowestDepth = min(min(depthI));
[lowestDepthPixelRow,lowestDepthPixelCol]= find(depthI==lowestDepth);
highestDepth = max(max(depthI)); % sky
[highestDepthPixelRow,highestDepthPixelCol]= find(depthI==highestDepth);
assert(highestDepth>lowestDepth)
%% First thing to note:
%% Matrix coordinate (row,col) --> Image coordinate (col,row) 
if display
    figure(1);
    imshow(rgbI)
    hold on
    scatter(lowestDepthPixelCol,lowestDepthPixelRow,'filled','ro','LineWidth',10);
    hold on
    scatter(highestDepthPixelCol,highestDepthPixelRow,'bx','LineWidth',2);
    hold off
end
% random feature selection
pixelRow = randi([1 size(rgbI,1)],1);
pixelCol = randi([1 size(rgbI,2)],1);
pixelDepth = double(depthI(pixelRow,pixelCol));
if display
   figure(2);
   imshow(rgbI)
   hold on
   scatter(pixelCol,pixelRow,'filled','bo','LineWidth',10);
   hold off
end
% project into 3D
% read camera poses
fid = fopen(cameraExtrinsicsFile);
lineCell = textscan(fid,'%s',1,'delimiter','\n','headerlines',i+1);
fclose(fid);
lineArray = str2num(cell2mat(lineCell{1,1}));
assert(lineArray(1)==i);
cameraPoseMatrix = inv(reshape(lineArray(2:end),[4,4])');
fid = fopen(cameraExtrinsicsFile);
lineCell = textscan(fid,'%s',1,'delimiter','\n','headerlines',i+2);
fclose(fid);
lineArray = str2num(cell2mat(lineCell{1,1}));
assert(lineArray(1)==i+1);
nextCameraPoseMatrix = inv(reshape(lineArray(2:end),[4,4])');
    % image--> camera
    camera3DPoint = K\[pixelRow;pixelCol;1];
    camera3DPoint(3) = pixelDepth/100;
    % camera --> world
    world3DPoint = cameraPoseMatrix * [camera3DPoint;1];
% project onto next frame
    % world --> camera
    nextCamera3DPoint = nextCameraPoseMatrix\world3DPoint;
    nextCamera3DPoint = nextCamera3DPoint(1:3,1);
    % camera --> image
    nextImagePoint = K * nextCamera3DPoint;
% project into 3D
    % image--> camera
    nextCamera3DPoint2 = K\nextImagePoint;
    % camera --> world
    world3DPoint2 = nextCameraPoseMatrix * [nextCamera3DPoint2;1];
% assert equality within tolerance 
assert(all(abs(nextCamera3DPoint-nextCamera3DPoint2)<tol));
assert(all(abs(world3DPoint-world3DPoint2)<tol));
% project onto last frame
    % world --> camera
     lastCamera3DPoint = cameraPoseMatrix\world3DPoint2;
     lastCamera3DPoint = lastCamera3DPoint(1:3,1);
    % camera --> image
    lastImagePointDepth = lastCamera3DPoint(3);
    lastImagePoint = K * [lastCamera3DPoint(1);lastCamera3DPoint(2);1];
% assert equality within tolerance 
assert(all(abs(camera3DPoint-lastCamera3DPoint)<tol))    
assert(all(abs(round(lastImagePoint(1:2,1))-[pixelRow;pixelCol])<tol))
assert(abs(lastImagePointDepth-pixelDepth/100)<tol)

if display
   figure(3);
   imshow(rgbI)
   hold on
   scatter(lastImagePoint(2),lastImagePoint(1),'rx','LineWidth',4);
   hold off
end

%% 2.1 Check the vKITTIPointCloudPlot.m file in /Utils/Plotting
vKittiPointCloudPlot(i);
%% remarks:
    % pixelRow = i; 1:imageHeight
    % pixelCol = j; 1:imageWidth
    % camera3DPoint = K\[pixelRow;pixelCol;1];
    % 1. need to invert the 2nd element of camera3DPoint for a correct view
        % camera3DPoint(2) = -camera3DPoint(2);
    % 2. provided depth is the z coordinate of each pixel in camera coordinate space
        % camera3DPoint(3) = pixelDepth/100;
    % 3. provided camera extrinsics is transfomartion from camera to the
    % world so needs to be inverted  to get world to camera transfomation
        % cameraPoseMatrix = inv(cameraExtrinsics);
        % world3DPoint = cameraPoseMatrix * [camera3DPoint;1];
%% 3.1 Check Test_projectMovingObjectPoints_vKITTI.m in /Tests
%% 3.2 Check plotObjectPoses_vKITTI.m in /Utils/Plotting
imageRange = 334:426;
plotObjectPoses_vKITTI(imageRange)