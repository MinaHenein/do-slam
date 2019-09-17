K = [7.215377000000e+02 0.000000000000e+00 6.095593000000e+02;
     0.000000000000e+00 7.215377000000e+02 1.728540000000e+02;
     0.000000000000e+00 0.000000000000e+00 1.000000000000e+00];

sequence = '0003'; 
dir = '/media/mina/ACRV Samsung SSD T5/KITTI dataset/';
rgbDir = 'tracking/data_tracking_image_2/training/image_02/';
depthDir = 'tracking/depth/';
objSegDir = 'mots/instances/';  
 

maskI1 = imread(strcat(dir,objSegDir,sequence,'/000000.png'));
maskI25 = imread(strcat(dir,objSegDir,sequence,'/000025.png'));
maskI27 = imread(strcat(dir,objSegDir,sequence,'/000027.png'));
depthI1 = imread(strcat(dir,depthDir,sequence,'/000000.png'));
depthI25 = imread(strcat(dir,depthDir,sequence,'/000025.png'));
depthI27 = imread(strcat(dir,depthDir,sequence,'/000027.png'));
rgbI1 = imread(strcat(dir,rgbDir,sequence,'/000000.png'));
rgbI25 = imread(strcat(dir,rgbDir,sequence,'/000025.png'));
rgbI27 = imread(strcat(dir,rgbDir,sequence,'/000027.png'));

resultFilePath = '/home/mina/workspace/src/Git/gtsam/Data/GraphFiles/vk-Mina/kitti-0003-0-40_result.graph';
fileID = fopen(resultFilePath,'r');
Data = textscan(fileID,'%s','delimiter','\n','whitespace',' ');
resultCStr = Data{1};
fclose(fileID);
resultMotions = [];
resultPoses = [];
for i = 1:size(resultCStr,1)
    line = strsplit(resultCStr{i},' ');
    if strcmp(line{1},'VERTEX_SE3Motion')
        resultMotions = [resultMotions, [str2double(line{3}); str2double(line{4}); str2double(line{5});...
            str2double(line{6}); str2double(line{7}); str2double(line{8})]];
    elseif strcmp(line{1},'VERTEX_POSE_R3_SO3')
        resultPoses = [resultPoses, [str2double(line{3}); str2double(line{4}); str2double(line{5});...
            str2double(line{6}); str2double(line{7}); str2double(line{8})]];
    end
end

obj1Motions = [resultMotions(:,1:26),resultMotions(:,28)];
obj2Motions = [resultMotions(:,27),resultMotions(:,29:end)];

obj1FirstMask = maskI1(:,:,:) == 1000;
obj2FirstMask = maskI25(:,:,:) == 1001;

obj1LastMask = maskI27(:,:,:) == 1000;

camera1PoseMatrix = poseToTransformationMatrix(resultPoses(:,1));
camera25PoseMatrix = poseToTransformationMatrix(resultPoses(:,26));
camera27PoseMatrix = poseToTransformationMatrix(resultPoses(:,28));

[imgH, imgW] = size(maskI1);
% pointCloud1
pcloud1  = zeros(imgH,imgW,3);
nObj1FirstPixels = sum(sum(obj1FirstMask));
obj1FirstWorld3DPoints = zeros(3,nObj1FirstPixels);
n = 0;
rows = [];
cols = [];
for i=1:imgH
    for j=1:imgW
        if obj1FirstMask(i,j) == 1
            pixelRow = i;
            pixelCol = j;
            rows = [rows,pixelRow];
            cols = [cols, pixelCol];
            pixelDisparity = double(depthI1(pixelRow,pixelCol))/256;
            pixelDepth = K(1,1)*0.537/pixelDisparity;
            camera3DPoint = K\[pixelRow;pixelCol;1];
            camera3DPoint(3) = pixelDepth;
            world3DPoint = camera1PoseMatrix * [camera3DPoint;1];
            n = n + 1;
            obj1FirstWorld3DPoints(:,n) = world3DPoint(1:3,1);
            pcloud1(i,j,:) = world3DPoint(1:3,1);
        end
    end
end
ptCloud1 = pointCloud(pcloud1,'Color',rgbI1);

% pointCloud25
nObj2FirstPixels = sum(sum(obj2FirstMask));
obj2FirstWorld3DPoints = zeros(3,nObj2FirstPixels);
n = 0;
pcloud25  = zeros(imgH,imgW,3);
for i=1:imgH
   for j=1:imgW
       if obj2FirstMask(i,j) == 1
           pixelRow = i;
           pixelCol = j;
           pixelDisparity = double(depthI25(pixelRow,pixelCol))/256;
           pixelDepth = K(1,1)*0.537/pixelDisparity;
           camera3DPoint = K\[pixelRow;pixelCol;1];
           camera3DPoint(3) = pixelDepth;
           world3DPoint = camera25PoseMatrix * [camera3DPoint;1];
           n = n + 1;
           obj2FirstWorld3DPoints(:,n) = world3DPoint(1:3,1);
           pcloud25(i,j,:) = world3DPoint(1:3,1);
       end
   end
end
ptCloud25 = pointCloud(pcloud25,'Color',rgbI25);

% pointCloud271
obj1TotalMotion = eye(4);
for i = 1: size(obj1Motions,2)
    obj1TotalMotion = poseToTransformationMatrix(obj1Motions(:,i))*obj1TotalMotion;
end
obj1LastWorld3DPoints = zeros(3,nObj1FirstPixels);
for i=1:nObj1FirstPixels
    obj1LastWorld3DPoints(:,i) = obj1TotalMotion(1:3,:)*[obj1FirstWorld3DPoints(:,i);1];
end
n = 0;
pcloud271  = zeros(imgH,imgW,3);
for i=1:imgH
    for j=1:imgW
        if obj1FirstMask(i,j) == 1
            n = n + 1;
            pcloud271(rows(n),cols(n),:) = obj1LastWorld3DPoints(:,n);
        end
    end
end
ptCloud271 = pointCloud(pcloud271,'Color',rgbI1);

% % for GT last object 1 point cloud
% for i=1:imgH
%     for j=1:imgW
%         if obj1LastMask(i,j) == 1
%             pixelRow = i;
%             pixelCol = j;
%             pixelDisparity = double(depthI27(pixelRow,pixelCol))/256;
%             pixelDepth = K(1,1)*0.537/pixelDisparity;
%             camera3DPoint = K\[pixelRow;pixelCol;1];
%             camera3DPoint(3) = pixelDepth;
%             world3DPoint = camera27PoseMatrix * [camera3DPoint;1];
%             pcloud271(i,j,:) = world3DPoint(1:3,1);
%         end
%     end
% end
% ptCloud271 = pointCloud(pcloud271,'Color',rgbI1);

% pointCloud 272
obj2TotalMotion = eye(4);
for i = 1: 3%size(obj2Motions,2)
    obj2TotalMotion = poseToTransformationMatrix(obj2Motions(:,i))*obj2TotalMotion;
end
obj2LastWorld3DPoints = zeros(3,nObj2FirstPixels);
for i=1:nObj2FirstPixels
    obj2LastWorld3DPoints(:,i) = obj2TotalMotion(1:3,:)*[obj2FirstWorld3DPoints(:,i);1];
end
n = 0;
pcloud272  = zeros(imgH,imgW,3);
for i=1:imgH
    for j=1:imgW
        if obj2FirstMask(i,j) == 1
            n = n + 1;
            pcloud272(i,j,:) = obj2LastWorld3DPoints(:,n);
        end
    end
end
ptCloud272 = pointCloud(pcloud272,'Color',rgbI1);


figure;
pcshow(ptCloud1);
hold on
pcshow(ptCloud25);
hold on
pcshow(ptCloud271);
hold on
pcshow(ptCloud272);
xlabel('x');
ylabel('y');
zlabel('z');
view(90,90)

