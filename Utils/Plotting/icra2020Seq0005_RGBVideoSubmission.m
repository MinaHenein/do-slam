K = [7.215377000000e+02 0.000000000000e+00 6.095593000000e+02;
     0.000000000000e+00 7.215377000000e+02 1.728540000000e+02;
     0.000000000000e+00 0.000000000000e+00 1.000000000000e+00];

sequence = '0005'; 
imageRange = 185:229;
dir = '/media/mina/ACRV Samsung SSD T5/KITTI dataset/';
rgbDir = 'tracking/data_tracking_image_2/training/image_02/';
depthDir = 'tracking/depth/';
objSegDir = 'mots/instances/';

resultFilePath = '/home/mina/workspace/src/Git/gtsam/Data/GraphFiles/vk-Mina/kitti-0005-185-229_result.graph';
fileID = fopen(resultFilePath,'r');
Data = textscan(fileID,'%s','delimiter','\n','whitespace',' ');
resultCStr = Data{1};
fclose(fileID);
resultPoses = [];
resultMotions = [];
resultPoints = [];
resultPointIds = [];
for i = 1:size(resultCStr,1)
    line = strsplit(resultCStr{i},' ');
    if strcmp(line{1},'VERTEX_POSE_R3_SO3')
        resultPoses = [resultPoses, [str2double(line{3}); str2double(line{4}); str2double(line{5});...
            str2double(line{6}); str2double(line{7}); str2double(line{8})]];
    elseif strcmp(line{1},'VERTEX_SE3Motion')
        resultMotions = [resultMotions, [str2double(line{3}); str2double(line{4}); str2double(line{5});...
            str2double(line{6}); str2double(line{7}); str2double(line{8})]];
    elseif strcmp(line{1},'VERTEX_POINT_3D')
        resultPoints = [resultPoints, [str2double(line{3}); str2double(line{4}); str2double(line{5})]];
        resultPointIds = [resultPointIds, str2double(line{2})];
    end
end

gtFilePath = '/home/mina/workspace/src/Git/gtsam/Data/GraphFiles/vk-Mina/kitti-0005-185-229_GT_constantMotion.graph';
fileID = fopen(gtFilePath,'r');
Data = textscan(fileID,'%s','delimiter','\n','whitespace',' ');
gtCStr = Data{1};
fclose(fileID);
seenPoints = {};
nPoses = 0;
for i = 1:size(gtCStr,1)
    line = strsplit(gtCStr{i},' ');
    if strcmp(line{1},'VERTEX_POSE_R3_SO3')
        nPoses = nPoses+1;
    elseif strcmp(line{1},'VERTEX_POINT_3D')
        if nPoses > size(seenPoints,1)
            seenPoints{nPoses,1} = str2double(line{2});
        else
            seenPoints{nPoses,1} = [seenPoints{nPoses,1}, str2double(line{2})];
        end
    end
end

measFilePath = '/home/mina/workspace/src/Git/gtsam/Data/GraphFiles/vk-Mina/kitti-0005-185-229_Meas_constantMotion.graph';
objectPoints = vKITTI_pointObservability(measFilePath);
allDynamicPointIds = identifyDynamicPointIndices(measFilePath);
staticPointIds = setdiff(resultPointIds, allDynamicPointIds);

obj1PointIds = unique([objectPoints{1,:}]);
obj2PointIds = unique([objectPoints{2,:}]);
obj3PointIds = unique([objectPoints{3,:}]);
obj4PointIds = unique([objectPoints{4,:}]);
obj5PointIds = unique([objectPoints{5,:}]);

obj1Motion = resultMotions(:,1);
obj2Motion = resultMotions(:,2);
obj3Motion = resultMotions(:,3);
obj4Motion = resultMotions(:,4);
obj5Motion = resultMotions(:,5);

colors = {'cinnamon','radioactive green','black','swamp','plum','chartreuse'}; 

% with object mask percentage > 2 %
% object1FirstFrame = 189-(imageRange(1)-1); object1LastFrame = 192-(imageRange(1)-1); 
% object2FirstFrame = 198-(imageRange(1)-1); object2LastFrame = 202-(imageRange(1)-1); 
% object3FirstFrame = 211-(imageRange(1)-1); object3LastFrame = 214-(imageRange(1)-1); 
% object4FirstFrame = 219-(imageRange(1)-1); object4LastFrame = 222-(imageRange(1)-1); 

% with object mask percentage > 1 %
object1FirstFrame = 188-(imageRange(1)-1); object1LastFrame = 193-(imageRange(1)-1); 
object2FirstFrame = 196-(imageRange(1)-1); object2LastFrame = 202-(imageRange(1)-1); 
object3FirstFrame = 209-(imageRange(1)-1); object3LastFrame = 214-(imageRange(1)-1); 
object4FirstFrame = 217-(imageRange(1)-1); object4LastFrame = 222-(imageRange(1)-1); 
object5FirstFrame = 227-(imageRange(1)-1); object5LastFrame = 229-(imageRange(1)-1); 

% object 1 cenrtoid
maskI = imread(strcat(dir,objSegDir,sequence,'/',repmat('0',1,...
    6-numel(num2str(object1FirstFrame-1+imageRange(1)))),num2str(object1FirstFrame-1+imageRange(1)),'.png'));
depthI = imread(strcat(dir,depthDir,sequence,'/',repmat('0',1,...
    6-numel(num2str(object1FirstFrame-1+imageRange(1)))),num2str(object1FirstFrame-1+imageRange(1)),'.png'));
obj1FirstMask = maskI(:,:,:) == 1012;

% object 1 first 3D Centroid
[imgH, imgW] = size(maskI);
nObj1FirstPixels = sum(sum(obj1FirstMask));
obj1First3DPoints = zeros(3,nObj1FirstPixels);
n = 0;
for i=1:imgH
    for j=1:imgW
        if obj1FirstMask(i,j) == 1
            pixelRow = i;
            pixelCol = j;
            pixelDisparity = double(depthI(pixelRow,pixelCol))/256;
            pixelDepth = K(1,1)*0.537/pixelDisparity;
            camera3DPoint = K\[pixelCol;pixelRow;1];
            camera3DPoint = camera3DPoint*pixelDepth;
            world3DPoint = poseToTransformationMatrix(resultPoses(:,object1FirstFrame)) * [camera3DPoint;1];
            n = n + 1;
            obj1First3DPoints(:,n) = world3DPoint(1:3,1);
        end
    end
end
obj1First3DCentroid = [mean(obj1First3DPoints(1,:));mean(obj1First3DPoints(2,:));mean(obj1First3DPoints(3,:))];

% all obj1 centroids at frames 189-192
obj1Centroids3D = zeros(3,object1LastFrame-(object1FirstFrame-1));
obj1Centroids3D(:,1) = obj1First3DCentroid;
for i = 2:object1LastFrame-(object1FirstFrame-1)
     objectCentroid = poseToTransformationMatrix(obj1Motion)*[obj1Centroids3D(:,i-1);1]; 
     obj1Centroids3D(:,i) = objectCentroid(1:3,1);
end
% all obj1Centroids projected on images
obj1CentroidsPixel = zeros(2,object1LastFrame-(object1FirstFrame-1));
for i = 1:object1LastFrame-(object1FirstFrame-1)
    cameraPoseMatrix = poseToTransformationMatrix(resultPoses(:,i+object1FirstFrame-1));
    camera3DPoint = cameraPoseMatrix\[obj1Centroids3D(:,i);1];
    camera3DPoint = camera3DPoint(1:3,1);
    pixel = K * camera3DPoint;
    pixel = pixel/pixel(3);
    obj1CentroidsPixel(:,i) = pixel(1:2,1);
end

% object 2 centroid
maskI = imread(strcat(dir,objSegDir,sequence,'/',repmat('0',1,...
    6-numel(num2str(object2FirstFrame-1+imageRange(1)))),num2str(object2FirstFrame-1+imageRange(1)),'.png'));
depthI = imread(strcat(dir,depthDir,sequence,'/',repmat('0',1,...
    6-numel(num2str(object2FirstFrame-1+imageRange(1)))),num2str(object2FirstFrame-1+imageRange(1)),'.png'));
obj2FirstMask = maskI(:,:,:) == 1017;
% object 2 first 3D Centroid
nObj2FirstPixels = sum(sum(obj2FirstMask));
obj2First3DPoints = zeros(3,nObj2FirstPixels);
n = 0;
for i=1:imgH
    for j=1:imgW
        if obj2FirstMask(i,j) == 1
            pixelRow = i;
            pixelCol = j;
            pixelDisparity = double(depthI(pixelRow,pixelCol))/256;
            pixelDepth = K(1,1)*0.537/pixelDisparity;
            camera3DPoint = K\[pixelCol;pixelRow;1];
            camera3DPoint = camera3DPoint*pixelDepth;
            world3DPoint = poseToTransformationMatrix(resultPoses(:,object2FirstFrame)) * [camera3DPoint;1];
            n = n + 1;
            obj2First3DPoints(:,n) = world3DPoint(1:3,1);
        end
    end
end
obj2First3DCentroid = [mean(obj2First3DPoints(1,:));mean(obj2First3DPoints(2,:));mean(obj2First3DPoints(3,:))];

% all obj2 centroids at frames 198-202
obj2Centroids3D = zeros(3,object2LastFrame-(object2FirstFrame-1));
obj2Centroids3D(:,1) = obj2First3DCentroid;
for i = 2:object2LastFrame-(object2FirstFrame-1)
     objectCentroid = poseToTransformationMatrix(obj2Motion)*[obj2Centroids3D(:,i-1);1]; 
     obj2Centroids3D(:,i) = objectCentroid(1:3,1);
end
% all obj2Centroids projected on images
obj2CentroidsPixel = zeros(2,object2LastFrame-(object2FirstFrame-1));
for i = 1:object2LastFrame-(object2FirstFrame-1)
    cameraPoseMatrix = poseToTransformationMatrix(resultPoses(:,i+object2FirstFrame-1));
    camera3DPoint = cameraPoseMatrix\[obj2Centroids3D(:,i);1];
    camera3DPoint = camera3DPoint(1:3,1);
    pixel = K * camera3DPoint;
    pixel = pixel/pixel(3);
    obj2CentroidsPixel(:,i) = pixel(1:2,1);
end

% object 3 centroid
maskI = imread(strcat(dir,objSegDir,sequence,'/',repmat('0',1,...
    6-numel(num2str(object3FirstFrame-1+imageRange(1)))),num2str(object3FirstFrame-1+imageRange(1)),'.png'));
depthI = imread(strcat(dir,depthDir,sequence,'/',repmat('0',1,...
    6-numel(num2str(object3FirstFrame-1+imageRange(1)))),num2str(object3FirstFrame-1+imageRange(1)),'.png'));
obj3FirstMask = maskI(:,:,:) == 1018;
% object 3 first 3D Centroid
nObj3FirstPixels = sum(sum(obj3FirstMask));
obj3First3DPoints = zeros(3,nObj3FirstPixels);
n = 0;
for i=1:imgH
    for j=1:imgW
        if obj3FirstMask(i,j) == 1
            pixelRow = i;
            pixelCol = j;
            pixelDisparity = double(depthI(pixelRow,pixelCol))/256;
            pixelDepth = K(1,1)*0.537/pixelDisparity;
            camera3DPoint = K\[pixelCol;pixelRow;1];
            camera3DPoint = camera3DPoint*pixelDepth;
            world3DPoint = poseToTransformationMatrix(resultPoses(:,object3FirstFrame)) * [camera3DPoint;1];
            n = n + 1;
            obj3First3DPoints(:,n) = world3DPoint(1:3,1);
        end
    end
end
obj3First3DCentroid = [mean(obj3First3DPoints(1,:));mean(obj3First3DPoints(2,:));mean(obj3First3DPoints(3,:))];

% all obj3 centroids at frames 211-214
obj3Centroids3D = zeros(3,object3LastFrame-(object3FirstFrame-1));
obj3Centroids3D(:,1) = obj3First3DCentroid;
for i = 2:object3LastFrame-(object3FirstFrame-1)
     objectCentroid = poseToTransformationMatrix(obj3Motion)*[obj3Centroids3D(:,i-1);1]; 
     obj3Centroids3D(:,i) = objectCentroid(1:3,1);
end
% all obj3Centroids projected on images
obj3CentroidsPixel = zeros(2,object3LastFrame-(object3FirstFrame-1));
for i = 1:object3LastFrame-(object3FirstFrame-1)
    cameraPoseMatrix = poseToTransformationMatrix(resultPoses(:,i+object3FirstFrame-1));
    camera3DPoint = cameraPoseMatrix\[obj3Centroids3D(:,i);1];
    camera3DPoint = camera3DPoint(1:3,1);
    pixel = K * camera3DPoint;
    pixel = pixel/pixel(3);
    obj3CentroidsPixel(:,i) = pixel(1:2,1);
end

% object 4 centroid
maskI = imread(strcat(dir,objSegDir,sequence,'/',repmat('0',1,...
    6-numel(num2str(object4FirstFrame-1+imageRange(1)))),num2str(object4FirstFrame-1+imageRange(1)),'.png'));
depthI = imread(strcat(dir,depthDir,sequence,'/',repmat('0',1,...
    6-numel(num2str(object4FirstFrame-1+imageRange(1)))),num2str(object4FirstFrame-1+imageRange(1)),'.png'));
obj4FirstMask = maskI(:,:,:) == 1019;
% object 4 first 3D Centroid
nObj4FirstPixels = sum(sum(obj4FirstMask));
obj4First3DPoints = zeros(3,nObj4FirstPixels);
n = 0;
for i=1:imgH
    for j=1:imgW
        if obj4FirstMask(i,j) == 1
            pixelRow = i;
            pixelCol = j;
            pixelDisparity = double(depthI(pixelRow,pixelCol))/256;
            pixelDepth = K(1,1)*0.537/pixelDisparity;
            camera3DPoint = K\[pixelCol;pixelRow;1];
            camera3DPoint = camera3DPoint*pixelDepth;
            world3DPoint = poseToTransformationMatrix(resultPoses(:,object4FirstFrame)) * [camera3DPoint;1];
            n = n + 1;
            obj4First3DPoints(:,n) = world3DPoint(1:3,1);
        end
    end
end
obj4First3DCentroid = [mean(obj4First3DPoints(1,:));mean(obj4First3DPoints(2,:));mean(obj4First3DPoints(3,:))];

% all obj4 centroids at frames 219-222
obj4Centroids3D = zeros(3,object4LastFrame-(object4FirstFrame-1));
obj4Centroids3D(:,1) = obj4First3DCentroid;
for i = 2:object4LastFrame-(object4FirstFrame-1)
     objectCentroid = poseToTransformationMatrix(obj4Motion)*[obj4Centroids3D(:,i-1);1]; 
     obj4Centroids3D(:,i) = objectCentroid(1:3,1);
end
% all obj4Centroids projected on images
obj4CentroidsPixel = zeros(2,object4LastFrame-(object4FirstFrame-1));
for i = 1:object4LastFrame-(object4FirstFrame-1)
    cameraPoseMatrix = poseToTransformationMatrix(resultPoses(:,i+object4FirstFrame-1));
    camera3DPoint = cameraPoseMatrix\[obj4Centroids3D(:,i);1];
    camera3DPoint = camera3DPoint(1:3,1);
    pixel = K * camera3DPoint;
    pixel = pixel/pixel(3);
    obj4CentroidsPixel(:,i) = pixel(1:2,1);
end

% object 5 centroid
maskI = imread(strcat(dir,objSegDir,sequence,'/',repmat('0',1,...
    6-numel(num2str(object5FirstFrame-1+imageRange(1)))),num2str(object5FirstFrame-1+imageRange(1)),'.png'));
depthI = imread(strcat(dir,depthDir,sequence,'/',repmat('0',1,...
    6-numel(num2str(object5FirstFrame-1+imageRange(1)))),num2str(object5FirstFrame-1+imageRange(1)),'.png'));
obj5FirstMask = maskI(:,:,:) == 1020;
% object 5 first 3D Centroid
nObj5FirstPixels = sum(sum(obj5FirstMask));
obj5First3DPoints = zeros(3,nObj5FirstPixels);
n = 0;
for i=1:imgH
    for j=1:imgW
        if obj4FirstMask(i,j) == 1
            pixelRow = i;
            pixelCol = j;
            pixelDisparity = double(depthI(pixelRow,pixelCol))/256;
            pixelDepth = K(1,1)*0.537/pixelDisparity;
            camera3DPoint = K\[pixelCol;pixelRow;1];
            camera3DPoint = camera3DPoint*pixelDepth;
            world3DPoint = poseToTransformationMatrix(resultPoses(:,object5FirstFrame)) * [camera3DPoint;1];
            n = n + 1;
            obj5First3DPoints(:,n) = world3DPoint(1:3,1);
        end
    end
end
obj5First3DCentroid = [mean(obj5First3DPoints(1,:));mean(obj5First3DPoints(2,:));mean(obj5First3DPoints(3,:))];

% all obj 5 centroids at frames 219-222
obj5Centroids3D = zeros(3,object5LastFrame-(object5FirstFrame-1));
obj5Centroids3D(:,1) = obj5First3DCentroid;
for i = 2:object5LastFrame-(object5FirstFrame-1)
     objectCentroid = poseToTransformationMatrix(obj5Motion)*[obj5Centroids3D(:,i-1);1]; 
     obj5Centroids3D(:,i) = objectCentroid(1:3,1);
end
% all obj5Centroids projected on images
obj5CentroidsPixel = zeros(2,object5LastFrame-(object5FirstFrame-1));
for i = 1:object5LastFrame-(object5FirstFrame-1)
    cameraPoseMatrix = poseToTransformationMatrix(resultPoses(:,i+object5FirstFrame-1));
    camera3DPoint = cameraPoseMatrix\[obj5Centroids3D(:,i);1];
    camera3DPoint = camera3DPoint(1:3,1);
    pixel = K * camera3DPoint;
    pixel = pixel/pixel(3);
    obj5CentroidsPixel(:,i) = pixel(1:2,1);
end

for i = 1:length(imageRange)
    rgbI = imread(strcat(dir,rgbDir,sequence,'/',repmat('0',1,...
        6-numel(num2str(i-1+imageRange(1)))),num2str(i-1+imageRange(1)),'.png'));  
    figure;imshow(rgbI); hold on;
        
    % obj1 pixel centroid
    if i >=  object1FirstFrame && i<= object1LastFrame
        scatter(obj1CentroidsPixel(1,i-(object1FirstFrame-1)),obj1CentroidsPixel(2,i-(object1FirstFrame-1)),'MarkerEdgeColor',...
            rgb(colors{1}),'MarkerFaceColor',rgb(colors{1}));  
        if i > object1FirstFrame
        speed = norm(obj1Motion(1:3,1) - ((eye(3)-rot(obj1Motion(4:6,1)))*obj1Centroids3D(:,i-(object1FirstFrame))));
        text(obj1CentroidsPixel(1,i-(object1FirstFrame-1))-100,obj1CentroidsPixel(2,i-(object1FirstFrame-1))-100,...
            strcat('\bf',num2str(round(speed*36)),' km/hr'),'Color', rgb(colors{1}),'FontSize',22);
        end
    end
    % obj2 pixel centroids
    if i>= object2FirstFrame && i<= object2LastFrame
        scatter(obj2CentroidsPixel(1,i-(object2FirstFrame-1)),obj2CentroidsPixel(2,i-(object2FirstFrame-1)),'MarkerEdgeColor',...
            rgb(colors{2}),'MarkerFaceColor',rgb(colors{2}));
        if i > object2FirstFrame
        speed = norm(obj2Motion(1:3,1) - ((eye(3)-rot(obj2Motion(4:6,1)))*...
            obj2Centroids3D(:,i-(object2FirstFrame))));
        text(obj2CentroidsPixel(1,i-(object2FirstFrame-1))-100,obj2CentroidsPixel(2,i-(object2FirstFrame-1))-100,...
            strcat('\bf',num2str(round(speed*36)),' km/hr'),'Color', rgb(colors{2}),'FontSize',22);
        end
    end
    % obj3 pixel centroids
    if i>= object3FirstFrame && i<= object3LastFrame
        scatter(obj3CentroidsPixel(1,i-(object3FirstFrame-1)),obj3CentroidsPixel(2,i-(object3FirstFrame-1)),'MarkerEdgeColor',...
            rgb(colors{3}),'MarkerFaceColor',rgb(colors{3}));
        if i > object3FirstFrame
        speed = norm(obj3Motion(1:3,1) - ((eye(3)-rot(obj3Motion(4:6,1)))*...
            obj3Centroids3D(:,i-(object3FirstFrame))));
        text(obj3CentroidsPixel(1,i-(object3FirstFrame-1))-100,obj3CentroidsPixel(2,i-(object3FirstFrame-1))-100,...
            strcat('\bf',num2str(round(speed*36)),' km/hr'),'Color', rgb(colors{3}),'FontSize',22);
        end
    end
    % obj4 pixel centroids
    if i>= object4FirstFrame && i<= object4LastFrame
        scatter(obj4CentroidsPixel(1,i-(object4FirstFrame-1)),obj4CentroidsPixel(2,i-(object4FirstFrame-1)),'MarkerEdgeColor',...
            rgb(colors{4}),'MarkerFaceColor',rgb(colors{4}));
        if i > object4FirstFrame
        speed = norm(obj4Motion(1:3,1) - ((eye(3)-rot(obj4Motion(4:6,1)))*...
            obj4Centroids3D(:,i-(object4FirstFrame))));
        text(obj4CentroidsPixel(1,i-(object4FirstFrame-1))-100,obj4CentroidsPixel(2,i-(object4FirstFrame-1))-100,...
            strcat('\bf',num2str(round(speed*36)),' km/hr'),'Color', rgb(colors{4}),'FontSize',22);
        end
    end
    % obj5 pixel centroids
    if i>= object5FirstFrame && i<= object5LastFrame
        scatter(obj5CentroidsPixel(1,i-(object5FirstFrame-1)),obj5CentroidsPixel(2,i-(object5FirstFrame-1)),'MarkerEdgeColor',...
            rgb(colors{5}),'MarkerFaceColor',rgb(colors{5}));
        if i > object5FirstFrame
        speed = norm(obj5Motion(1:3,1) - ((eye(3)-rot(obj5Motion(4:6,1)))*...
            obj5Centroids3D(:,i-(object5FirstFrame))));
        text(obj5CentroidsPixel(1,i-(object5FirstFrame-1))-100,obj5CentroidsPixel(2,i-(object5FirstFrame-1))-100,...
            strcat('\bf',num2str(round(speed*36)),' km/hr'),'Color', rgb(colors{5}),'FontSize',22);
        end
    end
    % structure
    iSeenPoints = [seenPoints{i,1}];
    for j = 1:length(iSeenPoints)
        jSeenPointId = iSeenPoints(j);
        % static
        if ismember(jSeenPointId, staticPointIds)
            cameraPoseMatrix = poseToTransformationMatrix(resultPoses(:,i));
            camera3DPoint = cameraPoseMatrix\[resultPoints(:,resultPointIds == jSeenPointId);1];
            camera3DPoint = camera3DPoint(1:3,1);
            pixel = K * camera3DPoint;
            pixel = pixel/pixel(3);
            scatter(pixel(1,1),pixel(2,1),15,'b*');
        end
        % object 1
        if ismember(jSeenPointId, obj1PointIds)
            cameraPoseMatrix = poseToTransformationMatrix(resultPoses(:,i));
            camera3DPoint = cameraPoseMatrix\[resultPoints(:,resultPointIds == jSeenPointId);1];
            camera3DPoint = camera3DPoint(1:3,1);
            pixel = K * camera3DPoint;
            pixel = pixel/pixel(3);
            scatter(pixel(1,1),pixel(2,1),10,'MarkerEdgeColor',...
            rgb(colors{1}),'MarkerFaceColor',rgb(colors{1}),'Marker','*');
        end
        % object 2
        if ismember(jSeenPointId, obj2PointIds)
            cameraPoseMatrix = poseToTransformationMatrix(resultPoses(:,i));
            camera3DPoint = cameraPoseMatrix\[resultPoints(:,resultPointIds == jSeenPointId);1];
            camera3DPoint = camera3DPoint(1:3,1);
            pixel = K * camera3DPoint;
            pixel = pixel/pixel(3);
            scatter(pixel(1,1),pixel(2,1),10,'MarkerEdgeColor',...
            rgb(colors{2}),'MarkerFaceColor',rgb(colors{2}),'Marker','*');
        end
        % object 3
        if ismember(jSeenPointId, obj3PointIds)
            cameraPoseMatrix = poseToTransformationMatrix(resultPoses(:,i));
            camera3DPoint = cameraPoseMatrix\[resultPoints(:,resultPointIds == jSeenPointId);1];
            camera3DPoint = camera3DPoint(1:3,1);
            pixel = K * camera3DPoint;
            pixel = pixel/pixel(3);
            scatter(pixel(1,1),pixel(2,1),10,'MarkerEdgeColor',...
            rgb(colors{3}),'MarkerFaceColor',rgb(colors{3}),'Marker','*');
        end
        % object 4
        if ismember(jSeenPointId, obj4PointIds)
            cameraPoseMatrix = poseToTransformationMatrix(resultPoses(:,i));
            camera3DPoint = cameraPoseMatrix\[resultPoints(:,resultPointIds == jSeenPointId);1];
            camera3DPoint = camera3DPoint(1:3,1);
            pixel = K * camera3DPoint;
            pixel = pixel/pixel(3);
            scatter(pixel(1,1),pixel(2,1),10,'MarkerEdgeColor',...
            rgb(colors{4}),'MarkerFaceColor',rgb(colors{4}),'Marker','*');
        end
        % object 5
        if ismember(jSeenPointId, obj5PointIds)
            cameraPoseMatrix = poseToTransformationMatrix(resultPoses(:,i));
            camera3DPoint = cameraPoseMatrix\[resultPoints(:,resultPointIds == jSeenPointId);1];
            camera3DPoint = camera3DPoint(1:3,1);
            pixel = K * camera3DPoint;
            pixel = pixel/pixel(3);
            scatter(pixel(1,1),pixel(2,1),10,'MarkerEdgeColor',...
            rgb(colors{5}),'MarkerFaceColor',rgb(colors{5}),'Marker','*');
        end
    end
    
    fig = figure(1);
    fig.Renderer = 'Painters';
    fig.Position = [1 1 728 2231];
    print(fig,strcat(sequence,'_',num2str(i)),'-dpng');
    close all;
    pause(0.5)
end
