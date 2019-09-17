K = [7.215377000000e+02 0.000000000000e+00 6.095593000000e+02;
     0.000000000000e+00 7.215377000000e+02 1.728540000000e+02;
     0.000000000000e+00 0.000000000000e+00 1.000000000000e+00];

sequence = '0003'; 
dir = '/media/mina/ACRV Samsung SSD T5/KITTI dataset/';
rgbDir = 'tracking/data_tracking_image_2/training/image_02/';
 
gtFilePath = '/home/mina/workspace/src/Git/gtsam/Data/GraphFiles/vk-Mina/kitti-0003-0-40_GT.graph';
measFilePath = '/home/mina/workspace/src/Git/gtsam/Data/GraphFiles/vk-Mina/kitti-0003-0-40_Meas.graph';
resultFilePath = '/home/mina/workspace/src/Git/gtsam/Data/GraphFiles/vk-Mina/kitti-0003-0-40_result.graph';
fileID = fopen(resultFilePath,'r');
Data = textscan(fileID,'%s','delimiter','\n','whitespace',' ');
resultCStr = Data{1};
fclose(fileID);
resultMotions = [];
resultPoses = [];
resultPoints = [];
resultPointIds = [];
for i = 1:size(resultCStr,1)
    line = strsplit(resultCStr{i},' ');
    if strcmp(line{1},'VERTEX_SE3Motion')
        resultMotions = [resultMotions, [str2double(line{3}); str2double(line{4}); str2double(line{5});...
            str2double(line{6}); str2double(line{7}); str2double(line{8})]];
    elseif strcmp(line{1},'VERTEX_POSE_R3_SO3')
        resultPoses = [resultPoses, [str2double(line{3}); str2double(line{4}); str2double(line{5});...
            str2double(line{6}); str2double(line{7}); str2double(line{8})]];
    elseif strcmp(line{1},'VERTEX_POINT_3D')
        resultPoints = [resultPoints, [str2double(line{3}); str2double(line{4}); str2double(line{5})]];
        resultPointIds = [resultPointIds, str2double(line{2})];
    end
end


fileID = fopen(gtFilePath,'r');
Data = textscan(fileID,'%s','delimiter','\n','whitespace',' ');
gtCStr = Data{1};
fclose(fileID);
gtPoses = [];
gtPoints = [];
gtPointIds = [];
for i = 1:size(gtCStr,1)
    line = strsplit(resultCStr{i},' ');
    if  strcmp(line{1},'VERTEX_POSE_R3_SO3')
        gtPoses = [gtPoses, [str2double(line{3}); str2double(line{4}); str2double(line{5});...
            str2double(line{6}); str2double(line{7}); str2double(line{8})]];
    elseif strcmp(line{1},'VERTEX_POINT_3D')
        gtPoints = [gtPoints, [str2double(line{3}); str2double(line{4}); str2double(line{5})]];
        gtPointIds = [gtPointIds, str2double(line{2})];
    end
end

allDynamicPointIds = identifyDynamicPointIndices(measFilePath);
staticPointIds = setdiff(resultPointIds, allDynamicPointIds);
objectPoints = vKITTI_pointObservability(measFilePath);

object1PointIds = [objectPoints{28,28}];
object1PointIds = unique(object1PointIds);

object2PointIds = [objectPoints{42,41}];
object2PointIds = unique(object2PointIds);

obj1Motions = [resultMotions(:,1:26),resultMotions(:,28)];
obj2Motions = [resultMotions(:,27),resultMotions(:,29:end)];

camera1PoseMatrix = poseToTransformationMatrix(zeros(6,1));
camera25PoseMatrix = poseToTransformationMatrix([0.053254489 -0.042281451 ...
    28.452109564 0.004605552 -0.036166036 0.005545187]');
camera40PoseMatrix = poseToTransformationMatrix([0.342597572 -0.244253715 ...
    46.428466183 0.003063212 -0.000356523 0.011754798]');

% centroid obj1 at frame 0
obj1First3DCentroid =  [3.4080; 1.5368; 4.7588];
% centroid obj2 at frame 25
depthDir = 'tracking/depth/';
depthI25 = imread(strcat(dir,depthDir,sequence,'/000025.png'));
pixelRow = 330;
pixelCol = 1185;
pixelDisparity = double(depthI25(pixelRow,pixelCol))/256;
pixelDepth = K(1,1)*0.537/pixelDisparity;
camera3DPoint = K\[pixelCol;pixelRow;1];
camera3DPoint = camera3DPoint * pixelDepth;
cameraPoseMatrix = poseToTransformationMatrix(resultPoses(:,26));
obj2First3DCentroid = cameraPoseMatrix * [camera3DPoint;1];
obj2First3DCentroid = obj2First3DCentroid(1:3,1);

% all obj1 centroids at frames 0-27
obj1Centroids3D = zeros(3,41);
obj1Centroids3D(:,1) = obj1First3DCentroid;
for i = 2:41
     if i > 28
        objectCentroid = poseToTransformationMatrix([0.07;0;1.95;0;0;0])*[obj1Centroids3D(:,i-1);1]; 
        obj1Centroids3D(:,i) = objectCentroid(1:3,1);
     else
        objectCentroid = poseToTransformationMatrix(obj1Motions(:,i-1))*[obj1Centroids3D(:,i-1);1]; 
        obj1Centroids3D(:,i) = objectCentroid(1:3,1);
     end
end

% all obj2 centroids at frames 25-40
obj2Centroids3D = zeros(3,16);
obj2Centroids3D(:,1) = obj2First3DCentroid;
for i = 2:16
     objectCentroid = poseToTransformationMatrix(obj2Motions(:,i-1))*[obj2Centroids3D(:,i-1);1]; 
     obj2Centroids3D(:,i) = objectCentroid(1:3,1);
end

% all obj1Centroids projected on images
obj1CentroidsPixel = zeros(2,41);
for i = 1:41
    cameraPoseMatrix = poseToTransformationMatrix(resultPoses(:,i));
    camera3DPoint = cameraPoseMatrix\[obj1Centroids3D(:,i);1];
    camera3DPoint = camera3DPoint(1:3,1);
    pixel = K * camera3DPoint;
    pixel = pixel/pixel(3);
    obj1CentroidsPixel(:,i) = pixel(1:2,1);
end

% all obj1Centroids projected on images
obj2CentroidsPixel = zeros(2,16);
for i = 1:16
    cameraPoseMatrix = poseToTransformationMatrix(resultPoses(:,25+i));
    camera3DPoint = cameraPoseMatrix\[obj2Centroids3D(:,i);1];
    camera3DPoint = camera3DPoint(1:3,1);
    pixel = K * camera3DPoint;
    pixel = pixel/pixel(3);
    obj2CentroidsPixel(:,i) = pixel(1:2,1);
end


staticPoints = zeros(3,length(staticPointIds));
n = 0;
for i=1:size(gtPoints,2)
    if ismember(gtPointIds(i),staticPointIds)
        n = n+1;
        staticPoints(:,n) =  gtPoints(:,i);
    end
end

object1Points = zeros(3,length(object1PointIds));
n = 0;
for i=1:size(gtPoints,2)
    if ismember(gtPointIds(i),object1PointIds)
        n = n+1;
        object1Points(:,n) =  gtPoints(:,i);
    end
end

object2Points = zeros(3,length(object2PointIds));
n = 0;
for i=1:size(gtPoints,2)
    if ismember(gtPointIds(i),object2PointIds)
        n = n+1;
        object2Points(:,n) =  gtPoints(:,i);
    end
end

% projecting all 3D points on frame 40
lastFrameStaticPixels = [];
fr = 41;
for i = 1:size(staticPoints,2)
    cameraPoseMatrix = poseToTransformationMatrix(gtPoses(:,fr));
    camera3DPoint = cameraPoseMatrix\[staticPoints(:,i);1];
    camera3DPoint = camera3DPoint(1:3,1);
    pixel = K * camera3DPoint;
    pixel = pixel/pixel(3);
    lastFrameStaticPixels = [lastFrameStaticPixels,pixel(1:2,1)];
end

lastFrameObject1Pixels = [];
for i = 1:size(object1Points,2)
    cameraPoseMatrix = poseToTransformationMatrix(gtPoses(:,fr));
    camera3DPoint = cameraPoseMatrix\[object1Points(:,i);1];
    camera3DPoint = camera3DPoint(1:3,1);
    pixel = K * camera3DPoint;
    pixel = pixel/pixel(3);
    lastFrameObject1Pixels = [lastFrameObject1Pixels,pixel(1:2,1)];
end

lastFrameObject2Pixels = [];
for i = 1:size(object2Points,2)
    cameraPoseMatrix = poseToTransformationMatrix(gtPoses(:,fr));
    camera3DPoint = cameraPoseMatrix\[object2Points(:,i);1];
    camera3DPoint = camera3DPoint(1:3,1);
    pixel = K * camera3DPoint;
    pixel = pixel/pixel(3);
    lastFrameObject2Pixels = [lastFrameObject2Pixels,pixel(1:2,1)];
end

colors = {'magenta','radioactive green','leather','blue','red','black','cornflower',...
    'sapphire','swamp','plum','light bluish green','butterscotch','cinnamon','chartreuse','green'}; 

% projecting every centroid on its corresponding image
% for i = 0:40
%     rgbI = imread(strcat(dir,rgbDir,sequence,'/',repmat('0',...
%         1,6-numel(num2str(i))),num2str(i),'.png'));
%     figure;imshow(rgbI); hold on;
%     %obj1 pixel centroid
%     if ismember(i,0:27)
%         scatter(obj1CentroidsPixel(1,i+1),obj1CentroidsPixel(2,i+1),'MarkerEdgeColor',...
%             rgb(colors{1}),'MarkerFaceColor',rgb(colors{1}));
%     end
%     %obj1 pixel centroid
%     if ismember(i,25:40)
%         scatter(obj2CentroidsPixel(1,i-25+1),obj2CentroidsPixel(2,i-25+1),'MarkerEdgeColor',...
%             rgb(colors{2}),'MarkerFaceColor',rgb(colors{2}));
%     end
%     hold off;
% end

% projecting all centroids on last image with both objects; frame 27
i = 40;
rgbI = imread(strcat(dir,rgbDir,sequence,'/',repmat('0',1,6-numel(num2str(i))),num2str(i),'.png'));
figure;imshow(rgbI); hold on;

%obj1 pixel centroids
for i = 1:41
  scatter(obj1CentroidsPixel(1,i),obj1CentroidsPixel(2,i),'MarkerEdgeColor',...
        rgb(colors{1}),'MarkerFaceColor',rgb(colors{1}));  
  hold on
end
%obj2 pixel centroids
for i = 1:16
    scatter(obj2CentroidsPixel(1,i),obj2CentroidsPixel(2,i),'MarkerEdgeColor',...
        rgb(colors{2}),'MarkerFaceColor',rgb(colors{2}));
    hold on
end

scatter(lastFrameStaticPixels(1,1:10:end),lastFrameStaticPixels(2,1:10:end),16,'*',...
        'MarkerEdgeColor',[0 0 1],'MarkerFaceColor',[0 0 1]);
%scatter(lastFrameObject1Pixels(1,1:10:end),lastFrameObject1Pixels(2,1:10:end),7,'x',...
%    'MarkerEdgeColor',rgb(colors{1}),'MarkerFaceColor',rgb(colors{1}));
scatter(lastFrameObject2Pixels(1,1:10:end),lastFrameObject2Pixels(2,1:10:end),7,'x',...
    'MarkerEdgeColor',rgb(colors{2}),'MarkerFaceColor',rgb(colors{2}));
