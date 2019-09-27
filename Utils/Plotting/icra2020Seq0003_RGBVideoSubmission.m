K = [7.215377000000e+02 0.000000000000e+00 6.095593000000e+02;
     0.000000000000e+00 7.215377000000e+02 1.728540000000e+02;
     0.000000000000e+00 0.000000000000e+00 1.000000000000e+00];

sequence = '0003'; 
imageRange = 0:40;
dir = '/media/mina/ACRV Samsung SSD T5/KITTI dataset/';
rgbDir = 'tracking/data_tracking_image_2/training/image_02/';
depthDir = 'tracking/depth/';
objSegDir = 'mots/instances/';

resultFilePath = '/home/mina/workspace/src/Git/gtsam/Data/GraphFiles/vk-Mina/kitti-0003-0-40_result.graph';
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

gtFilePath = '/home/mina/workspace/src/Git/gtsam/Data/GraphFiles/vk-Mina/kitti-0003-0-40_GT.graph';
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

measFilePath = '/home/mina/workspace/src/Git/gtsam/Data/GraphFiles/vk-Mina/kitti-0003-0-40_Meas.graph';
objectPoints = vKITTI_pointObservability(measFilePath);
allDynamicPointIds = identifyDynamicPointIndices(measFilePath);
staticPointIds = setdiff(resultPointIds, allDynamicPointIds);

obj1PointIds = [];
obj2PointIds = [];
for j = 1:size(objectPoints,1)
    for k = 1:size(objectPoints,2)
        ids = [objectPoints{j,k}];
        if ~isempty(ids)
            if ismember(j,[1:26,28])
              obj1PointIds = [obj1PointIds,ids];  
            else
              obj2PointIds = [obj2PointIds,ids];  
            end
        end
    end
end

obj1Motions = [resultMotions(:,1:26),resultMotions(:,28)];
obj2Motions = [resultMotions(:,27),resultMotions(:,29:end)];

colors = {'magenta','radioactive green','leather','blue','red','black',...
    'sapphire','swamp','plum','light bluish green','butterscotch','chartreuse','green'}; 

object1FirstFrame = 1; object1LastFrame = 28; 
object2FirstFrame = 26; object2LastFrame = 41; 

% object 1 cenrtoid
maskI = imread(strcat(dir,objSegDir,sequence,'/',repmat('0',1,...
    6-numel(num2str(object1FirstFrame-1))),num2str(object1FirstFrame-1),'.png'));
depthI = imread(strcat(dir,depthDir,sequence,'/',repmat('0',1,...
    6-numel(num2str(object1FirstFrame-1))),num2str(object1FirstFrame-1),'.png'));
obj1FirstMask = maskI(:,:,:) == 1000;

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

% all obj1 centroids at frames 0-27
obj1Centroids3D = zeros(3,28);
obj1Centroids3D(:,1) = obj1First3DCentroid;
for i = 2:28
     objectCentroid = poseToTransformationMatrix(obj1Motions(:,i-1))*[obj1Centroids3D(:,i-1);1]; 
     obj1Centroids3D(:,i) = objectCentroid(1:3,1);
end
% all obj1Centroids projected on images
obj1CentroidsPixel = zeros(2,28);
for i = 1:28
    cameraPoseMatrix = poseToTransformationMatrix(resultPoses(:,i));
    camera3DPoint = cameraPoseMatrix\[obj1Centroids3D(:,i);1];
    camera3DPoint = camera3DPoint(1:3,1);
    pixel = K * camera3DPoint;
    pixel = pixel/pixel(3);
    obj1CentroidsPixel(:,i) = pixel(1:2,1);
end

% object 2 centroid
maskI = imread(strcat(dir,objSegDir,sequence,'/',repmat('0',1,...
    6-numel(num2str(object2FirstFrame-1))),num2str(object2FirstFrame-1),'.png'));
depthI = imread(strcat(dir,depthDir,sequence,'/',repmat('0',1,...
    6-numel(num2str(object2FirstFrame-1))),num2str(object2FirstFrame-1),'.png'));
obj2FirstMask = maskI(:,:,:) == 1001;
% object 1 first 3D Centroid
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

% all obj2 centroids at frames 25-40
obj2Centroids3D = zeros(3,16);
obj2Centroids3D(:,1) = obj2First3DCentroid;
for i = 2:16
     objectCentroid = poseToTransformationMatrix(obj2Motions(:,i-1))*[obj2Centroids3D(:,i-1);1]; 
     obj2Centroids3D(:,i) = objectCentroid(1:3,1);
end
% all obj2Centroids projected on images
obj2CentroidsPixel = zeros(2,16);
for i = 1:16
    cameraPoseMatrix = poseToTransformationMatrix(resultPoses(:,i+object2FirstFrame-1));
    camera3DPoint = cameraPoseMatrix\[obj2Centroids3D(:,i);1];
    camera3DPoint = camera3DPoint(1:3,1);
    pixel = K * camera3DPoint;
    pixel = pixel/pixel(3);
    obj2CentroidsPixel(:,i) = pixel(1:2,1);
end

% projecting all centroids on last image with both objects; frame 27
for i = 1:length(imageRange)
    rgbI = imread(strcat(dir,rgbDir,sequence,'/',repmat('0',1,6-numel(num2str(i-1))),num2str(i-1),'.png'));  
    figure;imshow(rgbI); hold on;
        
    % obj1 pixel centroid
    if i >=  object1FirstFrame && i<= object1LastFrame
        scatter(obj1CentroidsPixel(1,i),obj1CentroidsPixel(2,i),'MarkerEdgeColor',...
            rgb(colors{1}),'MarkerFaceColor',rgb(colors{1}));  
        if i > object1FirstFrame
        speed = norm(obj1Motions(1:3,i-1) - ((eye(3)-rot(obj1Motions(4:6,i-1)))*obj1Centroids3D(:,i-1)));
        text(obj1CentroidsPixel(1,i)-100,obj1CentroidsPixel(2,i)-100,strcat('\bf',num2str(round(speed*36)),' km/hr'),...
            'Color', rgb(colors{1}),'FontSize',22);
        end
    end
    % obj2 pixel centroids
    if i>= object2FirstFrame && i<= object2LastFrame
        scatter(obj2CentroidsPixel(1,i-(object2FirstFrame-1)),obj2CentroidsPixel(2,i-(object2FirstFrame-1)),'MarkerEdgeColor',...
            rgb(colors{2}),'MarkerFaceColor',rgb(colors{2}));
        if i > object2FirstFrame
        speed = norm(obj2Motions(1:3,i-(object2FirstFrame)) - ((eye(3)-rot(obj2Motions(4:6,i-(object2FirstFrame))))*...
            obj2Centroids3D(:,i-(object2FirstFrame))));
        text(obj2CentroidsPixel(1,i-(object2FirstFrame-1))-100,obj2CentroidsPixel(2,i-(object2FirstFrame-1))-100,...
            strcat('\bf',num2str(round(speed*36)),' km/hr'),'Color', rgb(colors{2}),'FontSize',22);
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
        if ismember(jSeenPointId, obj2PointIds)
            cameraPoseMatrix = poseToTransformationMatrix(resultPoses(:,i));
            camera3DPoint = cameraPoseMatrix\[resultPoints(:,resultPointIds == jSeenPointId);1];
            camera3DPoint = camera3DPoint(1:3,1);
            pixel = K * camera3DPoint;
            pixel = pixel/pixel(3);
            scatter(pixel(1,1),pixel(2,1),10,'MarkerEdgeColor',...
            rgb(colors{2}),'MarkerFaceColor',rgb(colors{2}),'Marker','*');
        end
    end
    
    fig = figure(1);
    fig.Renderer = 'Painters';
    print(fig,strcat(sequence,'_',num2str(i)),'-dpng');
    close all;
    pause(0.5)
end
