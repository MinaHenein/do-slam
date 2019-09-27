K = [7.215377000000e+02 0.000000000000e+00 6.095593000000e+02;
     0.000000000000e+00 7.215377000000e+02 1.728540000000e+02;
     0.000000000000e+00 0.000000000000e+00 1.000000000000e+00];
 
sequence = '0005'; 
imageRange = 185:229;
dir = '/media/mina/ACRV Samsung SSD T5/KITTI dataset/';
depthDir = 'tracking/depth/';
objSegDir = 'mots/instances/';  

gtFilePath = '/home/mina/workspace/src/Git/gtsam/Data/GraphFiles/vk-Mina/kitti-0005-185-229_GT_constantMotion.graph';
measFilePath = '/home/mina/workspace/src/Git/gtsam/Data/GraphFiles/vk-Mina/kitti-0005-185-229_Meas_constantMotion.graph';
resultFilePath = '/home/mina/workspace/src/Git/gtsam/Data/GraphFiles/vk-Mina/kitti-0005-185-229_result.graph';

fileID = fopen(resultFilePath,'r');
Data = textscan(fileID,'%s','delimiter','\n','whitespace',' ');
resultCStr = Data{1};
fclose(fileID);

resultPoses = [];
resultPoints = [];
resultPointIds = [];
resultMotions = [];

for i = 1:size(resultCStr,1)
    line = strsplit(resultCStr{i},' ');
    if strcmp(line{1},'VERTEX_POSE_R3_SO3')
        resultPoses = [resultPoses, [str2double(line{3}); str2double(line{4}); str2double(line{5});...
            str2double(line{6}); str2double(line{7}); str2double(line{8})]];
    elseif strcmp(line{1},'VERTEX_POINT_3D')
        resultPoints = [resultPoints, [str2double(line{3}); str2double(line{4}); str2double(line{5})]];
        resultPointIds = [resultPointIds, str2double(line{2})];
    elseif strcmp(line{1},'VERTEX_SE3Motion')
        resultMotions = [resultMotions, [str2double(line{3}); str2double(line{4}); str2double(line{5});...
            str2double(line{6}); str2double(line{7}); str2double(line{8})]];
    end
end

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

seenPointsCumulative = {};
seenPointsCumulative{1,1} = [seenPoints{1,1}];
for i = 2 :nPoses
    seenPointsCumulative{i,1} = union(seenPointsCumulative{i-1,1},[seenPoints{i,1}]);
end


allDynamicPointIds = identifyDynamicPointIndices(measFilePath);
staticPointIds = setdiff(resultPointIds, allDynamicPointIds);
staticPoints = zeros(3,length(staticPointIds));
n = 0;
for i=1:size(resultPoints,2)
    if ismember(resultPointIds(i),staticPointIds)
        n = n+1;
        staticPoints(:,n) =  resultPoints(:,i);
    end
end

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

% with object mask percentage > 2 %
% object 1 first 3D Centroid
% obj1First3DCentroid = [-4.0556; 1.7079; 17.5965];
% % object2 first 3D Centroid
% obj2First3DCentroid = [-3.8980; 1.8016; 28.1162];
% % object3 first 3D Centroid
% obj3First3DCentroid = [-4.0062; 1.8753; 44.2451];
% % object4 first 3D Centroid
% obj4First3DCentroid = [-3.9056; 1.8838; 53.8406];

%with object mask percentage > 2 %
%object 1 first 3D Centroid
obj1First3DCentroid = [-4.0807; 1.7254; 18.8803];
% object2 first 3D Centroid
obj2First3DCentroid = [-3.9390; 1.7819; 31.0156];
% object3 first 3D Centroid
obj3First3DCentroid = [-4.0479; 1.8584; 47.1850];
% object4 first 3D Centroid
obj4First3DCentroid = [-3.9332; 1.9028; 56.7259];
% object5 first 3D Centroid
obj5First3DCentroid = [-3.8078; 2.05519; 69.2076];

obj1Motion = resultMotions(:,1);
obj2Motion = resultMotions(:,2);
obj3Motion = resultMotions(:,3);
obj4Motion = resultMotions(:,4);
obj5Motion = resultMotions(:,5);

object1MotionPose = zeros(6,1);
object2MotionPose = zeros(6,1);
object3MotionPose = zeros(6,1);
object4MotionPose = zeros(6,1);
object5MotionPose = zeros(6,1);

for i = 1:length(imageRange)
    figure; hold on; axis equal;
    
    % camera
    iPose = resultPoses(:,i);
    plotiCamera = plotCamera('Location',iPose(1:3),'Orientation',rot(-iPose(4:6)));
    plotiCamera.Opacity = 0.1;
    plotiCamera.Size = 1;
    plotiCamera.Color = 'red';
    % objects
    if i == object1FirstFrame
        plot3(obj1First3DCentroid(1,1),obj1First3DCentroid(2,1),obj1First3DCentroid(3,1),'s',...
            'MarkerSize',16,'MarkerEdgeColor',rgb(colors{1}),'MarkerFaceColor',...
                rgb(colors{1}),'LineStyle','none');        
    end
    if i == object2FirstFrame
        plot3(obj2First3DCentroid(1,1),obj2First3DCentroid(2,1),obj2First3DCentroid(3,1),'s',...
            'MarkerSize',16,'MarkerEdgeColor',rgb(colors{2}),'MarkerFaceColor',...
                rgb(colors{2}),'LineStyle','none');        
    end
    if i == object3FirstFrame
        plot3(obj3First3DCentroid(1,1),obj3First3DCentroid(2,1),obj3First3DCentroid(3,1),'s',...
            'MarkerSize',16,'MarkerEdgeColor',rgb(colors{3}),'MarkerFaceColor',...
                rgb(colors{3}),'LineStyle','none');        
    end
    if i == object4FirstFrame
        plot3(obj4First3DCentroid(1,1),obj4First3DCentroid(2,1),obj4First3DCentroid(3,1),'s',...
            'MarkerSize',16,'MarkerEdgeColor',rgb(colors{4}),'MarkerFaceColor',...
                rgb(colors{4}),'LineStyle','none');        
    end
    if i == object5FirstFrame
        plot3(obj5First3DCentroid(1,1),obj5First3DCentroid(2,1),obj5First3DCentroid(3,1),'s',...
            'MarkerSize',16,'MarkerEdgeColor',rgb(colors{5}),'MarkerFaceColor',...
                rgb(colors{5}),'LineStyle','none');        
    end
    if i > object1FirstFrame && i <= object1LastFrame 
        n = i-object1FirstFrame;
        for k = 1:n
            object1MotionMatrix =  poseToTransformationMatrix(obj1Motion) * ...
                poseToTransformationMatrix(object1MotionPose);
        end
        object1MotionPose = transformationMatrixToPose(object1MotionMatrix);
        newCentroid = RelativeToAbsolutePositionR3xso3(object1MotionPose, obj1First3DCentroid);
        plot3(mean(newCentroid(1,1)),mean(newCentroid(2,1)),mean(newCentroid(3,1)),'s',...
            'MarkerSize',16,'MarkerEdgeColor',rgb(colors{1}),'MarkerFaceColor',...
                rgb(colors{1}),'LineStyle','none');
    end
    if i > object2FirstFrame && i <= object2LastFrame
        n = i-object2FirstFrame;
        for k = 1:n
            object2MotionMatrix =  poseToTransformationMatrix(obj2Motion) * ...
                poseToTransformationMatrix(object2MotionPose);
        end
        object2MotionPose = transformationMatrixToPose(object2MotionMatrix);
        newCentroid = RelativeToAbsolutePositionR3xso3(object2MotionPose, obj2First3DCentroid);
        plot3(mean(newCentroid(1,1)),mean(newCentroid(2,1)),mean(newCentroid(3,1)),'s',...
            'MarkerSize',16,'MarkerEdgeColor',rgb(colors{2}),'MarkerFaceColor',...
                rgb(colors{2}),'LineStyle','none');
    end
    if i > object3FirstFrame && i <= object3LastFrame
        n = i-object3FirstFrame;
        for k = 1:n
            object3MotionMatrix =  poseToTransformationMatrix(obj3Motion) * ...
                poseToTransformationMatrix(object3MotionPose);
        end
        object3MotionPose = transformationMatrixToPose(object3MotionMatrix);
        newCentroid = RelativeToAbsolutePositionR3xso3(object3MotionPose, obj3First3DCentroid);
        plot3(mean(newCentroid(1,1)),mean(newCentroid(2,1)),mean(newCentroid(3,1)),'s',...
            'MarkerSize',16,'MarkerEdgeColor',rgb(colors{3}),'MarkerFaceColor',...
                rgb(colors{3}),'LineStyle','none');
    end
    if i > object4FirstFrame && i <= object4LastFrame
        n = i-object4FirstFrame;
        for k = 1:n
            object4MotionMatrix =  poseToTransformationMatrix(obj4Motion) * ...
                poseToTransformationMatrix(object4MotionPose);
        end
        object4MotionPose = transformationMatrixToPose(object4MotionMatrix);
        newCentroid = RelativeToAbsolutePositionR3xso3(object4MotionPose, obj4First3DCentroid);
        plot3(mean(newCentroid(1,1)),mean(newCentroid(2,1)),mean(newCentroid(3,1)),'s',...
            'MarkerSize',16,'MarkerEdgeColor',rgb(colors{4}),'MarkerFaceColor',...
                rgb(colors{4}),'LineStyle','none');
    end
    if i > object5FirstFrame && i <= object5LastFrame
        n = i-object5FirstFrame;
        for k = 1:n
            object5MotionMatrix =  poseToTransformationMatrix(obj5Motion) * ...
                poseToTransformationMatrix(object5MotionPose);
        end
        object5MotionPose = transformationMatrixToPose(object5MotionMatrix);
        newCentroid = RelativeToAbsolutePositionR3xso3(object5MotionPose, obj5First3DCentroid);
        plot3(mean(newCentroid(1,1)),mean(newCentroid(2,1)),mean(newCentroid(3,1)),'s',...
            'MarkerSize',16,'MarkerEdgeColor',rgb(colors{5}),'MarkerFaceColor',...
                rgb(colors{5}),'LineStyle','none');
    end
    % static structure
    iSeenPoints = [seenPointsCumulative{i,1}];
    for j = 1:length(iSeenPoints)
        jSeenPointId = iSeenPoints(j);
        id = find(staticPointIds == jSeenPointId);
        if ~isempty(id)
            plot3(staticPoints(1,id),staticPoints(2,id),staticPoints(3,id),...
                'Color','b','Marker','.','MarkerSize',10,'LineStyle','none')
        end
    end
    
    xlim([-10 10])
    zlim([0 70])
    view(0,0);
    
    fig = figure(1);
    fig.Renderer = 'Painters';
    fig.Position = [10 10 900 600];
    print(fig,strcat(sequence,'_',num2str(i)),'-dpng');
    close all;
end


