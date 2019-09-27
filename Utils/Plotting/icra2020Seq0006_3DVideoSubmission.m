K = [7.215377000000e+02 0.000000000000e+00 6.095593000000e+02;
     0.000000000000e+00 7.215377000000e+02 1.728540000000e+02;
     0.000000000000e+00 0.000000000000e+00 1.000000000000e+00];
 
sequence = '0006'; 
imageRange = 40:140;
dir = '/media/mina/ACRV Samsung SSD T5/KITTI dataset/';
depthDir = 'tracking/depth/';
objSegDir = 'mots/instances/';  

gtFilePath = '/home/mina/workspace/src/Git/gtsam/Data/GraphFiles/vk-Mina/kitti-0006-40-140_GT_constantMotion.graph';
measFilePath = '/home/mina/workspace/src/Git/gtsam/Data/GraphFiles/vk-Mina/kitti-0006-40-140_Meas_constantMotion.graph';
resultFilePath = '/home/mina/workspace/src/Git/gtsam/Data/GraphFiles/vk-Mina/kitti-0006-40-140_result.graph';

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
for i = 2 :size(seenPoints,1)
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

colors = {'magenta','radioactive green','leather','red','black','green','cornflower',...
    'sapphire','swamp','plum','light bluish green','butterscotch','cinnamon','chartreuse','blue'};

object1FirstFrame = 40-(imageRange(1)-1); object1LastFrame = 49-(imageRange(1)-1); 
object2FirstFrame = 44-(imageRange(1)-1); object2LastFrame = 52-(imageRange(1)-1); 
object3FirstFrame = 55-(imageRange(1)-1); object3LastFrame = 59-(imageRange(1)-1); 
object4FirstFrame = 57-(imageRange(1)-1); object4LastFrame = 70-(imageRange(1)-1); 
object5FirstFrame = 70-(imageRange(1)-1); object5LastFrame = 85-(imageRange(1)-1); 
object6FirstFrame = 86-(imageRange(1)-1); object6LastFrame = 102-(imageRange(1)-1); 
object7FirstFrame = 97-(imageRange(1)-1); object7LastFrame = 109-(imageRange(1)-1); 
object8FirstFrame = 103-(imageRange(1)-1); object8LastFrame = 111-(imageRange(1)-1);

% object 1 first 3D Centroid
obj1First3DCentroid = [3.8556; 1.6302; 5.7422];
% object 2 first 3D Centroid
obj2First3DCentroid = [7.7000; 1.6206; 8.7726];
% object 3 first 3D Centroid
obj3First3DCentroid = [7.7288; 1.6235; 7.9063];
% object 4 first 3D Centroid
obj4First3DCentroid = [3.3403; 1.6547; 2.9276];
% object 5 first 3D Centroid
obj5First3DCentroid = [3.7028; 1.6511; 2.6195];
% object 6 first 3D Centroid
obj6First3DCentroid = [3.6876;1.6383; 2.5188];
% object 7 first 3D Centroid
obj7First3DCentroid = [3.6332; 1.6183; 3.0379];
% object 8 first 3D Centroid
obj8First3DCentroid = [7.5398; 1.6095; 8.2310];

obj1Motion = resultMotions(:,1);
obj2Motion = resultMotions(:,2);
obj3Motion = resultMotions(:,3);
obj4Motion = resultMotions(:,4);
obj5Motion = resultMotions(:,5);
obj6Motion = resultMotions(:,6);
obj7Motion = resultMotions(:,7);
obj8Motion = resultMotions(:,8);

object1MotionPose = zeros(6,1);
object2MotionPose = zeros(6,1);
object3MotionPose = zeros(6,1);
object4MotionPose = zeros(6,1);
object5MotionPose = zeros(6,1);
object6MotionPose = zeros(6,1);
object7MotionPose = zeros(6,1);
object8MotionPose = zeros(6,1);

for i = 1:size(seenPoints,1)
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
    if i == object6FirstFrame
        plot3(obj6First3DCentroid(1,1),obj6First3DCentroid(2,1),obj6First3DCentroid(3,1),'s',...
            'MarkerSize',16,'MarkerEdgeColor',rgb(colors{6}),'MarkerFaceColor',...
                rgb(colors{6}),'LineStyle','none');        
    end
    if i == object7FirstFrame
        plot3(obj7First3DCentroid(1,1),obj7First3DCentroid(2,1),obj7First3DCentroid(3,1),'s',...
            'MarkerSize',16,'MarkerEdgeColor',rgb(colors{7}),'MarkerFaceColor',...
                rgb(colors{7}),'LineStyle','none');        
    end
    if i == object8FirstFrame
        plot3(obj8First3DCentroid(1,1),obj8First3DCentroid(2,1),obj8First3DCentroid(3,1),'s',...
            'MarkerSize',16,'MarkerEdgeColor',rgb(colors{8}),'MarkerFaceColor',...
                rgb(colors{8}),'LineStyle','none');        
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
    if i > object6FirstFrame && i <= object6LastFrame
        n = i-object6FirstFrame;
        for k = 1:n
            object6MotionMatrix =  poseToTransformationMatrix(obj6Motion) * ...
                poseToTransformationMatrix(object6MotionPose);
        end
        object6MotionPose = transformationMatrixToPose(object6MotionMatrix);
        newCentroid = RelativeToAbsolutePositionR3xso3(object6MotionPose, obj6First3DCentroid);
        plot3(mean(newCentroid(1,1)),mean(newCentroid(2,1)),mean(newCentroid(3,1)),'s',...
            'MarkerSize',16,'MarkerEdgeColor',rgb(colors{6}),'MarkerFaceColor',...
                rgb(colors{6}),'LineStyle','none');
    end
    if i > object7FirstFrame && i <= object7LastFrame
        n = i-object7FirstFrame;
        for k = 1:n
            object7MotionMatrix =  poseToTransformationMatrix(obj7Motion) * ...
                poseToTransformationMatrix(object7MotionPose);
        end
        object7MotionPose = transformationMatrixToPose(object7MotionMatrix);
        newCentroid = RelativeToAbsolutePositionR3xso3(object7MotionPose, obj7First3DCentroid);
        plot3(mean(newCentroid(1,1)),mean(newCentroid(2,1)),mean(newCentroid(3,1)),'s',...
            'MarkerSize',16,'MarkerEdgeColor',rgb(colors{7}),'MarkerFaceColor',...
                rgb(colors{7}),'LineStyle','none');
    end
    if i > object8FirstFrame && i <= object8LastFrame
        n = i-object8FirstFrame;
        for k = 1:n
            object8MotionMatrix =  poseToTransformationMatrix(obj8Motion) * ...
                poseToTransformationMatrix(object8MotionPose);
        end
        object8MotionPose = transformationMatrixToPose(object8MotionMatrix);
        newCentroid = RelativeToAbsolutePositionR3xso3(object8MotionPose, obj8First3DCentroid);
        plot3(mean(newCentroid(1,1)),mean(newCentroid(2,1)),mean(newCentroid(3,1)),'s',...
            'MarkerSize',16,'MarkerEdgeColor',rgb(colors{8}),'MarkerFaceColor',...
                rgb(colors{8}),'LineStyle','none');
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
    zlim([0 30])
    view(0,0);
    
    fig = figure(1);
    fig.Renderer = 'Painters';
    fig.Position = [10 10 900 600];
    print(fig,strcat(sequence,'_',num2str(i)),'-dpng');
    close all;
end


