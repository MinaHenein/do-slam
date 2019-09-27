K = [7.215377000000e+02 0.000000000000e+00 6.095593000000e+02;
     0.000000000000e+00 7.215377000000e+02 1.728540000000e+02;
     0.000000000000e+00 0.000000000000e+00 1.000000000000e+00];

sequence = '0003'; 
imageRange = 0:40;
dir = '/media/mina/ACRV Samsung SSD T5/KITTI dataset/';
depthDir = 'tracking/depth/';
objSegDir = 'mots/instances/';  

gtFilePath = '/home/mina/workspace/src/Git/gtsam/Data/GraphFiles/vk-Mina/kitti-0003-0-40_GT.graph';
measFilePath = '/home/mina/workspace/src/Git/gtsam/Data/GraphFiles/vk-Mina/kitti-0003-0-40_Meas.graph';
resultFilePath = '/home/mina/workspace/src/Git/gtsam/Data/GraphFiles/vk-Mina/kitti-0003-0-40_result.graph';

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

colors = {'magenta','radioactive green','leather','blue','red','black',...
    'sapphire','swamp','plum','light bluish green','butterscotch','chartreuse','green'}; 

object1FirstFrame = 1; object1LastFrame = 28; 
object2FirstFrame = 26; object2LastFrame = 41; 

% object 1 first 3D Centroid
obj1First3DCentroid = [3.4080; 1.5368; 4.7588];
% object2 first 3D Centroid
obj2First3DCentroid = [3.7179; 1.4915; 32.5005];

obj1Motions = [resultMotions(:,1:26),resultMotions(:,28)];
obj2Motions = [resultMotions(:,27),resultMotions(:,29:end)];
object1MotionPose = zeros(6,1);
object2MotionPose = zeros(6,1);

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
    if i > object1FirstFrame && i <= object1LastFrame 
        n = i-object1FirstFrame;
        for k = 1:n
            object1MotionMatrix =  poseToTransformationMatrix(obj1Motions(:,k)) * ...
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
            object2MotionMatrix =  poseToTransformationMatrix(obj2Motions(:,k)) * ...
                poseToTransformationMatrix(object2MotionPose);
        end
        object2MotionPose = transformationMatrixToPose(object2MotionMatrix);
        newCentroid = RelativeToAbsolutePositionR3xso3(object2MotionPose, obj2First3DCentroid);
        plot3(mean(newCentroid(1,1)),mean(newCentroid(2,1)),mean(newCentroid(3,1)),'s',...
            'MarkerSize',16,'MarkerEdgeColor',rgb(colors{2}),'MarkerFaceColor',...
                rgb(colors{2}),'LineStyle','none');
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
    zlim([0 65])
    view(0,0);
    
    fig = figure(i);
    fig.Renderer = 'Painters';
    print(fig,strcat(sequence,'_',num2str(i)),'-dpng');
end


