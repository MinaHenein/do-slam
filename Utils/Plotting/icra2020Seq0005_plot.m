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

clear h
figure; hold on; axis equal;
for i=1:size(resultPoses,2)
    iPose = resultPoses(:,i);
    plotiCamera = plotCamera('Location',iPose(1:3),'Orientation',rot(-iPose(4:6))); %LHS invert pose
    plotiCamera.Opacity = 0.1;
    plotiCamera.Size = 0.5;
    plotiCamera.Color = 'red';
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

plotStaticStructure = plot3(staticPoints(1,:),staticPoints(2,:),staticPoints(3,:),...
    'Color','b','Marker','.','MarkerSize',7,'LineStyle','none');
h(1) = plotStaticStructure;

colors = {'cinnamon','radioactive green','black','swamp','plum',...
    'light bluish green','butterscotch','blue','chartreuse','green'}; 


obj1FirstGTPose = [-4.0556; 1.7079; 17.5965; -0.0048; 3.1369; 0.0020]; % id = 12, frame 189 
obj1motion = resultMotions(:,1); r1 = 190:192;
obj1Poses = zeros(6,length(r1)+1);
obj1Poses(:,1) = obj1FirstGTPose;
for i = 2:length(r1)+1
    obj1Pose = poseToTransformationMatrix(obj1motion)*poseToTransformationMatrix(obj1Poses(:,i-1));
    obj1Poses(:,i) = transformationMatrixToPose(obj1Pose);
end

obj2FirstGTPose = [-3.8980; 1.8016; 28.1162; -0.0044; -3.1405; -0.0038];% id = 17, frame 198
obj2motion = resultMotions(:,2); r2 = 199:202;
obj2Poses = zeros(6,length(r2)+1);
obj2Poses(:,1) = obj2FirstGTPose;
for i = 2:length(r2)+1
    obj2Pose = poseToTransformationMatrix(obj2motion)*poseToTransformationMatrix(obj2Poses(:,i-1));
    obj2Poses(:,i) = transformationMatrixToPose(obj2Pose);
end

obj3FirstGTPose = [-4.0062; 1.8753; 44.2451; 0.0028; -3.1379; -0.0063]; %id = 18, frame 211
obj3motion = resultMotions(:,3); r3 = 212:214;
obj3Poses = zeros(6,length(r3)+1);
obj3Poses(:,1) = obj3FirstGTPose;
for i = 2:length(r3)+1
    obj3Pose = poseToTransformationMatrix(obj3motion)*poseToTransformationMatrix(obj3Poses(:,i-1));
    obj3Poses(:,i) = transformationMatrixToPose(obj3Pose);
end

obj4FirstGTPose = [-3.9056; 1.8838; 53.8406; -0.0001; 3.1098; 0.0071]; %id = 19, frame 219 
obj4motion = resultMotions(:,4); r4 = 220:222;
obj4Poses = zeros(6,length(r4)+1);
obj4Poses(:,1) = obj4FirstGTPose;
for i = 2:length(r4)+1
    obj4Pose = poseToTransformationMatrix(obj4motion)*poseToTransformationMatrix(obj4Poses(:,i-1));
    obj4Poses(:,i) = transformationMatrixToPose(obj4Pose);
end


h(2) = plot3(obj1Poses(1,:),obj1Poses(2,:),obj1Poses(3,:),'.','MarkerSize',15,'Color',rgb(colors{1}),'LineStyle','none');
hold on
h(3) = plot3(obj2Poses(1,:),obj2Poses(2,:),obj2Poses(3,:),'.','MarkerSize',15,'Color',rgb(colors{2}),'LineStyle','none');
hold on
h(4) = plot3(obj3Poses(1,:),obj3Poses(2,:),obj3Poses(3,:),'.','MarkerSize',15,'Color',rgb(colors{3}),'LineStyle','none');
hold on
h(5) = plot3(obj4Poses(1,:),obj4Poses(2,:),obj4Poses(3,:),'.','MarkerSize',15,'Color',rgb(colors{4}),'LineStyle','none');

xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
axis equal
xlim([-10 10])
zlim([0 70])
view(0,0)
legend(h, {' static structure ',' 14.9 m/s ',' 15.1 m/s ',' 15.2 m/s ', ' 14.8 m/s '})

AxesH    = gca;
UpVector = [-sind(30), cosd(30), 0];
DAR      = get(AxesH, 'DataAspectRatio');
AxesH.Box = 'on';
AxesH.FontSize = 16;
set(AxesH, 'CameraUpVector', DAR .* UpVector);
 

