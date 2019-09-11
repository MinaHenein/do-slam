gtFilePath = '/home/mina/workspace/src/Git/gtsam/Data/GraphFiles/vk-Mina/kitti-0006-40-140_GT_constantMotion.graph';
measFilePath = '/home/mina/workspace/src/Git/gtsam/Data/GraphFiles/vk-Mina/kitti-0006-40-140_Meas_constantMotion.graph';
resultFilePath = '/home/mina/workspace/src/Git/gtsam/Data/GraphFiles/vk-Mina/kitti-0006-40-140_result.graph';
fileID = fopen(resultFilePath,'r');
Data = textscan(fileID,'%s','delimiter','\n','whitespace',' ');
resultCStr = Data{1};
fclose(fileID);

resultMotions = [];
for i = 1:size(resultCStr,1)
    line = strsplit(resultCStr{i},' ');
    if strcmp(line{1},'VERTEX_SE3Motion')
        resultMotions = [resultMotions, [str2double(line{3}); str2double(line{4}); str2double(line{5});...
            str2double(line{6}); str2double(line{7}); str2double(line{8})]];
    end
end

colors = {'magenta','radioactive green','leather','blue','red','black','cornflower',...
    'sapphire','swamp','plum','light bluish green','butterscotch','cinnamon','chartreuse','green'}; 

obj1FirstGTPose = [3.8556; 1.6302; 5.7422; 0.0000; -0.0293; 0.0000]; % id = 2, frame 40
obj1motion = resultMotions(:,1); r1 = 41:49;
obj1Poses = zeros(6,length(r1)+1);
obj1Poses(:,1) = obj1FirstGTPose;
for i = 2:length(r1)+1
    obj1Pose = poseToTransformationMatrix(obj1motion)*poseToTransformationMatrix(obj1Poses(:,i-1));
    obj1Poses(:,i) = transformationMatrixToPose(obj1Pose);
end

obj2FirstGTPose = [7.7000; 1.6206; 8.7726; 0.0000; -0.0078; 0.0001];% id = 7, frame 44
obj2motion = resultMotions(:,2); r2 = 45:52;
obj2Poses = zeros(6,length(r2)+1);
obj2Poses(:,1) = obj2FirstGTPose;
for i = 2:length(r2)+1
    obj2Pose = poseToTransformationMatrix(obj2motion)*poseToTransformationMatrix(obj2Poses(:,i-1));
    obj2Poses(:,i) = transformationMatrixToPose(obj2Pose);
end

obj3FirstGTPose = [7.7288; 1.6235; 7.9063; 0.0000; -0.0240; 0.0001]; %id = 8, frame 55
obj3motion = resultMotions(:,3); r3 = 56:59;
obj3Poses = zeros(6,length(r3)+1);
obj3Poses(:,1) = obj3FirstGTPose;
for i = 2:length(r3)+1
    obj3Pose = poseToTransformationMatrix(obj3motion)*poseToTransformationMatrix(obj3Poses(:,i-1));
    obj3Poses(:,i) = transformationMatrixToPose(obj3Pose);
end

obj4FirstGTPose = [3.3403; 1.6547; 2.9276; 0.0000; -0.0220; 0.0001]; %id = 3, frame 57
obj4motion = resultMotions(:,4); r4 = 58:70;
obj4Poses = zeros(6,length(r4)+1);
obj4Poses(:,1) = obj4FirstGTPose;
for i = 2:length(r4)+1
    obj4Pose = poseToTransformationMatrix(obj4motion)*poseToTransformationMatrix(obj4Poses(:,i-1));
    obj4Poses(:,i) = transformationMatrixToPose(obj4Pose);
end

obj5FirstGTPose = [3.7028; 1.6511; 2.6195; 0.0001; -0.0417; 0.0002]; %id = 4, frame 70
obj5motion = resultMotions(:,5); r5 = 71:85;
obj5Poses = zeros(6,length(r5)+1);
obj5Poses(:,1) = obj5FirstGTPose;
for i = 2:length(r5)+1
    obj5Pose = poseToTransformationMatrix(obj5motion)*poseToTransformationMatrix(obj5Poses(:,i-1));
    obj5Poses(:,i) = transformationMatrixToPose(obj5Pose);
end

obj6FirstGTPose = [3.6876;1.6383; 2.5188; 0.0001; -0.0169; 0.0002]; %id = 5, frame 86
obj6motion = resultMotions(:,6); r6 = 87:102;
obj6Poses = zeros(6,length(r6)+1);
obj6Poses(:,1) = obj6FirstGTPose;
for i = 2:length(r6)+1
    obj6Pose = poseToTransformationMatrix(obj6motion)*poseToTransformationMatrix(obj6Poses(:,i-1));
    obj6Poses(:,i) = transformationMatrixToPose(obj6Pose);
end

obj7FirstGTPose = [3.6332; 1.6183; 3.0379; 0.0001; 0.0033; 0.0001]; %id = 6, frame 97
obj7motion = resultMotions(:,7); r7 = 98:109;
obj7Poses = zeros(6,length(r7)+1);
obj7Poses(:,1) = obj7FirstGTPose;
for i = 2:length(r7)+1
    obj7Pose = poseToTransformationMatrix(obj7motion)*poseToTransformationMatrix(obj7Poses(:,i-1));
    obj7Poses(:,i) = transformationMatrixToPose(obj7Pose);
end

obj8FirstGTPose = [7.5398; 1.6095; 8.2310; 0.0001; -0.0280; 0.0000]; %id = 10, frame 103
obj8motion = resultMotions(:,8); r8 = 104:111;
obj8Poses = zeros(6,length(r8)+1);
obj8Poses(:,1) = obj8FirstGTPose;
for i = 2:length(r8)+1
    obj8Pose = poseToTransformationMatrix(obj8motion)*poseToTransformationMatrix(obj8Poses(:,i-1));
    obj8Poses(:,i) = transformationMatrixToPose(obj8Pose);
end

figure;
scatter3(obj1Poses(1,end),obj1Poses(2,end),obj1Poses(3,end),...
'MarkerEdgeColor',rgb(colors{1}),'MarkerFaceColor',rgb(colors{1}));
hold on
scatter3(obj2Poses(1,end),obj2Poses(2,end),obj2Poses(3,end),...
'MarkerEdgeColor',rgb(colors{2}),'MarkerFaceColor',rgb(colors{2}));
hold on
scatter3(obj3Poses(1,end),obj3Poses(2,end),obj3Poses(3,end),...
'MarkerEdgeColor',rgb(colors{3}),'MarkerFaceColor',rgb(colors{3}));
hold on
scatter3(obj4Poses(1,end),obj4Poses(2,end),obj4Poses(3,end),...
'MarkerEdgeColor',rgb(colors{4}),'MarkerFaceColor',rgb(colors{4}));
hold on
scatter3(obj5Poses(1,end),obj5Poses(2,end),obj5Poses(3,end),...
'MarkerEdgeColor',rgb(colors{5}),'MarkerFaceColor',rgb(colors{5}));
hold on
scatter3(obj6Poses(1,end),obj6Poses(2,end),obj6Poses(3,end),...
'MarkerEdgeColor',rgb(colors{6}),'MarkerFaceColor',rgb(colors{6}));
hold on
scatter3(obj7Poses(1,end),obj7Poses(2,end),obj7Poses(3,end),...
'MarkerEdgeColor',rgb(colors{7}),'MarkerFaceColor',rgb(colors{7}));
hold on
scatter3(obj8Poses(1,end),obj8Poses(2,end),obj8Poses(3,end),...
'MarkerEdgeColor',rgb(colors{8}),'MarkerFaceColor',rgb(colors{8}));
hold on
plot3(obj1Poses(1,:),obj1Poses(2,:),obj1Poses(3,:),'Color',rgb(colors{1}));
hold on
plot3(obj2Poses(1,:),obj2Poses(2,:),obj2Poses(3,:),'Color',rgb(colors{2}));
hold on
plot3(obj3Poses(1,:),obj3Poses(2,:),obj3Poses(3,:),'Color',rgb(colors{3}));
hold on
plot3(obj4Poses(1,:),obj4Poses(2,:),obj4Poses(3,:),'Color',rgb(colors{4}));
hold on
plot3(obj5Poses(1,:),obj5Poses(2,:),obj5Poses(3,:),'Color',rgb(colors{5}));
hold on
plot3(obj6Poses(1,:),obj6Poses(2,:),obj6Poses(3,:),'Color',rgb(colors{6}));
hold on
plot3(obj7Poses(1,:),obj7Poses(2,:),obj7Poses(3,:),'Color',rgb(colors{7}));
hold on
plot3(obj8Poses(1,:),obj8Poses(2,:),obj8Poses(3,:),'Color',rgb(colors{8}));
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
axis equal
grid off
xlim([-2 10])
zlim([2 22])
legend({'8.8 m/s','9.5 m/s','8.1 m/s','8.7 m/s','9.3 m/s','9.2 m/s','10.2 m/s','10.7 m/s'},'Position',[0.4 0.75 0.1 0.2]);
view(0,0)