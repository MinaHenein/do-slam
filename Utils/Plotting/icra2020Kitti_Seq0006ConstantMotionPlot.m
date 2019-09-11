obj1FirstGTPose = [3.8556; 1.6302; 5.7422; 0.0000; -0.0293; 0.0000]; % id = 2, frame 40
obj2FirstGTPose = [7.7000; 1.6206; 8.7726; 0.0000; -0.0078; 0.0001]; % id = 7, frame 44
obj3FirstGTPose = [7.7288; 1.6235; 7.9063; 0.0000; -0.0240; 0.0001]; % id = 8; frame 55
obj4FirstGTPose = [3.3403; 1.6547; 2.9276; 0.0000; -0.0220; 0.0001]; % id = 3, frame 57
obj5FirstGTPose = [3.7028; 1.6511; 2.6195; 0.0001; -0.0417; 0.0002]; % id = 4, frame 70
obj6FirstGTPose = [3.6876; 1.6383; 2.5188; 0.0001; -0.0169; 0.0002]; % id = 5, frame 86
obj7FirstGTPose = [3.6332; 1.6183; 3.0379; 0.0001;  0.0033; 0.0001]; % id = 6, frame 97
obj8FirstGTPose = [7.5398; 1.6095; 8.2310; 0.0001; -0.0280; 0.0000]; % id = 10, frame 103
%--------------------------------------------------------------------------
obj1motion = [-0.016613714 -0.007036165 0.883586718 -0.001823747 0.000012302 -0.000542680]'; r1 = 41:49;
obj2motion = [0.001945550 -0.000626476 0.926918746 -0.000183176 -0.002364142 -0.000652793]'; r2 = 45:52; 
obj3motion = [0.005970735 0.000872689 0.785413237 0.000026723 -0.003837729 0.000271542]';    r3 = 56:59;
obj4motion = [0.015639018 0.007701345 0.854513466 0.000480494 -0.004064250 0.000168913]';    r4 = 58:70;
obj5motion = [0.007917363 0.021409589 0.916816059 0.000763938 -0.003061592 -0.002715538]';   r5 = 71:85;
obj6motion = [0.014414451 0.008616286 0.913545311 0.000577555 -0.003140297 0.000435110]';    r6 = 87:102;
obj7motion = [0.021317994 0.011002005 1.006773319 0.000572929 -0.003031003 -0.000130644]';   r7 = 98:109;
obj8motion = [-0.014894930 0.000610638 1.075595615 -0.001332878 0.000377607 -0.001501121]';  r8 = 104:111;
%--------------------------------------------------------------------------
obj1Poses = zeros(6,length(r1)+1);
obj1Poses(:,1) = obj1FirstGTPose;
for i = 2:length(r1)+1
 obj1Pose = poseToTransformationMatrix(obj1motion)*poseToTransformationMatrix(obj1Poses(:,i-1));
 obj1Poses(:,i) = transformationMatrixToPose(obj1Pose);
end

obj2Poses = zeros(6,length(r2)+1);
obj2Poses(:,1) = obj2FirstGTPose;
for i = 2:length(r2)+1
 obj2Pose = poseToTransformationMatrix(obj2motion)*poseToTransformationMatrix(obj2Poses(:,i-1));
 obj2Poses(:,i) = transformationMatrixToPose(obj2Pose);
end

obj3Poses = zeros(6,length(r3)+1);
obj3Poses(:,1) = obj3FirstGTPose;
for i = 2:length(r3)+1
 obj3Pose = poseToTransformationMatrix(obj3motion)*poseToTransformationMatrix(obj3Poses(:,i-1));
 obj3Poses(:,i) = transformationMatrixToPose(obj3Pose);
end

obj4Poses = zeros(6,length(r4)+1);
obj4Poses(:,1) = obj4FirstGTPose;
for i = 2:length(r4)+1
 obj4Pose = poseToTransformationMatrix(obj4motion)*poseToTransformationMatrix(obj4Poses(:,i-1));
 obj4Poses(:,i) = transformationMatrixToPose(obj4Pose);
end

obj5Poses = zeros(6,length(r5)+1);
obj5Poses(:,1) = obj5FirstGTPose;
for i = 2:length(r5)+1
 obj5Pose = poseToTransformationMatrix(obj5motion)*poseToTransformationMatrix(obj5Poses(:,i-1));
 obj5Poses(:,i) = transformationMatrixToPose(obj5Pose);
end

obj6Poses = zeros(6,length(r6)+1);
obj6Poses(:,1) = obj6FirstGTPose;
for i = 2:length(r6)+1
 obj6Pose = poseToTransformationMatrix(obj6motion)*poseToTransformationMatrix(obj6Poses(:,i-1));
 obj6Poses(:,i) = transformationMatrixToPose(obj6Pose);
end

obj7Poses = zeros(6,length(r7)+1);
obj7Poses(:,1) = obj7FirstGTPose;
for i = 2:length(r7)+1
 obj7Pose = poseToTransformationMatrix(obj7motion)*poseToTransformationMatrix(obj7Poses(:,i-1));
 obj7Poses(:,i) = transformationMatrixToPose(obj7Pose);
end

obj8Poses = zeros(6,length(r8)+1);
obj8Poses(:,1) = obj8FirstGTPose;
for i = 2:length(r7)+1
 obj8Pose = poseToTransformationMatrix(obj8motion)*poseToTransformationMatrix(obj8Poses(:,i-1));
 obj8Poses(:,i) = transformationMatrixToPose(obj8Pose);
end

colors = {'magenta','radioactive green','leather','red','green','black','sapphire','swamp','light bluish green',...
    'butterscotch','cinnamon','chartreuse','blue'}; 

figure; 
hold on

for j=1:size(obj1Poses,2)
    id  = 1;
    pose = poseToTransformationMatrix(obj1Poses(:,j));
    hi = scatter3(pose(1,4),pose(2,4),pose(3,4),'MarkerEdgeColor',rgb(colors(id)),...
                'MarkerFaceColor',rgb(colors(id)));
    h(1) = hi(1);        
end
hold on
for j=1:size(obj2Poses,2)
    id  = 2;
    pose = poseToTransformationMatrix(obj2Poses(:,j));
    hi = scatter3(pose(1,4),pose(2,4),pose(3,4),'MarkerEdgeColor',rgb(colors(id)),...
                'MarkerFaceColor',rgb(colors(id)));
   h(2) = hi(1);         
end
hold on
for j=1:size(obj3Poses,2)
    id  = 3;
    pose = poseToTransformationMatrix(obj3Poses(:,j));
    hi = scatter3(pose(1,4),pose(2,4),pose(3,4),'MarkerEdgeColor',rgb(colors(id)),...
                'MarkerFaceColor',rgb(colors(id)));
    h(3) = hi(1);
end
hold on
for j=1:size(obj4Poses,2)
    id  = 4;
    pose = poseToTransformationMatrix(obj4Poses(:,j));
    hi = scatter3(pose(1,4),pose(2,4),pose(3,4),'MarkerEdgeColor',rgb(colors(id)),...
                'MarkerFaceColor',rgb(colors(id)));
    h(4) = hi(1);
end
hold on
for j=1:size(obj5Poses,2)
    id  = 5;
    pose = poseToTransformationMatrix(obj5Poses(:,j));
    hi = scatter3(pose(1,4),pose(2,4),pose(3,4),'MarkerEdgeColor',rgb(colors(id)),...
        'MarkerFaceColor',rgb(colors(id)));
    h(5) = hi(1);
end
hold on
for j=1:size(obj6Poses,2)
    id  = 6;
    pose = poseToTransformationMatrix(obj6Poses(:,j));
    hi = scatter3(pose(1,4),pose(2,4),pose(3,4),'MarkerEdgeColor',rgb(colors(id)),...
                'MarkerFaceColor',rgb(colors(id)));
    h(6) = hi(1);
end
hold on
for j=1:size(obj7Poses,2)
    id  = 7;
    pose = poseToTransformationMatrix(obj7Poses(:,j));
    hi = scatter3(pose(1,4),pose(2,4),pose(3,4),'MarkerEdgeColor',rgb(colors(id)),...
                'MarkerFaceColor',rgb(colors(id)));     
    h(7) = hi(1);
end
hold on
for j=1:size(obj8Poses,2)
    id  = 8;
    pose = poseToTransformationMatrix(obj8Poses(:,j));
    hi = scatter3(pose(1,4),pose(2,4),pose(3,4),'MarkerEdgeColor',rgb(colors(id)),...
                'MarkerFaceColor',rgb(colors(id)));
    h(8) = hi(1);
end
xlabel('x (m)');
ylabel('y (m)');
zlabel('z (m)');
legend(h,{'8.8 m/s','9.5 m/s','8.1 m/s','8.7 m/s','9.3 m/s','9.2 m/s','10.2 m/s','10.7 m/s'})