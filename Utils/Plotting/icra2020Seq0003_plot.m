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


figure; hold on; axis equal;
l = 2; % coordinate axis length
A = [0 0 0 1; l 0 0 1; 0 0 0 1; 0 l 0 1; 0 0 0 1; 0 0 l 1]';
for i=1:3:size(resultPoses,2)
  B = poseToTransformationMatrix(resultPoses(:,i))*A;
  plot3(B(1,1:2),B(2,1:2),B(3,1:2),'-r','LineWidth',2); % x: red
  plot3(B(1,3:4),B(2,3:4),B(3,3:4),'-g','LineWidth',2); % y: green
  plot3(B(1,5:6),B(2,5:6),B(3,5:6),'-b','LineWidth',2); % z: blue
end
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
axis equal
view(0,0)

colors = {'magenta','radioactive green','leather','blue','red','black','cornflower',...
    'sapphire','swamp','plum','light bluish green','butterscotch','cinnamon','chartreuse','green'}; 


objectPoints = vKITTI_pointObservability(gtFilePath,measFilePath);
for i=1:size(resultPoints,2)
    for j = 1:size(objectPoints,1)     
        iObjectPoints = [objectPoints{j,:}];
        if ismember(resultPointIds(i),iObjectPoints)
            if ismember(j,[1:26,28])
                scatter3(resultPoints(1,i),resultPoints(2,i),resultPoints(3,i),7,...
                    'MarkerEdgeColor',rgb(colors{1}),'MarkerFaceColor',rgb(colors{1}));
            elseif ismember(j,[27,29:42])
                scatter3(resultPoints(1,i),resultPoints(2,i),resultPoints(3,i),7,...
                    'MarkerEdgeColor',rgb(colors{2}),'MarkerFaceColor',rgb(colors{2}));
            end
        end
    end
end

% for j = 1:size(objectPoints,1)
%     for k = 1:size(objectPoints,2)
%         ids = [objectPoints{j,k}];
%         if ~isempty(ids)
%             Points = [];
%             for m = 1:length(ids)
%                 Points = [Points, resultPoints(:,resultPointIds == ids(m))];
%             end
%             if ismember(j,[1:26,28])
%                 plot3(Points(1,:),Points(2,:),Points(3,:),'.','MarkerSize',10, 'Color', rgb(colors{1}));
%                 hold on
%             else
%                 plot3(Points(1,:),Points(2,:),Points(3,:),'.','MarkerSize',10, 'Color', rgb(colors{2}));
%                 hold on
%             end
%         end
%     end
% end

legend({'object 1','object 2'})
 

