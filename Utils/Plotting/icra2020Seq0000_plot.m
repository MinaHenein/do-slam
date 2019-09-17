measFilePath = '/home/mina/workspace/src/Git/gtsam/Data/GraphFiles/vk-Mina/kitti-0000-0-60_Meas.graph';
resultFilePath = '/home/mina/workspace/src/Git/gtsam/Data/GraphFiles/vk-Mina/kitti-0000-0-60_result.graph';
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

colors = {'cinnamon','radioactive green','leather','blue','red','black',...
    'sapphire','swamp','plum','light bluish green','butterscotch','chartreuse','green'}; 

allDynamicPointIds = identifyDynamicPointIndices(measFilePath);
staticPointIds = setdiff(resultPointIds, allDynamicPointIds);

objectPoints = vKITTI_pointObservability(measFilePath);

clear h
figure; hold on; axis equal;
for i=1:3:size(resultPoses,2)
    iPose = resultPoses(:,i);
    plotiCamera = plotCamera('Location',iPose(1:3),'Orientation',rot(-iPose(4:6))); %LHS invert pose
    plotiCamera.Opacity = 0.1;
    plotiCamera.Size = 0.5;
    plotiCamera.Color = 'red';
end

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

for j = 1:size(objectPoints,1)
    for k = 1:size(objectPoints,2)
        ids = [objectPoints{j,k}];
        if ~isempty(ids)
            Points = [];
            for m = 1:length(ids)
                Points = [Points, resultPoints(:,resultPointIds == ids(m))];
            end
            if ismember(j,[1:2:11,12:2:16,18:25])
                plotObject1 = plot3(Points(1,:),Points(2,:),Points(3,:),'.',...
                    'MarkerSize',10, 'Color', rgb(colors{1}),'LineStyle','none');
                h(2) = plotObject1;
                hold on
            else
                plotObject2 = plot3(Points(1,:),Points(2,:),Points(3,:),'.',...
                    'MarkerSize',10, 'Color', rgb(colors{2}),'LineStyle','none');
                h(3) = plotObject2;
                hold on
            end
        end
    end
end

xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
axis equal
xlim([-25 15])
zlim([0 40])
view(0,0)
legend(h, {' static structure ',' van ',' cyclist '})


AxesH    = gca;
%UpVector = [-sind(30), cosd(30), 0];
%DAR      = get(AxesH, 'DataAspectRatio');
AxesH.Box = 'on';
AxesH.FontSize = 16;
%set(AxesH, 'CameraUpVector', DAR .* UpVector);
 

