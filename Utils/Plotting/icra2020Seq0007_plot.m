initFilePath = '/home/mina/workspace/src/Git/gtsam/Data/GraphFiles/vk-Mina/kitti-0007-15-510_initialisation.graph';
gtFilePath = '/home/mina/workspace/src/Git/gtsam/Data/GraphFiles/vk-Mina/kitti-0007-15-510_GT.graph';
resultFilePath = '/home/mina/workspace/src/Git/gtsam/Data/GraphFiles/vk-Mina/kitti-0007-15-510_result.graph';

fileID = fopen(initFilePath,'r');
initData = textscan(fileID,'%s','delimiter','\n','whitespace',' ');
initCStr = initData{1};
fclose(fileID);

fileID = fopen(gtFilePath,'r');
gtData = textscan(fileID,'%s','delimiter','\n','whitespace',' ');
gtCStr = gtData{1};
fclose(fileID);

fileID = fopen(resultFilePath,'r');
Data = textscan(fileID,'%s','delimiter','\n','whitespace',' ');
resultCStr = Data{1};
fclose(fileID);

initPoses = [];
initPoints = [];
for i = 1:size(initCStr,1)
    line = strsplit(initCStr{i},' ');
    if strcmp(line{1},'VERTEX_POSE_R3_SO3')
        initPoses = [initPoses, [str2double(line{3}); str2double(line{4}); str2double(line{5});...
            str2double(line{6}); str2double(line{7}); str2double(line{8})]];
    elseif strcmp(line{1},'VERTEX_POINT_3D')
        initPoints = [initPoints, [str2double(line{3}); str2double(line{4}); str2double(line{5})]];
    end
end

gtPoses = [];
gtPoints = [];
for i = 1:size(gtCStr,1)
    line = strsplit(gtCStr{i},' ');
    if strcmp(line{1},'VERTEX_POSE_R3_SO3')
        gtPoses = [gtPoses, [str2double(line{3}); str2double(line{4}); str2double(line{5});...
            str2double(line{6}); str2double(line{7}); str2double(line{8})]];
    elseif strcmp(line{1},'VERTEX_POINT_3D')
        gtPoints = [gtPoints, [str2double(line{3}); str2double(line{4}); str2double(line{5})]];
    end
end

resultPoses = [];
resultPoints = [];
for i = 1:size(resultCStr,1)
    line = strsplit(resultCStr{i},' ');
    if strcmp(line{1},'VERTEX_POSE_R3_SO3')
        resultPoses = [resultPoses, [str2double(line{3}); str2double(line{4}); str2double(line{5});...
            str2double(line{6}); str2double(line{7}); str2double(line{8})]];
    elseif strcmp(line{1},'VERTEX_POINT_3D')
        resultPoints = [resultPoints, [str2double(line{3}); str2double(line{4}); str2double(line{5})]];
    end
end

figure;
plot3(initPoses(1,:),initPoses(2,:),initPoses(3,:),'Color','r','Marker','.','MarkerSize',7,'LineStyle','none');
hold on
plot3(gtPoses(1,:),gtPoses(2,:),gtPoses(3,:),'Color','g','Marker','*','MarkerSize',3,'LineStyle','none');
hold on
plot3(resultPoses(1,:),resultPoses(2,:),resultPoses(3,:),'Color','b','Marker','.','MarkerSize',7,'LineStyle','none');
hold on
plot3(resultPoints(1,:),resultPoints(2,:),resultPoints(3,:),'Color','b','Marker','.','MarkerSize',7,'LineStyle','none');
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
axis equal
view(0,0)
lgd = legend({'initialisation','ground-truth','final estimate'});
lgd.Location = 'northwest';

AxesH    = gca;
AxesH.Box = 'on';
AxesH.FontSize = 16;