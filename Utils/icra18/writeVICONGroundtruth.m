function writeVICONGroundtruth(fileName,GTPoses,synchronisedData)

for i = 1:size(synchronisedData,1)
    
    if strcmp(fileName,'robot')
        poseID = i;
        pose = GTPoses(synchronisedData(i,4),:);
        pose(1) = pose(1)-0.1;
        pose(3) = 0.325; 
        fID = fopen('/home/mina/workspace/src/Git/do-slam/Utils/icra18/cameraGroundtruth.txt','a');
        fprintf(fID,'%s %d %6f %6f %6f %6f %6f %6f',...
            'VERTEX_POSE_R3_SO3',poseID,pose);
    elseif strcmp(fileName,'obj1')
        poseID = i;
        pose = GTPoses(synchronisedData(i,5),:);
        fID = fopen('/home/mina/workspace/src/Git/do-slam/Utils/icra18/obj1Groundtruth.txt','a');
        fprintf(fID,'%s %d %6f %6f %6f %6f %6f %6f',...
            'VERTEX_POSE_R3_SO3',poseID,pose);
    elseif strcmp(fileName,'obj2')
        poseID = i;
        pose = GTPoses(synchronisedData(i,6),:);
        fID = fopen('/home/mina/workspace/src/Git/do-slam/Utils/icra18/obj2Groundtruth.txt','a');
        fprintf(fID,'%s %d %6f %6f %6f %6f %6f %6f',...
            'VERTEX_POSE_R3_SO3',poseID,pose);
    end
    fprintf(fID,'\n');
end

fclose(fID);