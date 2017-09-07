function writeLandmarkMeas(pointsMeasurements,pointsLabels,pointsTurtlebotID)

disp('Writing landmarkMeasGraphFile ...');

for k = 1:size(pointsMeasurements,1)

    
    allCameras = uniqueValid3DPointsCameras(k,:);
    allCameras = allCameras(~cellfun('isempty',allCameras));      
  
    PWorld = uniqueValid3DPoints(k,:);

    for r=1:length(allCameras)
        camID = cell2mat(allCameras(r));
        if(~isempty(camID))
      
            fid = fopen('/home/mina/City Dataset/info/groundtruth.txt');
            a = textscan(fid, '%s', 1, 'delimiter', '\n', 'headerlines', camID-1);
            b = str2num(cell2mat(strsplit(cell2mat(a{1,1}),'')));
            fclose(fid);
            cameraPose = b(2:end);
            cameraTranslation = cameraPose(1:3)';
            cameraRotation = quaternion2Axis([cameraPose(4);cameraPose(5);cameraPose(6);cameraPose(7)]);
            cameraPose = [cameraTranslation;cameraRotation];
            
            pointInCamFrame = AbsoluteToRelativePosition(cameraPose,PWorld(1,1:3)');

            if(norm(pointInCamFrame) > 500)
                continue
            end
            
            if(isempty(unique3DPoints))
                unique3DPoints = [unique3DPoints; PWorld];
            elseif (sum(unique3DPoints(:,1)==PWorld(1,1) & unique3DPoints(:,2)==PWorld(1,2) & unique3DPoints(:,3)==PWorld(1,3))==0)
                unique3DPoints = [unique3DPoints; PWorld];
            end
            
            [ptID,~] = find(unique3DPoints(:,1)==PWorld(1,1) & unique3DPoints(:,2)==PWorld(1,2) & unique3DPoints(:,3)==PWorld(1,3));
            
            fid = fopen('/home/mina/City Dataset/info/landmarkMeasGraphFile1.txt','a');
            fprintf(fid,'%d %d %s %s %s %6f %6f %6f',camID,ptID,'LANDMARK-MEAS3D','R3','EDGE',pointInCamFrame);
            fprintf(fid,'\n');
            fclose(fid);

        end
    end
 
end

% making sure landmarkMeasGraphFile has no duplicates lines and removing 
% them if needed
fid = fopen('/home/mina/City Dataset/info/landmarkMeasGraphFile1.txt');
a = textscan(fid,'%d %d %s %s %s %f %f %f','delimiter',' ');
[~,idx]=unique(strcat(num2str(cell2mat(a(:,1))),num2str(cell2mat(a(:,2)))),'rows','stable');
for i=1:length(idx)
    fid = fopen('/home/mina/City Dataset/info/landmarkMeasGraphFile1.txt');
    c = textscan(fid, '%s', 1, 'delimiter', '\n', 'headerlines', idx(i)-1);
    fid1 = fopen('/home/mina/City Dataset/info/landmarkMeasGraphFile.txt','a');
    fprintf(fid1,cell2mat(c{1,1}));
    fprintf(fid1,'\n');
    fclose(fid1);
    fclose(fid);
end


disp('Done writing landmarkMeasGraphFile');
save('/home/mina/City Dataset/Feature Extraction/3D Points/uniquePoints.mat','unique3DPoints');

end