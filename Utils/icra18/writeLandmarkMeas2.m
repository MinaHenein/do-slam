function writeLandmarkMeas2(pointsMeasurements,pointsLabels,pointsCameras,pointMeasCov)

disp('Writing landmarkMeasGraphFile ...');
staticPointsSeen = zeros(8,1);
staticPointsIDs = zeros(8,1);
id = 0;

for i = 1:size(pointsMeasurements,1)
    camID = pointsCameras(i,1);
    ptLabel = pointsLabels(i,1);
    ptID = i;
    if ptLabel >= 5 && ~staticPointsSeen(ptLabel-4,1)
        staticPointsSeen(ptLabel-4,1) = 1;
        staticPointsIDs(ptLabel-4,1) = ptID;
        id = id+1;
        ptIdx = staticPointsIDs(ptLabel-4,1);
    elseif ptLabel >= 5 && staticPointsSeen(ptLabel-4,1)
        ptIdx = staticPointsIDs(ptLabel-4,1);
    elseif ptLabel < 5
        id = id +1;
        ptIdx = id;
    end
    pCameraFrame = [pointsMeasurements(i,3),-pointsMeasurements(i,1),...
        -pointsMeasurements(i,2)];
    fid = fopen('/home/mina/workspace/src/Git/do-slam/Utils/icra18/landmarkMeasGraphFile.txt','a');
    fprintf(fid,'%s %d %d %d %6f %6f %6f %6f %6f %6f %6f %6f %6f \n','EDGE_3D',...
        camID,ptLabel,ptIdx,pCameraFrame,pointMeasCov);
    fclose(fid);
end

disp('Done writing landmarkMeasGraphFile');

end