function writeLandmarkMeas(pointsMeasurements,pointsLabels,pointsCameras,pointMeasCov)

disp('Writing landmarkMeasGraphFile ...');
staticPointSeen = 0;
nTimesStaticPointSeen = 0;
for i = 1:size(pointsMeasurements,1)
    camID = pointsCameras(i,1);
    ptLabel = pointsLabels(i,1);
    ptID = i;
    if ptLabel == 5 && ~staticPointSeen 
        staticPointSeen = 1;
        staticPointID = ptID;
        nTimesStaticPointSeen = nTimesStaticPointSeen +1;
        ptIdx = staticPointID;
    elseif ptLabel == 5 && staticPointSeen
        ptIdx = staticPointID;
        nTimesStaticPointSeen = nTimesStaticPointSeen +1;
    elseif ptLabel ~= 5
        ptIdx = ptID - (nTimesStaticPointSeen-1);
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