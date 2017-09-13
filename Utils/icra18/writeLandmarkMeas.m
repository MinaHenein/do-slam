function writeLandmarkMeas(pointsMeasurements,pointsLabels,pointsCameras)

disp('Writing landmarkMeasGraphFile ...');
% 0.4 m
pointMeasCov = [0.16 0.0 0.0 0.16 0.0 0.16];
staticPointSeen = 0;
nTimesStaticPoitnSeen = 0;
for i = 1:size(pointsMeasurements,1)
    camID = pointsCameras(i,1);
    ptLabel = pointsLabels(i,1);
    ptID = i;
    if ptLabel == 5 && ~staticPointSeen 
        staticPointSeen = 1;
        staticPointID = ptID;
        nTimesStaticPoitnSeen = nTimesStaticPoitnSeen +1;
        ptIdx = staticPointID;
    elseif ptLabel == 5 && staticPointSeen
        ptIdx = staticPointID;
        nTimesStaticPoitnSeen = nTimesStaticPoitnSeen +1;
    elseif ptLabel ~= 5
        ptIdx = ptID - (nTimesStaticPoitnSeen-1);
    end
    fid = fopen('/home/mina/workspace/src/Git/do-slam/Utils/icra18/landmarkMeasGraphFile.txt','a');
    fprintf(fid,'%s %d %d %d %6f %6f %6f %6f %6f %6f %6f %6f %6f \n','EDGE_3D',...
        camID,ptLabel,ptIdx,pointsMeasurements(i,:),pointMeasCov);
    fclose(fid);
end
 
disp('Done writing landmarkMeasGraphFile');

end