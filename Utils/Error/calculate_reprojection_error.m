function reprojectionError = calculate_reprojection_error(config,graphGT,graphN)

pointsN = [graphN.vertices(graphN.identifyVertices('point'))];

reprojectionError = zeros(2,1);
for i=1:size(pointsN,2)
    %find cameras that see this point
    pointVertex = pointsN(i);
    edgeIDs = pointVertex.iEdges;
    % get camera pose
    for j=1:length(edgeIDs)
        cameraPointID = [graphN.edges(edgeIDs(j)).iVertices];
        cameraID = setdiff(cameraPointID,pointVertex.index);
        cameraPose = graphN.vertices(cameraID).value;
        % get point position
        pointPosition = pointVertex.value;
        % get pixel location
        pointImageFrame = AbsoluteToRelativePositionR3xso3Image(cameraPose,...
            pointPosition,config.intrinsics,config.R);
        % get GT pixel location
        cameraGTPose = graphGT.vertices([graphGT.vertices.index]==cameraID).value;
        pointGTPosition  = graphGT.vertices([graphGT.vertices.index]==pointVertex.index).value;
        pointGTImageFrame = AbsoluteToRelativePositionR3xso3Image(cameraGTPose,...
            pointGTPosition,config.intrinsics,config.R);
        % calculate reprojectionr  error
        reprojectionError = reprojectionError + abs(pointGTImageFrame - pointImageFrame);
    end
end

reprojectionError = reprojectionError/size(pointsN,2);
disp(reprojectionError(1));disp(reprojectionError(2));

end