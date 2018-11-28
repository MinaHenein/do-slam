function results = computeDynamicMapError(graphGT,graphN)
% poses
posesN = [graphN.vertices(graphN.identifyVertices('pose')).value];
posesGT = [graphGT.vertices(graphGT.identifyVertices('pose')).value];
v_rel_pose = AbsoluteToRelativePoseR3xso3(posesGT(:,1),posesN(:,1));

%points
SE3MotionVertices = [graphN.identifyVertices('SE3Motion')];
edgesConnectedToMotionVertices = [graphN.vertices(SE3MotionVertices).iEdges];
dynamicPointsMotionIndices = [graphN.edges(edgesConnectedToMotionVertices).iVertices]';
dynamicPointsIndices = setdiff(dynamicPointsMotionIndices,SE3MotionVertices);
dynamicPointsN = graphN.vertices(dynamicPointsIndices);

dynamicPointsGT = zeros(3,length(dynamicPointsN));
for i=1:length(dynamicPointsN)
   dynamicPointsGT(:,i) = [graphGT.vertices(dynamicPointsN(i).index).value];
end
dynamicPointsN = [graphN.vertices(dynamicPointsIndices).value];

T_point = poseToTransformationMatrix(v_rel_pose);
for i= 1:size(dynamicPointsN,2)
    point = T_point * [dynamicPointsN(1:3,i);1];
    dynamicPointsN(1:3,i) = point(1:3,1);
end

%% 1. Point Error
%ASE
[ASE_translation_error,ASE_squared_translation_error] = ...
    Compute_AbsoluteStructurePointsError(dynamicPointsN,dynamicPointsGT);
%RPTE
n_delta = 1;
[RPTE_translation_error, RPTE_squared_translation_error] = ...
    Compute_RelativePointError(dynamicPointsN,dynamicPointsGT,n_delta);
%AARPTE
[AARPTE_translation_error,AARPTE_squared_translation_error] = ...
    Compute_RelativePointError_AllToAll(dynamicPointsN,dynamicPointsGT);


results.ASE_translation_error               = ASE_translation_error;
results.ASE_squared_translation_error       = ASE_squared_translation_error;
results.RPTE_translation_error              = RPTE_translation_error;
results.RPTE_squared_translation_error      = RPTE_squared_translation_error;
results.AARPTE_translation_error            = AARPTE_translation_error;
results.AARPTE_squared_translation_error    = AARPTE_squared_translation_error;
end

