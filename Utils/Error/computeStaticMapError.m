function results = computeStaticMapError(graphGT,graphN)

%points
SE3MotionVertices = [graphN.identifyVertices('SE3Motion')];
edgesConnectedToMotionVertices = [graphN.vertices(SE3MotionVertices).iEdges];
dynamicPointsMotionIndices = [graphN.edges(edgesConnectedToMotionVertices).iVertices]';
dynamicPointsIndices = setdiff(dynamicPointsMotionIndices,SE3MotionVertices);
allPointsIndices = [graphN.identifyVertices('point')];
staticPointsN = [graphN.vertices(setdiff(allPointsIndices,dynamicPointsIndices))];

staticPointsGT = zeros(3,length(staticPointsN));
for i=1:length(staticPointsN)
   staticPointsGT(:,i) = [graphGT.vertices(staticPointsN(i).index).value];
end
staticPointsN = [graphN.vertices(setdiff(allPointsIndices,dynamicPointsIndices)).value];


[rotM, t, ~] = Kabsch(staticPointsN(1:3,:),staticPointsGT(1:3,:));
T_point = [rotM t; 0 0 0 1];
for i= 1:size(staticPointsN,2)
    point = T_point * [staticPointsN(1:3,i);1];
    staticPointsN(1:3,i) = point(1:3,1);
end

%% 1. Point Error
%ASE
[ASE_translation_error,ASE_squared_translation_error] = ...
    Compute_AbsoluteStructurePointsError(staticPointsN,staticPointsGT);
%RPTE
n_delta = 1;
[RPTE_translation_error, RPTE_squared_translation_error] = ...
    Compute_RelativePointError(staticPointsN,staticPointsGT,n_delta);
%AARPTE
[AARPTE_translation_error,AARPTE_squared_translation_error] = ...
    Compute_RelativePointError_AllToAll(staticPointsN,staticPointsGT);


results.ASE_translation_error               = ASE_translation_error;
results.ASE_squared_translation_error       = ASE_squared_translation_error;
results.RPTE_translation_error              = RPTE_translation_error;
results.RPTE_squared_translation_error      = RPTE_squared_translation_error;
results.AARPTE_translation_error            = AARPTE_translation_error;
results.AARPTE_squared_translation_error    = AARPTE_squared_translation_error;
end

