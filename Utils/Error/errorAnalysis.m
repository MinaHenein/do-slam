function [results] = errorAnalysis(config,graphGT,graphN)
%ERRORANALYSYS Summary of this function goes here
%   Detailed explanation goes here

%poses
posesN = [graphN.vertices(graphN.identifyVertices('pose')).value];
posesGT = [graphGT.vertices(graphGT.identifyVertices('pose')).value];
%convert to R3xSO3
if strcmp(config.poseParameterisation,'SE3')
    for i = 1:size(posesN,2)
        posesN(:,i) = LogSE3_Rxt(posesN(:,i));
        posesGT(:,i) = LogSE3_Rxt(posesGT(:,i));
    end
end

% [rotM, t, ~] = Kabsch(posesN(1:3,:),posesGT(1:3,:));
% v_rel_pose = [t;arot(rotM)];
v_rel_pose = AbsoluteToRelativePoseR3xso3(posesN(:,1),posesGT(:,1));

% T_pose =[rotM t; 0 0 0 1];

% for i= 1:size(posesN,2)
%     pose = T_pose * [rot(posesN(4:6,i)) posesN(1:3,i); 0 0 0 1];
%     posesN(:,i) = [pose(1:3,4); arot(pose(1:3,1:3))];
% end


%points
pointsN = [graphN.vertices(graphN.identifyVertices('point')).value];
pointsGT = [graphGT.vertices(graphGT.identifyVertices('point')).value];

[rotM, t, ~] = Kabsch(pointsN(1:3,:),pointsGT(1:3,:));
T_point = [rotM t; 0 0 0 1];
for i= 1:size(pointsN,2)
    point = T_point * [pointsN(1:3,i);1];
    pointsN(1:3,i) = point(1:3,1);
end



% %planes
% planesN = [graphN.vertices(graphN.identifyVertices('plane')).value];
% planesGT = [graphGT.vertices(graphGT.identifyVertices('plane')).value];
%
% %points in each plane
% iPlanesN = graphN.identifyVertices('plane');
% iPlanesGT = graphN.identifyVertices('plane');
% for i = 1:numel(iPlanesN)
%     iPlaneN = iPlanesN(i);
%     iPointVerticesN = graphN.getConnectedVertices(iPlaneN,'point');
%     pointPositionsN = [graphN.vertices(iPointVerticesN).value];
%     iPlaneGT = iPlanesGT(i);
%     iPointVerticesGT = graphGT.getConnectedVertices(iPlaneGT,'point');
%     pointPositionsGT = [graphGT.vertices(iPointVerticesGT).value];
% end

%% 1. Pose Error
%ATE
[ATE_translation_error,ATE_rotation_error,...
    ATE_squared_translation_error,ATE_squared_rotation_error] = ...
    Compute_AbsoluteTrajectoryError(posesN,posesGT,v_rel_pose);
%RPE
n_delta = 1;
[RPE_translation_error,RPE_rotation_error,...
    RPE_squared_translation_error,RPE_squared_rotation_error] = ...
    Compute_RelativePoseError(posesN,posesGT,v_rel_pose,n_delta);
%AARPE
[AARPE_translation_error,AARPE_rotation_error,AARPE_squared_translation_error,...
    AARPE_squared_rotation_error] = ...
    Compute_RelativePoseError_AllToAll(posesN,posesGT,v_rel_pose);

%% 1. Point Error
%ASE
[ASE_translation_error,ASE_squared_translation_error] = ...
    Compute_AbsoluteStructurePointsError(pointsN,pointsGT);
%RPTE
n_delta = 1;
[RPTE_translation_error, RPTE_squared_translation_error] = ...
    Compute_RelativePointError(pointsN,pointsGT,n_delta);
%AARPTE
[AARPTE_translation_error,AARPTE_squared_translation_error] = ...
    Compute_RelativePointError_AllToAll(pointsN,pointsGT);


results.ATE_translation_error               = ATE_translation_error;
results.ATE_rotation_error                  = ATE_rotation_error;
results.ATE_squared_translation_error       = ATE_squared_translation_error;
results.ATE_squared_rotation_error          = ATE_squared_rotation_error;
results.RPE_translation_error               = RPE_translation_error;
results.RPE_rotation_error                  = RPE_rotation_error;
results.RPE_squared_translation_error       = RPE_squared_translation_error;
results.RPE_squared_rotation_error          = RPE_squared_rotation_error;
results.AARPE_translation_error             = AARPE_translation_error;
results.AARPE_rotation_error                = AARPE_rotation_error;
results.AARPE_squared_translation_error     = AARPE_squared_translation_error;
results.AARPE_squared_rotation_error        = AARPE_squared_rotation_error;
results.ASE_translation_error               = ASE_translation_error;
results.ASE_squared_translation_error       = ASE_squared_translation_error;
results.RPTE_translation_error              = RPTE_translation_error;
results.RPTE_squared_translation_error      = RPTE_squared_translation_error;
results.AARPTE_translation_error            = AARPTE_translation_error;
results.AARPTE_squared_translation_error    = AARPTE_squared_translation_error;
% results = [ATE_translation_error,ATE_rotation_error,...
%     ATE_squared_translation_error,ATE_squared_rotation_error,...
%     RPE_translation_error,RPE_rotation_error,...
%     RPE_squared_translation_error,RPE_squared_rotation_error,...
%     AARPE_translation_error,AARPE_rotation_error,AARPE_squared_translation_error,...
%     AARPE_squared_rotation_error,ASE_translation_error,ASE_squared_translation_error,...
%     RPTE_translation_error, RPTE_squared_translation_error,...
%     AARPTE_translation_error,AARPTE_squared_translation_error];


%ATE
% X = ['Cumulative ATE of the aligned poses =  ', num2str(ATE_translation_error),' , ',num2str(ATE_rotation_error)];
% disp(X);
% X = ['Per vertex ATE of the aligned poses =  ', num2str(ATE_translation_error/nPoses),' , ',num2str(ATE_rotation_error/nPoses)];
% disp(X);
% X = ['RMS ATE of the aligned poses =  ', num2str(sqrt(ATE_squared_translation_error)),' , ',num2str(sqrt(ATE_squared_rotation_error))];
% disp(X);
% %RPE
% X = ['Cumulative RPE of the aligned poses =  ', num2str(RPE_translation_error),' , ',num2str(RPE_rotation_error)];
% disp(X);
% X = ['Per vertex RPE of the aligned poses =  ', num2str(RPE_translation_error/nPoses),' , ',num2str(RPE_rotation_error/nPoses)];
% disp(X);
% X = ['RMS RPE of the aligned poses =  ', num2str(sqrt(RPE_squared_translation_error)),' , ',num2str(sqrt(RPE_squared_rotation_error))];
% disp(X);
% %AARPE
% X = ['Cumulative AARPE all-to-all of the aligned poses =  ', num2str(AARPE_translation_error),' , ',num2str(AARPE_rotation_error)];
% disp(X);
% X = ['Per vertex AARPE all-to-all of the aligned poses =  ', num2str(AARPE_translation_error/nPoses),' , ',num2str(AARPE_rotation_error/nPoses)];
% disp(X);
% X = ['RMS AARPE all-to-all of the aligned poses =  ', num2str(sqrt(AARPE_squared_translation_error)),' , ',num2str(sqrt(AARPE_squared_rotation_error))];
% disp(X);
% 
% %ASE
% X = ['Cumulative ASE of the aligned points =  ', num2str(ASE_translation_error)];
% disp(X);
% X = ['Per vertex ASE of the aligned points =  ', num2str(ASE_translation_error/nPoints)];
% disp(X);
% X = ['RMS ASE of the aligned points =  ', num2str(sqrt(ASE_squared_translation_error))];
% disp(X);
% %RPTE
% X = ['Cumulative RPTE of the aligned points =  ', num2str(RPTE_translation_error)];
% disp(X);
% X = ['Per vertex RPTE of the aligned points =  ', num2str(RPTE_translation_error/nPoints)];
% disp(X);
% X = ['RMS RPTE of the aligned points =  ', num2str(sqrt(RPTE_squared_translation_error))];
% disp(X);
% %AARPTE
% X = ['Cumulative AARPTE all-to-all of the aligned points =  ', num2str(AARPTE_translation_error)];
% disp(X);
% X = ['Per vertex AARPTE all-to-all of the aligned points =  ', num2str(AARPTE_translation_error/nPoints)];
% disp(X);
% X = ['RMS AARPTE all-to-all of the aligned points =  ', num2str(sqrt(AARPTE_squared_translation_error))];
% disp(X);

%% debug
fprintf('Absolute Trajectory Translation Error: %.4d \n',results.ATE_translation_error)
fprintf('Absolute Trajectory Rotation Error: %.4d \n',results.ATE_rotation_error)
fprintf('Absolute Structure Points Error: %d \n',results.ASE_translation_error);
fprintf('All to All Relative Pose Squared Translation Error: %.4d \n',results.AARPE_squared_translation_error)
fprintf('All to All Relative Pose Squared Rotation Error: %.4d \n',results.AARPE_squared_rotation_error)
fprintf('All to All Relative Point Squared Translation Error: %.4d \n',results.AARPTE_squared_translation_error)

end

