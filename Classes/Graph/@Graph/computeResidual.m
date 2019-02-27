function [residual] = computeResidual(obj,config,iEdge,measurement)
%COMPUTERESIDUAL computes residual from an edge
%   Depends on edge type. Residual computed from edge value and observation
%   value.
switch obj.edges(iEdge).type
    case 'posePrior'
        residual = config.absoluteToRelativePoseHandle(measurement,obj.edges(iEdge).value);
    case 'pose-pose'
        residual = config.absoluteToRelativePoseHandle(obj.edges(iEdge).value,measurement);
    case 'SE3Motion-SE3Motion'
        residual = config.absoluteToRelativePoseHandle(obj.edges(iEdge).value,zeros(6,1));
    case {'pose-point','pose-point-intrinsic'}
        if strcmp(config.landmarkErrorToMinimize,'reprojectionKnownIntrinsics')
            residual = measurement(4:5) - obj.edges(iEdge).value;
        else
            residual = measurement - obj.edges(iEdge).value;
        end
    case 'point-point'
        residual = measurement - obj.edges(iEdge).value;
    case {'point-pointSE3','point-pointSE3_Mina'}
        residual = measurement - obj.edges(iEdge).value;
    case '3-points'
        residual = measurement - obj.edges(iEdge).value;
    case '2points-velocity'
        %         residual = measurement - obj.edges(iEdge).value;
        residual = zeros(3,1) - obj.edges(iEdge).value;
    case '2points-SE3Motion'
%         residual = measurement - obj.edges(iEdge).value;
        residual = zeros(3,1) - obj.edges(iEdge).value;
    case 'point-plane'
        residual = measurement - obj.edges(iEdge).value;
    case 'point-rectangle'
        residual = obj.edges(iEdge).value;
    case 'point-cube'
        residual = obj.edges(iEdge).value;
    case 'plane-cube'
        residual = obj.edges(iEdge).value;
    case 'plane-plane-angle'
        residual = measurement - obj.edges(iEdge).value;
    case 'plane-plane-fixedAngle'
        residual = measurement - obj.edges(iEdge).value;
    case 'plane-plane-distance'
        residual = measurement - obj.edges(iEdge).value;
        if residual > 1
            residual = 2 - residual;
        end
    case 'plane-plane-fixedDistance'
        residual = obj.edges(iEdge).value;
    case 'planePrior'
        residual = 0 - obj.edges(iEdge).value;
    otherwise
        error('error: %s edge type not implemented',obj.edges(iEdge).type)
end
end
