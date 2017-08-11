function [residual] = computeResidual(obj,config,iEdge,measurement)
%COMPUTERESIDUAL computes residual from an edge
%   Depends on edge type. Residual computed from edge value and observation
%   value.

switch obj.edges(iEdge).type
    case 'posePrior'
        residual = config.absoluteToRelativePoseHandle(measurement,obj.edges(iEdge).value);
%         residual = AbsoluteToRelativePose(measurement,obj.edges(iEdge).value);
%         residual = Absolute2RelativeSE3(measurement,obj.edges(iEdge).value);
    case 'pose-pose'
        pose1 = obj.vertices(obj.edges(iEdge).iVertices(1)).value;
        pose2 = obj.vertices(obj.edges(iEdge).iVertices(2)).value;
        measurementPredicted = config.absoluteToRelativePoseHandle(pose1,pose2);
%         pose2Predicted = config.relativeToAbsolutePoseHandle(pose1,measurement);
%         pose2Predicted = RelativeToAbsolutePose(pose1,measurement);
%         pose2Predicted = Relative2AbsoluteSE3(pose1,measurement);
%         residual = config.absoluteToRelativePoseHandle(pose2Predicted,pose2);
%         residual = AbsoluteToRelativePose(pose2Predicted,pose2);
%         residual = Absolute2RelativeSE3(pose2Predicted,pose2);
        residual = config.absoluteToRelativePoseHandle(measurementPredicted,measurement);
%         residual = config.absoluteToRelativePoseHandle(measurement,measurementPredicted);
    case 'pose-point'
        residual = measurement - obj.edges(iEdge).value;
    case 'point-point'
        residual = measurement - obj.edges(iEdge).value;
    case {'point-pointSE3','point-pointSE3_Mina'}
        residual = measurement - obj.edges(iEdge).value;
    case '3-points'
        residual = measurement - obj.edges(iEdge).value;
    case '2points-velocity'
        residual = measurement - obj.edges(iEdge).value;
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
    