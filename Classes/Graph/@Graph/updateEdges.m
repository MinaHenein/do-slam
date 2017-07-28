function [obj] = updateEdges(obj,config)
%UPDATEEDGES recalculates edges of graph from vertices
%   Detailed explanation goes here

%loop over edges, call update function
for i = 1:obj.nEdges
    switch obj.edges(i).type
        case 'posePrior'
            edgeUpdater = @updatePosePriorEdge;
        case 'pose-pose'
            edgeUpdater = @updatePosePoseEdge;
        case 'pose-point'
            edgeUpdater = @updatePosePointEdge;
        case 'point-point'
            edgeUpdater = @updatePointPointEdge;
        case '3-points'
            edgeUpdater = @update3PointsEdge;
        case '2points-velocity'
            edgeUpdater = @update2PointsVelocityEdge;
        case 'point-plane'
            edgeUpdater = @updatePointPlaneEdge;
        case 'point-rectangle'
            edgeUpdater = @updatePointRectangleEdge;
        case 'point-cube'
            edgeUpdater = @updatePointCubeEdge;
        case 'plane-cube'
            edgeUpdater = @updatePlaneCubeEdge;
        case 'plane-plane-angle'
            edgeUpdater = @updateAngleEdge;
        case 'plane-plane-fixedAngle'
            edgeUpdater = @updateFixedAngleEdge;
        case 'plane-plane-distance'
            edgeUpdater = @updateDistanceEdge;
        case 'plane-plane-fixedDistance'
            edgeUpdater = @updateFixedDistanceEdge;
        case 'planePrior'
            edgeUpdater = @updatePlanePriorEdge;
        otherwise
            error('error: %s edge type not implemented',obj.edges(i).type)
    end
    obj = edgeUpdater(obj,config,i);
end


end

