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
        case 'point-pointSE3'
            edgeUpdater = @updatePointPointEdgeSE3;
        case 'point-pointSE3_Mina'
            edgeUpdater = @updatePointPointEdgeSE3_Mina;
        case '3-points'
            if strcmp(config.motionModel,'constantSpeed')
                edgeUpdater = @update3PointsEdge;
            elseif strcmp(config.motionModel,'constantVelocity')
                edgeUpdater = @update3PointsEdge_v2;
            else
                error('Motion model not implemented');
            end
        case '2points-velocity'
            if strcmp(config.motionModel,'constantSpeed')
                edgeUpdater = @update2PointsVelocityEdge;
            elseif strcmp(config.motionModel,'constantVelocity')
                edgeUpdater = @update2PointsVelocityEdge_v2;
            else
                error('Motion model not implemented');
            end
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

