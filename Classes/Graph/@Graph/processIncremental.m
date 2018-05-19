function [solver] = processIncremental(obj,config,measurementsCell,groundTruthCell)
%PROCESSINCREMENTAL Incrementally processes measurements, builds linear
%system and solves
%   At each time step, measurements are processed and vertices and edges
%   are added to the graph.
%   When enough vertices or edges are added, OR a number of time steps have
%   occurred, the system is optimised.
%   This repeats until all measurements have been processed

%% 1. Plot incrementally
if config.plotIncremental
    %movie
    frames(config.nSteps) = struct('cdata',[],'colormap',[]);

    %plot ground truth
    fig = figure;
    hold on
    %if config.axisEqual; axis equal; end
    %axis(config.axisLimits)
    %view(config.plotView)
    xlabel('x')
    ylabel('y')
    zlabel('z')
    plotGraphFile(config,groundTruthCell,[0 1 0]);
    %plotGraphFileICRA(config,groundTruthCell,'groundTruth');
    
end
    
%% 2. Adjust measurementsCell
%convert each element of measurementsCell to a row
measurementsCell = reshapeCell(measurementsCell,'array');

%create prior
%find odometry rows
odometryRows = find(strcmp({measurementsCell{:,1}}',config.posePoseEdgeLabel));
odometryIndex = 1; %first pose
poseRows = [];
for i = 1:numel(groundTruthCell)
    if strcmp(groundTruthCell{i}{1},config.poseVertexLabel)
        poseRows = [poseRows,i];
    end
end
if ~isempty(odometryRows)
    startPoseVertex = measurementsCell{odometryRows(odometryIndex),3};
else
    startPoseVertex = groundTruthCell{poseRows(odometryIndex)}{2};
end
startPoseValue = groundTruthCell{poseRows(odometryIndex)}{3};
startPoseCovariance = config.covPosePrior;
priorLine = {config.posePriorEdgeLabel,1,[],startPoseVertex,startPoseValue,startPoseCovariance};

%add prior line
measurementsCell = vertcat(priorLine,measurementsCell);

%% 3. Construct vertices and edges at each time step, solve system and update graph
nVertices = max([measurementsCell{:,4}]);
nEdges = size(measurementsCell,1);

%indexing
iPoseVertices = [];
iPointVertices = [];
iEntityVertices = [];
iObjectVertices = [];

%solving
skipCount = 0;
vertexCount = 0;
edgeCount = 0;

%store each step
solver = [];

%loop over nSteps
nSteps = numel(odometryRows) + 1;
for i = 1:nSteps
    %identify rows from this time step
    %add elements so formula for iRows works for first and last steps
    odometryRows = [1; find(strcmp(measurementsCell(:,1),config.posePoseEdgeLabel)); size(measurementsCell,1)+1];
    iRows = odometryRows(i):odometryRows(i+1)-1;
    nRows = numel(iRows);
    
    %loop over rows
    for j = 1:nRows
        jRow = measurementsCell(iRows(j),:);
        switch jRow{1}
            case config.posePriorEdgeLabel %posePrior
                %edge index
                jRow{2} = obj.nEdges+1;
                %construct pose vertex
                obj = obj.constructPoseVertex(config,jRow);
                %construct prior edge
                obj = obj.constructPosePriorEdge(config,jRow);
            case config.posePoseEdgeLabel %odometry
                %edge index
                jRow{2} = obj.nEdges+1;
                %construct pose vertex
                obj = obj.constructPoseVertex(config,jRow);
                %construct pose-pose edge
                obj = obj.constructPosePoseEdge(config,jRow);
            case config.posePointEdgeLabel
                %edge index
                jRow{2} = obj.nEdges+1;
                %create point vertex if it doesn't exist
                if jRow{4} > obj.nVertices
                    obj = obj.constructPointVertex(config,jRow);
                end
                %construct pose-point edge
                obj = obj.constructPosePointEdge(config,jRow);
                
            case config.pointPointEdgeLabel
                %edgeIndex
                jRow{2} = obj.nEdges+1;
                % construct point-point edge - both points should already exist
                obj = obj.constructPointPointEdge(config,jRow);
            case config.pointPlaneEdgeLabel
                %edge index
                jRow{2} = obj.nEdges+1;
                %create plane vertex if it doesn't exist
                if jRow{4} > obj.nVertices
                    %find all point vertices connected to this plane
                    pointRows = iRows([measurementsCell{iRows,4}]==jRow{4});
                    pointVertices = [measurementsCell{pointRows,3}]';
                    obj = obj.constructPlaneVertex(config,jRow,pointVertices);                   
                end
                %construct point-plane edge
                obj = obj.constructPointPlaneEdge(config,jRow);
            
            case config.pointSE3MotionEdgeLabel
                %edge index
                jRow{2} = obj.nEdges+1;
                %create velocity vertex if it doesn't exist
                if jRow{4} > obj.nVertices
                    %find all point vertices connected to this SE3 vertex
                    pointRows = iRows([measurementsCell{iRows,4}]==jRow{4});
                    pointVertices = [measurementsCell{pointRows,3}]';
                    obj = obj.constructSE3MotionVertex(config,jRow,pointVertices);
                end
                obj = obj.construct2PointsSE3MotionEdge(config,jRow);
            
            case config.planePriorEdgeLabel
                %edge index
                jRow{2} = obj.nEdges+1;
                %construct plane vertex
                %find all point vertices connected to this plane
                pointRows = iRows([measurementsCell{iRows,4}]==jRow{3});
                pointVertices = [measurementsCell{pointRows,3}]';
                %remove plane vertex that doesn't exist
                pointVertices(pointVertices>obj.nVertices) = [];
                %adjust jRow - constructor requires plane as output vertex
                jRow{4} = jRow{3};
                obj = obj.constructPlaneVertex(config,jRow,pointVertices);
                
                %construct plane prior edge
                iPlaneVertex = jRow{3};
                planeNormal = obj.vertices(iPlaneVertex).value(1:3);
                %edge value
                jRow{5} = planeNormal'*planeNormal - 1;
                %edge covariance
                jRow{6} = config.covPlaneNormal;
                obj = obj.constructPlanePriorEdge(config,jRow);
            case config.angleEdgeLabel
                %edge index
                jRow{2} = obj.nEdges+1;
                %create angle edge if it doesn't exist
                if jRow{4} > obj.nVertices
                    obj = obj.constructAngleVertex(config,jRow);
                end
                obj = obj.constructAngleEdge(config,jRow);
            case config.fixedAngleEdgeLabel
                %edge index
                jRow{2} = obj.nEdges+1;
                obj = obj.constructFixedAngleEdge(config,jRow);
            case config.distanceEdgeLabel
                %edge index
                jRow{2} = obj.nEdges+1;
                if jRow{4} > obj.nVertices
                    obj = obj.constructDistanceVertex(config,jRow);
                end
                obj = constructDistanceEdge(config,jRow);
            case config.fixedDistanceEdgeLabel
                %edge index
                jRow{2} = obj.nEdges+1;
                obj = obj.constructFixedDistanceEdge(config,jRow);
            otherwise; error('%s type invalid',label)
        end
        %construct edge
    end
    
    %counts
    nNewVertices = obj.nVertices - vertexCount;
    nNewEdges    = obj.nEdges - edgeCount;
    
    %solve?
    if (nNewVertices > config.nVerticesThreshold) || (nNewEdges > config.nEdgesThreshold) ||...
        (skipCount>=config.solveRate-1) || (i==nSteps) %|| (~mod(i,config.solveRate))
   
        %will solve
        skipCount = 0;
        vertexCount = obj.nVertices;
        edgeCount = obj.nEdges;
        
        %adjust angle constraints
        if config.automaticAngleConstraints
            [obj,measurementsCell] = obj.adjustAngleConstraints(measurementsCell);
        end

        %reorder vertices and edges
        measurementsCellCurrent = measurementsCell;
        if config.sortVertices
            [obj,newToOldVertices,measurementsCellCurrent] = obj.sortVertices(measurementsCellCurrent);
        end
        if config.sortEdges
            [obj,newToOldEdges,measurementsCellCurrent] = obj.sortEdges(measurementsCellCurrent);
        end
        
        %display progress
        if config.displayProgress
            fprintf('\n----------------------------------\n')
            fprintf('Time step:\t%d/%d\n',i,nSteps)
            fprintf('Vertices:\t%d/%d\n',obj.nVertices,nVertices)
            fprintf('Edges:\t\t%d/%d\n',obj.nEdges,nEdges)
        end
        
        %construct linear system, solve
        iSolver = NonlinearSolver(config);
        iSolver = iSolver.solve(config,obj,measurementsCellCurrent);
        obj  = iSolver.graphs(end);%update graph
        
        %undo reordering *TODO - only do this at the end
        if config.sortEdges
            [obj] = obj.unsortEdges(newToOldEdges);
        end
        if config.sortVertices
            [obj] = obj.unsortVertices(newToOldVertices);
        end
                
        %store iSolver
        solver = [solver iSolver];
%         solver = iSolver; %if memory is problem
    else
        skipCount = skipCount + 1;
    end
       
    %plot while solving
    if config.plotIncremental
        if any(strcmp(who,'currentPlotHandle'))
            for j = 1:numel(currentPlotHandle)
                delete(currentPlotHandle{j});
            end
        else

        end
        currentPlotHandle = plotGraph(config,obj,[0 0 1]);
        %currentPlotHandle = plotGraphFileICRA(config,obj,'solverResults');
        frames(i) = getframe(fig);
    end
end

% store animation
if config.plotIncremental
    solver(end).frames = frames;
end

end

