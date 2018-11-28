function [obj] = constructSE3MotionVertex(obj,config,edgeRow,varargin)
%CONSTRUCTSE3MOTIONVERTEX constructs vertex representing SE3 motion of an object. 
%SE3 motion is initialised by extracting information from the same points at 
%different time steps. 
%As more 2points-SE3Motion measurements are made later, additional edges
%are added to this vertex.

%% 1. load vars from edge row
edgeLabel = edgeRow{1};
edgeIndex = edgeRow{2};
pointVertex = edgeRow{3};
SE3MotionVertex = edgeRow{4};
edgeValue = edgeRow{5};
edgeCovariance = edgeRow{6};

if strcmp(config.SE3MotionVertexInitialization,'translation') && ~isempty(varargin)
    pointVertices = varargin{1};
    pointVertices1 = pointVertices(1:2:end);
    pointVertices2 = pointVertices(2:2:end);
    poseVertices = obj.identifyVertices('pose');
    poseVertices = [poseVertices;inf];
    pointTimeVertices = cell(length(poseVertices)-1,2);
    for i=1:length(poseVertices)-1
        for j=1:length(pointVertices1)
            if pointVertices1(j) > poseVertices(i) && pointVertices1(j) < poseVertices(i+1)
                pointTimeVertices{i,1} = [pointTimeVertices{i,1},pointVertices1(j)];
            end
        end
        for j=1:length(pointVertices2)
            if pointVertices2(j) > poseVertices(i+1) && pointVertices2(j) < poseVertices(i+2)
                pointTimeVertices{i,2} = [pointTimeVertices{i,2},pointVertices2(j)];
            end
        end
    end
    rotations= {};
    translations = [];
    for i= 1:size(pointTimeVertices,1)
        points1 = [pointTimeVertices{i,1}];
        points2 = [pointTimeVertices{i,2}];
        if ~isempty(points1) && length(points1) == length(points2) && length(points1)>2
            pointPositions1 = cell2mat({obj.vertices(points1).value});
            pointPositions2 = cell2mat({obj.vertices(points2).value});
            [rotM,t,~] = Kabsch(pointPositions1, pointPositions2);
            rotations{i} = rotM;
            translations(:,i) = t;
        end
    end
    rotations = rotations(:,any(~cellfun('isempty',rotations),1));
    translations(:,~any(translations,1)) = [];
    R = rotationAveraging(rotations);
    t = mean(translations,2);
    SE3MotionValue = [t;arot(R)];
end

% pointPositions1 = cell2mat({obj.vertices(pointVertices(1:2:end)).value});
% pointPositions2 = cell2mat({obj.vertices(pointVertices(2:2:end)).value});
% [rotM,t] = Kabsch(pointPositions1, pointPositions2);

if strcmp(config.SE3MotionVertexInitialization,'eye')
    SE3Motion = [0 0 0 0 0 0]';
elseif strcmp(config.SE3MotionVertexInitialization,'translation') && ~isempty(varargin)
    SE3Motion = SE3MotionValue;
else
    SE3Motion = [0 0 0 0 0 0]';
    disp('using default motion vertex initialization as eye')
end

%% 3. vertex properties
value = SE3Motion;
covariance = []; %not using this property yet
type = 'SE3Motion';
iEdges = edgeIndex;
index = SE3MotionVertex(end);  

%% 4. construct vertex
obj.vertices(index) = Vertex(value,covariance,type,iEdges,SE3MotionVertex);
end

