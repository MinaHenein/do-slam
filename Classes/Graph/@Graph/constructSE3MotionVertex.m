function [obj] = constructSE3MotionVertex(obj,config,edgeRow,pointVertices)
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

pointPositions1 = cell2mat({obj.vertices(pointVertices(1:2:end)).value});
pointPositions2 = cell2mat({obj.vertices(pointVertices(2:2:end)).value});
[rotM,t] = Kabsch(pointPositions1, pointPositions2);

if strcmp(config.SE3MotionVertexInitialization,'eye')
    SE3Motion = [0 0 0 0 0 0]';
elseif strcmp(config.SE3MotionVertexInitialization,'translation')
    SE3Motion = [t;arot(rotM)];
else 
    error('error, motion vertex initialization undefined')
end
    
%% 3. vertex properties
value = SE3Motion;
covariance = []; %not using this property yet
type = 'SE3Motion';
iEdges = [edgeIndex];
index = SE3MotionVertex;  

%% 4. construct vertex
obj.vertices(index) = Vertex(value,covariance,type,iEdges,index);
end

