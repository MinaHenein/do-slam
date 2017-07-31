function [obj] = constructVelocityVertex_v2(obj,config,edgeRow,pointVertices)
%CONSTRUCTVELOCITYVERTEX constructs vertex representing velocity. Velocity
%is initialised by extracting velocity information from the same points at 
%different time steps used to initialise the velocity vertex. 
%As more point-velocity measurements are made later, additional edges
%are added to this vertex.

%% 1. load vars from edge row
edgeLabel = edgeRow{1};
edgeIndex = edgeRow{2};
pointVertex = edgeRow{3};
velocityVertex = edgeRow{4};
edgeValue = edgeRow{5};
edgeCovariance = edgeRow{6};
%% 2. compute plane parameters
pointPositions = cell2mat({obj.vertices(pointVertices).value});
nPoints = size(pointPositions,2); 
velocities = zeros(3,nPoints-1);
for i=2:nPoints
velocities(:,i-1) = pointPositions(:,i)-pointPositions(:,i-1);
end
velocity = mean(velocities,2);
%% 3. vertex properties
value = velocity;
covariance = []; %not using this property yet
type = 'velocity';
iEdges = [edgeIndex];
index = velocityVertex;  

%% 4. construct vertex
obj.vertices(index) = Vertex(value,covariance,type,iEdges,index);
end

