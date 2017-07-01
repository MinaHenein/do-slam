function [obj] = constructAngleVertex(obj,config,edgeRow)
%CONSTRUCTANGLEVERTEX constructs vertex representing dot product between
%two planes

%% 1. load vars from edge row
edgeLabel = edgeRow{1};
edgeIndex = edgeRow{2};
planeVertices = edgeRow{3};
angleVertex = edgeRow{4};
edgeValue = edgeRow{5};
edgeCovariance = edgeRow{6};

%% 2. compute angle 
parameters = cell2mat({obj.vertices(planeVertices).value});
dotProduct = dot(parameters(1:3,1),parameters(1:3,2));

%% 3. vertex properties
value = dotProduct;
covariance = []; %not using this property yet
type = 'angle';
iEdges = [edgeIndex];
index = angleVertex;  

%% 4. construct vertex
obj.vertices(index) = Vertex(value,covariance,type,iEdges,index);

end

