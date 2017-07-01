function [obj] = constructPlaneVertex(obj,config,edgeRow,pointVertices)
%CONSTRUCTPLANEVERTEX constructs vertex representing plane. Plane
%parameters initialised by fitting plane to points used to initialise
%plane. As more point-plane measurements are made later, additional edges
%are added to this vertex.

%% 1. load vars from edge row
edgeLabel = edgeRow{1};
edgeIndex = edgeRow{2};
pointVertex = edgeRow{3};
planeVertex = edgeRow{4};
edgeValue = edgeRow{5};
edgeCovariance = edgeRow{6};

%% 2. compute plane parameters
pointPositions = cell2mat({obj.vertices(pointVertices).value});
[fit,parameters] = fitPlane(pointPositions);
assert(logical(fit),'could not fit plane to points')

%% 3. vertex properties
value = parameters;
covariance = []; %not using this property yet
type = 'plane';
iEdges = [edgeIndex];
index = planeVertex;  

%% 4. construct vertex
obj.vertices(index) = Vertex(value,covariance,type,iEdges,index);

end

