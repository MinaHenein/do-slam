function [obj] = constructSE3MotionVertex(obj,config,edgeRow)
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

[rotM,t] = Kabsch( obj.vertices(pointVertex(1)).value,...
    obj.vertices(pointVertex(2)).value);

if strcmp(config.SE3MotionVertexInitialization,'eye')
    SE3Motion = [0 0 0 0 0 0]';
elseif strcmp(config.SE3MotionVertexInitialization,'translation')
    SE3Motion = [t;arot(rotM)];
else 
    error('error, motion vertex initialization undefined')
end
    

%% 2. compute SE3Motion value
% nSteps = 0;
% objPtsAtTime = [];
% timeStep = 0;
% for i=1:obj.nVertices
%     if strcmp(obj.vertices(i).type,'pose')
%        nSteps = nSteps+1;
%        nPointsPerStep = 0;
%     end
%     if strcmp(obj.vertices(i).type,'point')
%         nPointsPerStep = nPointsPerStep +1;
%         objPtsAtTime = [objPtsAtTime, obj.vertices(i).value];
%     end
% end
% 
% if config.dimPoint==4
%     for i=1:size(objPtsAtTime,2)
%         objPtsAtTime(:,i) = objPtsAtTime(:,i)/objPtsAtTime(end,i); 
%     end
% end
% 
% for i=2:nSteps
%     [rotM,t] = Kabsch(objPtsAtTime(1:3,mapping(i,nPointsPerStep)),...
%         objPtsAtTime(1:3,mapping(i-1,nPointsPerStep)));
%     rotations{i-1} = rotM';
%     translations(:,i-1) = -rotM'*t;
% end
% 
% R = rotationAveraging(rotations);
% t = mean(translations,2);
% 
% SE3Motion = [t;arot(R)];

%% 3. vertex properties
value = SE3Motion;
covariance = []; %not using this property yet
type = 'SE3Motion';
iEdges = [edgeIndex];
index = SE3MotionVertex;  

%% 4. construct vertex
obj.vertices(index) = Vertex(value,covariance,type,iEdges,index);
end

