function [connectedTypeVertices] = getConnectedVertices(obj,iVertex,type)
%GETCONNECTEDVERTICES returns indexes of vertices connected to 'iVertex' 
%that have type 'type'
%   Detailed explanation goes here

vertex = obj.vertices(iVertex);
connectedEdges = vertex.iEdges;
connectedVertices = [obj.edges(connectedEdges).iVertices]';
connectedVertices = unique(connectedVertices);
allTypeVertices = obj.identifyVertices(type);
connectedTypeVertices = intersect(allTypeVertices,connectedVertices);                  
%remove iVertex in case same type
connectedTypeVertices = setdiff(connectedTypeVertices,iVertex);

end

