function [dX] = constrainedOptimisation(graph,system)
%CONSTRAINEDOPTIMISATION Summary of this function goes here
%   Detailed explanation goes here

C = sparse(size(system.A,1),size(system.A,2));
d = sparse(length(system.b),1);
E = sparse(size(system.A,1),size(system.A,2));
f = sparse(size(system.b,1),1);

iPlaneVertices = identifyVertices(graph,'plane');
iPointPlaneEdges = identifyEdges(graph,'point-plane');
iAngleEdges    = identifyEdges(graph,'plane-plane-fixedAngle');

%construct C
for i = 1:numel(iPointPlaneEdges)
    iPlaneVertex = graph.edges(iPointPlaneEdges(i)).iVertices(2);
    [iBlock,jBlock] = blockMap(system,iPointPlaneEdges(i),iPlaneVertex);
    C(iBlock,jBlock(4)) = -1;
end
    
%construct E
for i = 1:numel(iAngleEdges)
    iPlaneVertices = graph.edges(iAngleEdges(i)).iVertices;
    plane1Eta = graph.vertices(iPlaneVertices(1)).value(1:3);
    plane2Eta = graph.vertices(iPlaneVertices(2)).value(1:3);
    plane1Block = blockMap(system,iPlaneVertices(1),'vertex');
    plane2Block = blockMap(system,iPlaneVertices(2),'vertex');
    edgeBlock = blockMap(system,iAngleEdges(i),'edge');
    E(edgeBlock,plane1Block(1:3)) = plane2Eta';
    E(edgeBlock,plane2Block(1:3)) = plane1Eta';
    f(edgeBlock) = 2;
end

% dX = lsqlin(system.A,system.b,C,d,E,f);
dX = lsqlin(system.A,system.b,C,d);


end


        
        