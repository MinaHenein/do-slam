function [graph] = unsortEdges(graph,newToOldMap)
%UNSORTVERTICES Summary of this function goes here
%   Detailed explanation goes here

%preallocate
edgesReordered = Edge();
edgesReordered(1:graph.nEdges) = Edge();

%reorder edges
for i = 1:graph.nEdges
    edgesReordered(newToOldMap(i)) = graph.edges(i);
    edgesReordered(i).index = i;
end
graph.edges = edgesReordered;

%map edge indexes in vertices
for i = 1:graph.nVertices
    graph.vertices(i).iEdges = newToOldMap(graph.vertices(i).iEdges)';
end

end

