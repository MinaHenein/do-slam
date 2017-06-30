function [graph] = unsortVertices(graph,newToOldMap)
%UNSORTVERTICES Summary of this function goes here
%   Detailed explanation goes here

%preallocate
verticesReordered = Vertex();
verticesReordered(1:graph.nVertices) = Vertex();

%reorder vertices
for i = 1:graph.nVertices
    verticesReordered(newToOldMap(i)) = graph.vertices(i);
    verticesReordered(i).index = i;
end
graph.vertices = verticesReordered;

%map vertex indexes in edges
for i = 1:graph.nEdges
    graph.edges(i).iVertices = newToOldMap(graph.edges(i).iVertices)';
end

end

