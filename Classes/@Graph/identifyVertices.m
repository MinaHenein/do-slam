function [indexes] = identifyVertices(obj,type)
%IDENTIFYVERTICES returns indexes of desired vertices

vertexTypes = {obj.vertices.type}';

%find observations that have target value
indexes = strcmp(vertexTypes,type);
indexes = find(indexes);

end

