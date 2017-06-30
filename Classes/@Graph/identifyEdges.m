function [indexes] = identifyEdges(obj,type)
%IDENTIFYEDGES returns indexes of desired edge type

edgeTypes = {obj.edges.type}';

%find observations that have target value
indexes = strcmp(edgeTypes,type);
indexes = find(indexes);


end

