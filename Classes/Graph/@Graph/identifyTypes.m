function [obj] = identifyTypes(obj)
%IDENTIFYTYPES Summary of this function goes here
%   Detailed explanation goes here

%% 1. identify vertices
for i = 1:obj.nVertices
    if ~isempty(obj.vertices(i).type)
        switch obj.vertices(i).type
            case 'pose'
                obj.iPoseVertices = [obj.iPoseVertices; i];
            case 'point'
                obj.iPointVertices = [obj.iPointVertices; i];
        end
    end
end

%% 2. identify edges
for i = 1:obj.nEdges
    if ~isempty(obj.edges(i).type)
        switch obj.edges(i).type
            case 'pose-pose'
                obj.iPosePoseEdges = [obj.iPosePoseEdges; i];
            case 'pose-point'
                obj.iPosePointEdges = [obj.iPosePointEdges; i];
            case 'point-point'
                obj.iPointPointEdges = [obj.iPointPointEdges; i];
        end
    end
end

end

