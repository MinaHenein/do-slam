function [dXAdjusted] = adjustUpdate(system,graph,dX)
%ADJUSTUPDATE Summary of this function goes here
%   Detailed explanation goes here

dXAdjusted = dX;

for i = 1:graph.nVertices
    if ~isempty(graph.vertices(i).type)
        switch graph.vertices(i).type
            case 'plane'
                iBlock = blockMap(system,i,'vertex');
                dXPrev = dX(iBlock);
                parameters = graph.vertices(i).value;
                normal = parameters(1:3);
                distance = parameters(4);
                %Project onto tangent space of normal
                dXNormal = (eye(3)-normal*normal')*dXPrev(1:3);
                dXDistance = dXPrev(4);
                dXAdjusted(iBlock) = [dXNormal; dXDistance];
                
        end
    end
end


end

