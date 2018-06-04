function [obj] = updateVertices(obj,config,system,dVertices)
%UPDATEVERTICES updates vertices with vector dX
%   loop over vertices, increment values

% added for incremental update of affected vertices only
if strcmp(config.processing,'incrementalSolveHessian') || ...
        strcmp(config.processing,'incrementalSolveCholesky')
    %update last edge vertices only
    verticesToUpdate = obj.edges(end).iVertices;
    nVerticesToUpdate = length(verticesToUpdate);
    for j=1:nVerticesToUpdate
        jBlock = blockMap(system,j,'vertex');
        switch obj.vertices(verticesToUpdate(j)).type
            case {'pose','SE3Motion'}
                obj.vertices(verticesToUpdate(j)).value = ...
                    config.relativeToAbsolutePoseHandle(obj.vertices(verticesToUpdate(j)).value,...
                    dVertices(jBlock));
            case 'plane'
                % d> 0
                obj.vertices(verticesToUpdate(j)).value = obj.vertices(verticesToUpdate(j)).value + ...
                    dVertices(jBlock);
                if obj.vertices(verticesToUpdate(j)).value(4) < 0
                    obj.vertices(verticesToUpdate(j)).value = -obj.vertices(verticesToUpdate(j)).value;
                end
            otherwise
                obj.vertices(verticesToUpdate(j)).value = obj.vertices(verticesToUpdate(j)).value + ...
                    dVertices(jBlock);
        end
    end
else
    
    for j = 1:obj.nVertices
        jBlock = blockMap(system,j,'vertex');
        %     if norm(dVertices(jBlock)) > 1e-4
        if ~isempty(obj.vertices(j).type)
            switch obj.vertices(j).type
                case {'pose','SE3Motion'}
                    obj.vertices(j).value = config.relativeToAbsolutePoseHandle(obj.vertices(j).value,dVertices(jBlock));
                    %             obj.vertices(j).value = RelativeToAbsolutePose(obj.vertices(j).value,dVertices(jBlock));
                    %             obj.vertices(j).value = Relative2AbsoluteSE3(obj.vertices(j).value,dVertices(jBlock));
                case 'plane'
                    %                 obj.vertices(j).value = obj.vertices(j).value + dVertices(jBlock);
                    %                 obj.vertices(j).value(1:3) = obj.vertices(j).value(1:3)/norm(obj.vertices(j).value(1:3));
                    
                    %                 % d> 0
                    obj.vertices(j).value = obj.vertices(j).value + dVertices(jBlock);
                    if obj.vertices(j).value(4) < 0
                        obj.vertices(j).value = -obj.vertices(j).value;
                    end
                    
                    % same signs
                    %                 currentSign = sign(obj.vertices(j).value(4));
                    %                 obj.vertices(j).value = obj.vertices(j).value + dVertices(jBlock);
                    %                 if (currentSign ~= sign(obj.vertices(j).value(4)))
                    %                     obj.vertices(j).value(1:3) = -obj.vertices(j).value(1:3);
                    %                 end
                    
                otherwise
                    obj.vertices(j).value = obj.vertices(j).value + dVertices(jBlock);
            end
        end
        %     end
    end
    
end

end


