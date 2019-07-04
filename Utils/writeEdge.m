%--------------------------------------------------------------------------
% Author: Montiel Abello - montiel.abello@gmail.com - 23/05/17
% Contributors:
%--------------------------------------------------------------------------

function writeEdge(label,vIn,vOut,value,covariance,fileID)
%WRITEEDGE writes edge to graph file

covariance = covToUpperTriVec(covariance);
formatSpec = strcat('%s',...
                    repmat(' %d',1,numel(vIn)),...
                    repmat(' %d',1,numel(vOut)),...
                    repmat(' %0.9f',1,numel(value)),...
                    repmat(' %0.9f',1,numel(covariance)),'\n');
fprintf(fileID,formatSpec,label,vIn,vOut,value,covariance);
end

