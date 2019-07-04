%--------------------------------------------------------------------------
% Author: Montiel Abello - montiel.abello@gmail.com - 23/05/17
% Contributors:
%--------------------------------------------------------------------------

function writeVertex(label,index,value,fileID)
%WRITEVERTEX writes vertex to graph file

assert(~isempty(index))
formatSpec = strcat('%s %d ',repmat(' %0.9f',1,numel(value)),'\n');
fprintf(fileID,formatSpec,label,index,value);

end

