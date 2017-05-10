function writeVertex(label,index,value,fileID)
%WRITEVERTEX writes vertex to graph file

assert(~isempty(index))
formatSpec = strcat('%s %d ',repmat(' %6.6f',1,numel(value)),'\n');
fprintf(fileID,formatSpec,label,index,value);

end

