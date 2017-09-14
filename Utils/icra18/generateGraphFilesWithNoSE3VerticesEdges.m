function generateGraphFilesWithNoSE3VerticesEdges(filePath)

fileID = fopen(strcat(filePath,'.graph'),'r');
Data = textscan(fileID, '%s', 'delimiter', '\n', 'whitespace', '');
CStr = Data{1};
fclose(fileID);
IndexC1 = strfind(CStr, 'VERTEX_SE3Motion');
Index1 = find(~cellfun('isempty', IndexC1));
CStr(Index1) = [];

IndexC2 = strfind(CStr, 'EDGE_2POINTS_SE3Motion');
Index2 = find(~cellfun('isempty', IndexC2));
CStr(Index2) = [];

% Save the file again:
fileID = fopen(strcat(filePath,'NOSE3.graph'), 'w');
fprintf(fileID, '%s\n', CStr{:});
fclose(fileID);

end