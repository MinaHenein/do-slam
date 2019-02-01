function  deleteMotionVerticesFromGraphFile(FileName)

FID = fopen(FileName, 'r');
if FID == -1, error('Cannot open file'), end
Data = textscan(FID,'%s','Delimiter','\n');
CStr = Data{1};
fclose(FID);

IndexC = strfind(CStr, 'VERTEX_SE3Motion');
Index = find(not(cellfun('isempty',IndexC)));


% Delete inital lines:
if ~isempty(Index)
  CStr(Index) = [];
end

% Save the file again:
FID = fopen(FileName, 'w');
if FID == -1, error('Cannot open file'), end
fprintf(FID, '%s\n', CStr{:});
fclose(FID);
end