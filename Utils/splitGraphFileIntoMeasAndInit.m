function  splitGraphFileIntoMeasAndInit(filePath)

fid = fopen(filePath, 'r');
if fid == -1, error('Cannot open file'), end
Data = textscan(fid,'%s','Delimiter','\n');
CStr = Data{1};
fclose(fid);

IndexC = strfind(CStr, 'VERTEX');
Index = find(not(cellfun('isempty', IndexC)));

% Save in a different file
filepath = strcat(filePath(1:end-6),'_init.graph');
fid = fopen(filepath,'w');
if fid == -1, error('Cannot open file'), end
fprintf(fid, '%s\n', CStr{Index});
fclose(fid);

if ~isempty(Index)
  CStr(Index) = [];
end
% Save the file again:
fid = fopen(filePath, 'w');
if fid == -1, error('Cannot open file'), end
fprintf(fid, '%s\n', CStr{:});
fclose(fid);
end