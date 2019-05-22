function  deleteDataAssociationFromGraphFile(FileName)
FID = fopen(FileName, 'r');
if FID == -1, error('Cannot open file'), end
Data = textscan(FID,'%s','Delimiter','\n');
CStr = Data{1};
fclose(FID);

IndexC = strfind(CStr, '2PointsDataAssociation');
Index = find(cellfun('isempty', IndexC));

% Save the file again:
FID = fopen(strcat(FileName(1:end-6),'_noDataAssociation.graph'), 'w');
if FID == -1, error('Cannot open file'), end
fprintf(FID, '%s\n', CStr{Index});
fclose(FID);
end