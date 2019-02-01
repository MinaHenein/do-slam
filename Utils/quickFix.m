
fid = fopen(FileName, 'r');

Data = textscan(fid,'%s','Delimiter','\n');
CStr = Data{1};
fclose(fid);

IndexC = strfind(CStr, '2PointsDataAssociation');
Index = find(not(cellfun('isempty',IndexC)));

for i =1:length(Index)
    fileID = fopen(FileName,'r');
    line = textscan(fileID,'%s',1,'delimiter','\n','headerlines',Index(i)-1);
    splitLine = strsplit(cell2mat(line{1,1}),' ');
    label = splitLine{1,1};
    point1 = str2double(splitLine{1,2});
    point2 = str2double(splitLine{1,3});    
    object = str2double(splitLine{1,4});
    if object==1
        index = 1213;
    elseif object ==2
        index= 1214;
    end
    fclose(fileID);
    CStr(Index(i)) = cellstr(sprintf('%s %d %d %d',...
            label,point1,point2,index));
end

filepath = strcat(FileName(1:end-6),'Test.graph');
fileID = fopen(filepath,'w');
fprintf(fileID, '%s\n', CStr{:});
fclose(fileID);
