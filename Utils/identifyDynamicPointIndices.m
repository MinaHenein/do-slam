function dynamicPointIndices = identifyDynamicPointIndices(gtFileName)

fileID = fopen(strcat(pwd,'/Data/GraphFiles/',gtFileName),'r');
Data = textscan(fileID, '%s', 'delimiter', '\n', 'whitespace', '');
CStr = Data{1};
fclose(fileID);

IndexC = strfind(CStr, '2PointsDataAssociation');
Index = find(not(cellfun('isempty',IndexC)));

dynamicPointIndices = [];

for i=1:1:length(Index)
    fileID = fopen(strcat(pwd,'/Data/GraphFiles/',gtFileName),'r');
    line = textscan(fileID,'%s',1,'delimiter','\n','headerlines',Index(i)-1);
    splitLine = strsplit(cell2mat(line{1,1}),' ');
    label = splitLine{1,1};
    point1 = str2double(splitLine{1,2});
    point2 = str2double(splitLine{1,3});    
    dynamicPointIndices = [dynamicPointIndices, point1, point2];
end 

dynamicPointIndices = unique(dynamicPointIndices);