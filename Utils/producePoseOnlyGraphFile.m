function producePoseOnlyGraphFile(filepathGT,filepathMeas)

%GT
fileID = fopen(strcat(pwd,'/Data/GraphFiles/',filepathGT),'r');
Data = textscan(fileID, '%s', 'delimiter', '\n', 'whitespace', '');
CStr = Data{1};
fclose(fileID);

posesLines = [];
for j=1:1:length(CStr)
    splitLine = strsplit(CStr{j,1},' ');
    label = splitLine{1,1};
    if strcmp(label(1:length('VERTEX_POSE')),'VERTEX_POSE')
        posesLines = [posesLines, j];        
    end
end

toWriteGTFilePath = strcat(pwd,'/Data/GraphFiles/',filepathGT(1:end-6),'PosesOnly.graph');
fileID = fopen(toWriteGTFilePath, 'w');
for i=1:length(posesLines)
    label = 'VERTEX_POSE_R3_SO3';
    index = i;
    line = strsplit(CStr{posesLines(i)},' ');
    value = str2double(line(3:end));
    writeVertex(label,index,value,fileID);
end
fclose(fileID);

%Meas
fileID = fopen(strcat(pwd,'/Data/GraphFiles/',filepathMeas), 'r');
Data = textscan(fileID, '%s', 'delimiter', '\n', 'whitespace', '');
CStr = Data{1};

odometryLines = [];
for j=1:1:length(CStr)
    splitLine = strsplit(CStr{j,1},' ');
    label = splitLine{1,1};
    if strcmp(label(1:length('EDGE_R3')),'EDGE_R3')
        odometryLines = [odometryLines, j];        
    end
end

toWriteMeasFilePath = strcat(pwd,'/Data/GraphFiles/',filepathMeas(1:end-6),'PosesOnly.graph');
fileID = fopen(toWriteMeasFilePath, 'w');
for i=1:length(odometryLines)
    label = 'EDGE_R3_SO3';
    vIn = i;
    vOut = i+1;
    line = strsplit(CStr{odometryLines(i)},' ');
    value = str2double(line(4:9));
    covariance = str2double(line(10:end));
    writeEdge(label,vIn,vOut,value,covariance,fileID)
end
fclose(fileID);

end