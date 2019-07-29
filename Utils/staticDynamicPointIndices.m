function [staticPointIndices, dynamicPointIndices] = staticDynamicPointIndices(config)

folder = pwd;
filepath = strcat(folder,'/Data/GraphFiles/',config.measurementsFileName);

pointIndices = [];
dynamicPointIndices = [];

fileID = fopen(filepath,'r');
Data = textscan(fileID,'%s','delimiter','\n','whitespace',' ');
CStr = Data{1};

for i=1:length(CStr)
   line = strsplit(CStr{i,1},' ');
   label = line{1};
   switch label
       case 'EDGE_3D'
            pointIndices = [pointIndices,str2double(line{3})];
       case '2PointsDataAssociation'
            dynamicPointIndices = [dynamicPointIndices,str2double(line{2}),...
                str2double(line{3})];
   end
end
fclose(fileID);

pointIndices = unique(pointIndices);
dynamicPointIndices = unique(dynamicPointIndices);
staticPointIndices = setdiff(pointIndices,dynamicPointIndices);

assert(isempty(intersect(staticPointIndices,dynamicPointIndices)))
end