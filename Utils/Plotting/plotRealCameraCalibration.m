function plotRealCameraCalibration(config,fileName)
filepath = strcat(config.folderPath,config.sep,'Data',...
        config.sep,config.graphFileFolderName,config.sep,fileName);
fileID = fopen(filepath,'r');
Data = textscan(fileID, '%s', 'delimiter', '\n', 'whitespace', '');
CStr = Data{1};
fclose(fileID);
IndexC = strfind(CStr, config.poseVertexLabel);
% find lines with a DataAssociation entry
Index = find(~cellfun('isempty', IndexC));
poseValues = [];
for j=1:length(Index)
    % get line of Index
    fileID = fopen(filepath,'r');
    line = textscan(fileID,'%s',1,'delimiter','\n','headerlines',Index(j)-1);
    line = cell2mat(line{1,1});
    splitLine = strsplit(line,' ');
    value = str2double(splitLine(1,3:8));
    poseValues = [poseValues, value'];
    fclose(fileID);
end

figure;
for i=1:size(poseValues,2)
    iPose = poseValues(:,i); 
    plotCoordinates(iPose(1:3,:),rot(iPose(4:6,1)))
    hold on
end
xlabel('x');
ylabel('y');
zlabel('z');

end