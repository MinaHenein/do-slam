function [cameraGraphIndexes, cameraPoses, pointPositions] = preProcessVideo(config,solver,resultsFileName,nIterations)

for i=1:nIterations
    graph  = solver.graphs(i);
    %save results to graph file
    fileName = strcat(resultsFileName(1:8),num2str(i),resultsFileName(9:end));
    graph.saveGraphFile(config,fileName);
    filepath = strcat(config.folderPath,config.sep,'Data',...
        config.sep,config.graphFileFolderName,config.sep,fileName);
    fileID = fopen(filepath,'r');
    data = textscan(fileID,'%s','delimiter','\n','whitespace',' ');
    CStr = data{1};
    indexC = strfind(CStr, config.poseVertexLabel);
    fclose(fileID);
    lineIndex = find(~cellfun('isempty', indexC));
    for j=1:length(lineIndex)
        pointVertices = [];
        if j == length(lineIndex)
            %read to end of file
            k2 = size(CStr,1);
        else
            %read to one line before next psoe
            k2 = lineIndex(j+1)-1;
        end
        for k=lineIndex(j)+1:k2
            fileID = fopen(filepath,'r');
            line = textscan(fileID,'%s',1,'delimiter','\n','headerlines',k-1);
            line = cell2mat(line{1,1});
            splitLine = str2double(strsplit(line,' '));
            if strcmp(line(1:length(config.pointVertexLabel)),config.pointVertexLabel)
                pointVertexValue = splitLine(1,3:5)';
                pointVertices = [pointVertices,pointVertexValue];
            end
            fclose(fileID);
        end
        pointPositions{i,j} = pointVertices;
    end
    cameraGraphIndexes = graph.identifyVertices('pose');
    cameraPoses{i} = [graph.vertices(graph.identifyVertices('pose')).value];
end
    
end