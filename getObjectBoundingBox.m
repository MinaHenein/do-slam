function boundingBox = getObjectBoundingBox(detectionFile,frameID,objectClassGT)

%read file
fid = fopen(detectionFile,'r');
Data = textscan(fid, '%s', 'delimiter', '\n', 'whitespace', '');
CStr = Data{1};
if ~isempty(regexp(objectClassGT,'[\d.]+','match'))
    IndexC = strfind(CStr, strcat({num2str(frameID)},{' '},...
        regexp(objectClassGT,'[\d.]+','match'),{' '}));
    lineIndex = find(~cellfun('isempty', IndexC));
    line = cell2mat(CStr(lineIndex));
    splitLine = strsplit(line,' ');
    boundingBox = [str2double(splitLine(7)),str2double(splitLine(8)),...
        str2double(splitLine(9)),str2double(splitLine(10))];
    boundingBox = boundingBox + 1;
else
    boundingBox =[];
end
fclose(fid);

end


    