function cameraPoses = preprocessExtrinsics(extrinsicsFile,imageRange)

cameraPoses = zeros(6,numel(imageRange));

fileID = fopen(extrinsicsFile,'r');
Data = textscan(fileID,'%s','delimiter','\n','whitespace',' ');
CStr = Data{1};
fclose(fileID);

for i=1:numel(imageRange)
    lineCell = CStr{imageRange(i)+2,1}; % +1 to skip 1st line, +1 because frames start with 00000.png
    lineArray = str2double(strsplit(lineCell,' '));
    % assert frame number
    assert(lineArray(1)==imageRange(i));
    cameraPoseMatrix = inv(reshape(lineArray(2:end),[4,4])');
    cameraPoses(:,i) = transformationMatrixToPose(cameraPoseMatrix);
end

end