function cameraPoses = preprocessExtrinsics(extrinsicsFile,imageRange, settings)

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
    if strcmp(settings.dataset,'kitti')
        cameraPoseMatrix = reshape(lineArray(2:end),[4,4])';
    elseif strcmp(settings.dataset,'vkitti')
        cameraPoseMatrix = inv(reshape(lineArray(2:end),[4,4])');
    end
    
    cameraPoses(:,i) = transformationMatrixToPose(cameraPoseMatrix);
end

end