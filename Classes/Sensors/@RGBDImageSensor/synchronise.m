function syncedData = synchronise(~,config)

%--------------------------------------------------------------------------
% Author: Mina Hnenein - mina.henein@anu.edu.au - 20/06/17
% Contributors:
%--------------------------------------------------------------------------

% synchronises data  inputs in cases on non-synced data
% inputs: config
%   contains paths to rgb and depth images and trajectory
%   rgb images named rgbImageTimeStamp.png
%   depth images named depthImageTimeStamp.png
%   where rgbImageTimeStamp and depthImageTimeStamp are the time stamps of
%   the rgb and depth images respectively 
% outputs: syncedData
%   syncedData has the following format:
%   depthTimeStamp[depthFileNameFormat] rgbTimeStamp[rgbFileNameFormat]...
%   cameraPoseLineInGTFile[cameraPoseLineFormat]

if ~config.synchronizedData

rgbFilePath = strcat(config.folderPath,config.sep,config.rgbImagesFolderName);
depthFilePath = strcat(config.folderPath,config.sep,config.depthImagesFolderName);
gtFilePath = strcat(config.folderPath,config.sep,config.graphFileFolderName,...
    config.sep,config.groundTruthFileName);

% get depth files time stamps
depthFiles = dir(strcat(depthFilePath,'/*',config.depthImageFileExtension));
nDepthFiles = length(depthFiles);
depthTimeStamps = zeros(nDepthFiles,1);
depthFileNameFormat = getFileNameFormat(depthFiles(1).name);
depthTimeStampsStr = repmat(depthFileNameFormat,[nDepthFiles,1]);
for i = 1:nDepthFiles
    depthTimeStamps(i,1) = str2double(depthFiles(i).name(1:end-4));
    depthTimeStampsStr(i,:) = depthFiles(i).name(1:end-4);
end

% get rgb files time stamps
rgbFiles = dir(strcat(rgbFilePath,'/*',config.rgbImageFileExtension));
nRGBFiles = length(rgbFiles);
rgbTimeStamps = zeros(nRGBFiles,1);
rgbFileNameFormat = getFileNameFormat(rgbFiles(1).name);
rgbTimeStampsStr = repmat(rgbFileNameFormat,[nRGBFiles,1]);
for i = 1:nRGBFiles
    rgbTimeStamps(i,1) = str2double(rgbFiles(i).name(1:end-4));
    rgbTimeStampsStr(i,:) = rgbFiles(i).name(1:end-4);
end

% get ground truth files time stamps
fid = fopen(gtFilePath);
timeStampedCameraPoses = [];
line = 1;
for k=1:3
    tline = fgetl(fid);
    line = line+1;
end
switch config.poseRotationRepresentation
    case 'quaternion'
        timeStampedCameraPoseFormat = '%f %f %f %f %f %f %f %f';
    case {'axis-angle','euler angles'}
        timeStampedCameraPoseFormat = '%f %f %f %f %f %f %f';
    case 'rotation matrix'
        timeStampedCameraPoseFormat = '%f %f %f %f %f %f %f %f %f %f %f %f %f';
    otherwise 
        error('Undefined camera pose rotation representation')
end

while (~feof(fid))
   tline = fgetl(fid);
   a = textscan(tline,timeStampedCameraPoseFormat, 'delimiter',' ');
   timeStampedCameraPoses = [timeStampedCameraPoses;cell2mat(a(1)),line];
   line = line+1;
end
fclose(fid);

nCameraPoses = size(timeStampedCameraPoses,1);
cameraPoseLineFormat = '';
for i=1:length(num2str(nCameraPoses))
    cameraPoseLineFormat = strcat(cameraPoseLineFormat,'0');
end

% get the min of nDepthFiles, nRGBFiles and nCameraPoses, and for each file
% of the min, find the closest time stamp in the 2 other files
nFiles = [nDepthFiles,nRGBFiles,nCameraPoses];
[~,indexOfMin] = min(nFiles);
% for each file in min(nFiles), find the closest time stamp in the other two files
syncedData = repmat(strcat(depthFileNameFormat,' ',rgbFileNameFormat,' ',...
    cameraPoseLineFormat),[nFiles(indexOfMin),1]);
for i=1:nFiles(indexOfMin)
    switch indexOfMin
        case 1
            depthTime = depthTimeStamps(i,1);
            depthTimeStr = depthTimeStampsStr(i,:);
            rgbTimeGap = abs(bsxfun(@minus,rgbTimeStamps(:,1),depthTime));
            gtTimeGap = abs(bsxfun(@minus,timeStampedCameraPoses(:,1),depthTime));
            [~,closestRGB] = min(rgbTimeGap);
            [~,closestGT] = min(gtTimeGap);
            zeroPadding = '';
            for j=1:length(cameraPoseLineFormat)-...
                    length(num2str(timeStampedCameraPoses(closestGT,2)))
                zeroPadding = strcat(zeroPadding,'0');
            end
            timeStampedCameraPosesStr = strcat(zeroPadding,...
                        num2str(timeStampedCameraPoses(closestGT,2)));
            syncedData(i,1:17) = depthTimeStr;
            syncedData(i,19:35) = rgbTimeStampsStr(closestRGB,:);
            syncedData(i,37:end) = timeStampedCameraPosesStr;
        case 2
            rgbTime = rgbTimeStamps(i,1);
            rgbTimeStr = rgbTimeStampsStr(i,:);
            depthTimeGap = abs(bsxfun(@minus,depthTimeStamps(:,1),rgbTime));
            gtTimeGap = abs(bsxfun(@minus,timeStampedCameraPoses(:,1),rgbTime));
            [~,closestDepth] = min(depthTimeGap);
            [~,closestGT] = min(gtTimeGap);
            zeroPadding = '';
            for j=1:length(cameraPoseLineFormat)-...
                    length(num2str(timeStampedCameraPoses(closestGT,2)))
                zeroPadding = strcat(zeroPadding,'0');
            end
            timeStampedCameraPosesStr = strcat(zeroPadding,...
                        num2str(timeStampedCameraPoses(closestGT,2)));
            syncedData(i,1:17) = depthTimeStampsStr(closestDepth,:);
            syncedData(i,19:35) = rgbTimeStr;
            syncedData(i,37:end) = timeStampedCameraPosesStr;
        case 3
            gtTime = timeStampedCameraPoses(i,1);
            zeroPadding = '';
            for j=1:length(cameraPoseLineFormat)-...
                    length(num2str(timeStampedCameraPoses(closestGT,2)))
                zeroPadding = strcat(zeroPadding,'0');
            end
            timeStampedCameraPosesStr = strcat(zeroPadding,...
                        num2str(timeStampedCameraPoses(closestGT,2)));
            gtTimeStr = timeStampedCameraPosesStr;
            depthTimeGap = abs(bsxfun(@minus,depthTimeStamps(:,1),gtTime));
            rgbTimeGap = abs(bsxfun(@minus,rgbTimeStamps(:,1),gtTime));
            [~,closestDepth] = min(depthTimeGap);
            [~,closestRGB] = min(rgbTimeGap);
            syncedData(i,1:17) = depthTimeStampsStr(closestDepth,:);
            syncedData(i,19:35) = rgbTimeStampsStr(closestRGB,:);
            syncedData(i,37:end) = gtTimeStr;
    end
end

else
    syncedData = [];
    disp('Data already scynchronized')

end
end