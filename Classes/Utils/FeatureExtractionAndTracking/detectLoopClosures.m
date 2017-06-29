function loopClosureFrame = detectLoopClosures(firstFrame,increment,lastFrame,...
    config,syncedData,gtFilePath)

rgbFileName = config.rgbImageName;
rgbFileNameFormat = getFileNameFormat(rgbFileName);
depthFileName = config.depthImageName;
depthFileNameFormat = getFileNameFormat(depthFileName);

N = (lastFrame-firstFrame)/increment;
changed = zeros(N,1);

for i=firstFrame:increment:lastFrame-1
    for j=i+1:increment:lastFrame
        if(IsKeyFrame(i) && IsKeyFrame(j))
            [closeLoop,type] = CloseLoop(i,j,syncedData,gtFilePath,...
                rgbFileNameFormat,depthFileNameFormat);
            if closeLoop
                if j > i+500 && changed(i,1)== 0
                    loopClosureFrame(i,:) = [i,j,type];
                    changed(i,1)=1;
                end
            end
        end
    end
end

loopClosureFrame(~any(loopClosureFrame,2),:) = [];

end

function [closeLoop,type] = CloseLoop(i,j,syncedData,gtFilePath,...
                rgbFileNameFormat,depthFileNameFormat)

gtFileID = fopen(gtFilePath);

if ~isempty(syncedData)
    gtInSyncedData = length(depthFileNameFormat)+length(rgbFileNameFormat)+3;
    lineScan = textscan(gtFileID,'%s',1,'delimiter','\n','headerlines',...
        str2double(syncedData(i,gtInSyncedData:end))-1);
else
    lineScan = textscan(gtFileID,'%s',1,'delimiter','\n','headerlines', i-1);
end
cameraIDPose = str2num(cell2mat(strsplit(cell2mat(lineScan{1,1}),'')));
fclose(gtFileID);
cameraPose = cameraIDPose(2:end);
cameraTranslation = cameraPose(1:3)';
cameraRotation = quaternion2Axis([cameraPose(4);cameraPose(5);cameraPose(6);...
            cameraPose(7)]);
cameraPose = [cameraTranslation;cameraRotation];        
fclose(gtFileID);

closeLoop = 0;
type = 0;
% case 1. close in 3D positions
if(norm(cameraPose(i,1:3)-cameraPose(j,1:3))<5)
    closeLoop = 1;
    type = 1; 
% case 2. displaced only along 1-axis and inverse orientation
elseif (abs(cameraPose(i,1)-cameraPose(j,1))<1 && abs(cameraPose(i,2)-cameraPose(j,2))<1 && ...
        norm(arot(rot(cameraPose(i,4:6)')*rot(cameraPose(j,4:6)')))<0.1)
    closeLoop = 1;
    type = 2;
elseif (abs(cameraPose(i,2)-cameraPose(j,2))<1 && abs(cameraPose(i,3)-cameraPose(j,3))<1 && ...
        norm(arot(rot(cameraPose(i,4:6)')*rot(cameraPose(j,4:6)')))<0.1)
    closeLoop = 1;
    type = 2;
elseif (abs(cameraPose(i,1)-cameraPose(j,1))<1 && abs(cameraPose(i,3)-cameraPose(j,3))<1 && ...
        norm(arot(rot(cameraPose(i,4:6)')*rot(cameraPose(j,4:6)')))<0.1)
    closeLoop = 1;
    type = 2;
end
    
end

function isKeyFrame = IsKeyFrame(i)
isKeyFrame = 0;
if(mod(i,20)==0)
    isKeyFrame = 1;
end
end