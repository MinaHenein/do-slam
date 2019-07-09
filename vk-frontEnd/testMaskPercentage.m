
sequence = '0001';
variation = 'clone';
imageRange = 335:424;

% setup
cd /home/mina/workspace/src/Git/do-slam
dir = '/media/mina/Data/mina/Downloads/Virtual_KITTI/';
% directories
depthDir = 'vkitti_1.3.1_depthgt/';
objSegDir = 'vkitti_1.3.1_scenegt/';
motDir = 'vkitti_1.3.1_motgt/';
% data
depthI = strcat(dir,depthDir,sequence,'/',variation,'/');
maskI = strcat(dir,objSegDir,sequence,'/',variation,'/');
motFile = strcat(dir,motDir,sequence,'_',variation,'.txt');

% read mot file
fileID = fopen(motFile,'r');
Data = textscan(fileID,'%s','delimiter','\n','whitespace',' ');
CStr = Data{1};
fclose(fileID);

percentages = cell(1,4);
lastLine = 1;
for i = 1:length(imageRange)
    frameName = strcat(repmat('0',1,5-numel(num2str(imageRange(i)))),num2str(imageRange(i)),'.png');
    for j = lastLine:numel(CStr)-1
        lineCell = strsplit(CStr{j+1,1},' '); % +1 to skip 1st line
        if (str2double(lineCell{1,1}) == imageRange(i))
            lastLine = j;
            label = lineCell{3};
            isMoving = str2double(lineCell{1,23});
            if strcmp(label,'Car') && isMoving
                % object bounding box in pixels most left,top,right,bottom
                l = str2double(lineCell{1,7});
                t = str2double(lineCell{1,8});
                r = str2double(lineCell{1,9});
                b = str2double(lineCell{1,10});
                boundingBox = [l,t,r,b];
                % object binary mask
                maskIm = imread(strcat(maskI,frameName));
                objectMask = getObjectMask(l,t,r,b,maskIm);
                % object centroid
                xCentroid = (l+r)/2;
                yCentroid = (t+b)/2;
                % centroid depth
                depthIm = imread(strcat(depthI,frameName));
                [nRows, nCols] = size(depthIm);
                centroidDepth = double(depthIm(round(yCentroid),round(xCentroid)));
                if centroidDepth/100 <= 10 
                    percentages{1,1} = [percentages{1,1}, 100*sum(sum(objectMask))/(nRows*nCols)];
                elseif centroidDepth/100 > 10 && centroidDepth/100 <= 20                           
                    percentages{1,2} = [percentages{1,2}, 100*sum(sum(objectMask))/(nRows*nCols)];
                elseif centroidDepth/100 > 20 && centroidDepth/100 <= 30
                    percentages{1,3} = [percentages{1,3}, 100*sum(sum(objectMask))/(nRows*nCols)];
                else
                    percentages{1,4} = [percentages{1,4}, 100*sum(sum(objectMask))/(nRows*nCols)];
                end
            end
        elseif (str2double(lineCell{1,1}) > imageRange(i))
            break
        end
    end
end

averagePercentages = zeros(1,4);
for i=1:length(percentages)
    averagePercentages(1,i) = mean(percentages{1,i});
end
ceil(mean([averagePercentages(1,1),averagePercentages(1,2)]))