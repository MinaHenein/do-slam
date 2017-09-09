function [pointsMeasurements,pointsLabels,pointsTurtlebotID,pointsCameras] = ...
    manualLandmarkExtraction(rgbImagesPath,depthImagesPath, K_Cam)   


pointsMeasurements = [];
pointsLabels = [];
pointsTurtlebotID = [];
pointsCameras = [];
nImages = size(dir([rgbImagesPath '/*.jpg']),1);
for i= 1:nImages
    frame = i;
    I = imread(strcat(rgbImagesPath,num2str(frame),'.jpg'));
    imshow(I);
    prompt = 'Manually enter number of landmarks in image: ';
    nLandmarks = input(prompt);
    points = zeros(nLandmarks,2);
    for j=1:size(points,1)
        nPoints = nPoints+1;
        disp('Manually extract turtlebot landmark');
        [x,y] = ginput(1);
        points(j,:) = [x,y];
        prompt = 'Is color code the same? '; 
        sameColorCode = input(prompt);
        if ~sameColorCode
            prompt = 'Manually determine color code: '; 
            s = input(prompt);
            [label, turtlebotID] = getLabelTurtlebotID(s);
        end
        point = points(j,:);
        depth = load(strcat(depthImagesPath,num2str(i),'.csv'));
        PCamera = K_Cam \ [point(1);point(2);1];
        zCamera = depth(round(point(2)), round(point(1)));
        % reject points in a field of view greater than 5 m in distance
        if zCamera > 5 || isnan(zCamera)
            continue
        end
        PCamera = PCamera*zCamera;
        pointsMeasurements = [pointsMeasurements;PCamera];
        pointsLabels = [pointsLabels;label];
        pointsTurtlebotID = [pointsTurtlebotID;turtlebotID];
        pointsCameras = [pointsCameras;frame];
    end
end
    