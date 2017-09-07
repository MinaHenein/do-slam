
function [pointsMeasurements,pointsLabels,pointsTurtlebotID] = ...
    extract3DPoints(rgbImagesPath,depthImagesPath)
 
rectified = 0;
display = 1;
pointsMeasurements = [];
pointsLabels = [];
pointsTurtlebotID = [];
nImages = size(dir([rgbImagesPath '/*.jpg']),1);
K_Cam = [526.37013657, 0.00000000, 313.68782938;
         0.00000000, 526.37013657, 259.01834898;
         0.00000000, 0.00000000, 1.00000000];
for i=1:nImages
    
    I1 = imread(strcat(rgbImagesPath,num2str(i),'.jpg'));
    I1 = I1(size(I1,1)/2:end,:,:); 
    siftThreshold = 12;
    shiftPixels = 35;
    [turtlebotID1, points1,label1] = extractLabelLandmarks(I1,siftThreshold,shiftPixels);
    if(size(label1,1)<6)
        disp('Image needs rectification')
        rectified=1;
        prompt = 'User input : 1 or 2 ';
        id = input(prompt);
        if id ==1
            I = imread(strcat(rgbImagesPath,'matlab1.jpg'));
        elseif id ==2
            I = imread(strcat(rgbImagesPath,'matlab2.jpg'));
        end
        if(size(I1,1)<size(I,1) || size(I1,2)<size(I,2))
            I1=imcrop(I1,[round((size(I1,2)-size(I,2))/2) round((size(I1,1)-size(I,1))/2)...
                size(I,2)-1 size(I,1)-1]);
        end
        figure,image(I1);
        hold on
        h=imshow(I);
        set(h,'AlphaData',0.5);
        I1=double(I1);
        [I2,H]=frontoparallel(I1);
        siftThreshold = 12;
        shiftPixels = 60;
        [turtlebotID2,points2,label2] = extractLabelLandmarks(I2,siftThreshold,...
            shiftPixels);
    end
    
    if rectified
        if size(label2,1)< size(label1,1)
            rectified=0;
        end
    end
    
    if rectified
        points = points2;
        label = label2;
        turtlebotID = turtlebotID2;
        I = I2;
        labeledPoints=zeros(4,size(points,2));
    else
        points = points1;
        label = label1;
        turtlebotID = turtlebotID1;
        I = I1;
        labeledPoints =  points;
    end
    for j=1:size(points,2)
        if rectified
            point = H\[points(1:2,j);1];
            point = point/point(3);
            labeledPoints(1:2,j) =   point(1:2);
            labeledPoints(3:4,j) =   points(3:4,j);
        end
        if display
            figure;
            imshow(I);
            hold on
            plotsiftframe(labeledPoints(:,j));
            title(strcat('label ',num2str(label(j))))
            hold off
        end
        depth = reshape(load(strcat(depthImagesPath,num2str(i),'.csv')),640,480)';
        PCamera = K_Cam \ [point(1);point(2);1];
        zCamera = depth(round(point(1)), round(point(2)));
        % reject points in a field of view greater than 5 m in distance
        if depth(round(point(1)), round(point(2))) > 5
            continue
        end
        PCamera = PCamera*zCamera;
        pointsMeasurements = [pointsMeasurements;PCamera];
        pointsLabels = [pointsLabels;label];
        pointsTurtlebotID = [pointsTurtlebotID;turtlebotID];
    end
    
end
end

