function image = createImageObject(segmentationFile,detectionFile,i,imageList,assignObjectID)

I = imread(strcat('/home/mina/Downloads/vKitti/vkitti_1.3.1_scenegt/0001/clone/00',...
    num2str(imageList(i)),'.png'));
rgbImage = imread(strcat('/home/mina/Downloads/vKitti/rgbImages/00',...
    num2str(imageList(i)),'.png'));
image.I = rgbImage;
depthImage = imread(strcat('/home/mina/Downloads/vKitti/depthImages/00',...
    num2str(imageList(i)),'.png'));
image.depth = depthImage;

for j=1:size(I,1)
    for k=1:size(I,2)
        if j==1 && k==1
            %first pixel
            image.nObjects = 1;
            image.objects(1).rgb = [I(j,k,1),I(j,k,2),I(j,k,3)];
            if assignObjectID
                image.objects(1).id = 1;
                image.uniqueID = 1;
            end
            image.uniqueRGB = [I(j,k,1),I(j,k,2),I(j,k,3)];
        else
            %every other pixel
            rgb = [I(j,k,1),I(j,k,2),I(j,k,3)];
            if ~ismember(rgb,image.uniqueRGB,'rows')
                %increase nObjects
                image.nObjects = image.nObjects+1;
                %assign object rgb & id
                image.objects(image.nObjects).rgb = rgb;
                if assignObjectID
                    image.objects(image.nObjects).id = max([image.objects.id])+1;
                    % add id to uniqueID
                    image.uniqueID = [image.uniqueID;image.objects(image.nObjects).id];
                end
                % add rgb to uniqueRGB
                image.uniqueRGB = [image.uniqueRGB;rgb];
            end
        end
    end
end

for y=1:image.nObjects
    objectRGB = image.uniqueRGB(y,:);
    objectMask = I(:,:,1)==objectRGB(1) & I(:,:,2)==objectRGB(2) &...
        I(:,:,3)==objectRGB(3);
    %store object mask
    image.objects(y).mask = objectMask; 
    %store object centroids
    objectCentroid = getObjectCentroid(objectMask);
    image.objects(y).centroid = objectCentroid;
    %store object classes
    [objectClass, objectClassGT] = getObjectClass(segmentationFile, objectRGB);
    image.objects(y).class = objectClass;
    image.objects(y).classGT = objectClassGT;
end

nDeleted = 0;
for y=1:image.nObjects
    switch image.objects(y-nDeleted).class
        case{'Sky','Pole','Tree','Building','Vegetation','TrafficLight',...
                'TrafficSign','Terrain','Road','Misc'}
            image.objects(y-nDeleted) = [];
            image.uniqueRGB(y-nDeleted,:) = [];
            if assignObjectID
                image.uniqueID(y-nDeleted,:) = [];
            end
            nDeleted = nDeleted + 1; 
    end
end
image.nObjects = length(image.objects);

for y=1:image.nObjects
    %store object bounding box
    objectBoundingBox = getObjectBoundingBox(detectionFile,imageList(i),...
        image.objects(y).classGT);
    image.objects(y).boundingBox = objectBoundingBox;
    if ~assignObjectID
        image.uniqueID = [];
    end
end

nDeleted = 0;
for y=1:image.nObjects
    if isempty(image.objects(y-nDeleted).boundingBox)
        image.objects(y-nDeleted) = [];
        image.uniqueRGB(y-nDeleted,:) = [];
        if assignObjectID
            image.uniqueID(y-nDeleted,:) = [];
        end
        nDeleted = nDeleted + 1; 
    end
end
image.nObjects = length(image.objects);

end