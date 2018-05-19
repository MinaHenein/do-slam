function image = createImageObject(segmentationFile,i,imageList)

I = imread(strcat('00',num2str(imageList(i)),'.png'));
for j=1:size(I,1)
    for k=1:size(I,2)
        if j==1 && k==1
            %first pixel
            image.nObjects = 1;
            image.objects(1).rgb = [I(j,k,1),I(j,k,2),I(j,k,3)];
            image.objects(1).id = 1;
            image.uniqueRGB = [I(j,k,1),I(j,k,2),I(j,k,3)];
            image.uniqueID = 1;
        else
            %every other pixel
            rgb = [I(j,k,1),I(j,k,2),I(j,k,3)];
            if ~ismember(rgb,image.uniqueRGB,'rows')
                %increase nObjects
                image.nObjects = image.nObjects+1;
                %assign object rgb & id
                image.objects(image.nObjects).rgb = rgb;
                image.objects(image.nObjects).id = max([image.objects.id])+1;
                % add rgb to uniqueRGB & id to uniqueID
                image.uniqueRGB = [image.uniqueRGB;rgb];
                image.uniqueID = [image.uniqueID;image.objects(image.nObjects).id];
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
            image.uniqueID(y-nDeleted,:) = [];
            nDeleted = nDeleted + 1; 
    end
end
image.nObjects = length(image.objects);

end