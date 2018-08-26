function produceUniqueIDsFromSegementedImages(rgbImagesFilePath)

images = dir(strcat(rgbImagesFilePath,'*.png'));
tStart = tic;
for i=1:length(images)
    I = imread(strcat(rgbImagesFilePath,images(i).name));
    for j=1:size(I,1)
        for k=1:size(I,2)
            if i==1 && j==1 && k==1
                %first pixel of first image
                nObjects = 1;
                rgb = [I(j,k,1),I(j,k,2),I(j,k,3)];
                id = 1;
                uniqueIDRGB = [double(id),double(rgb)];
            else
                %every other pixel
                rgb = [I(j,k,1),I(j,k,2),I(j,k,3)];
                if ~ismember(rgb,uniqueIDRGB(:,2:4),'rows')
                    %increase nObjects
                    nObjects = nObjects+1;
                    % add id & rgb to uniqueIDRGB
                    id = nObjects;
                    uniqueIDRGB = [uniqueIDRGB;double(id),double(rgb)];
                end
            end
        end
    end
end
toc(tStart)

