
clear; clc;
segmentationFile = '/home/mina/Downloads/vkitti_1.3.1_scenegt/0001_clone_scenegt_rgb_encoding.txt';
detectionFile = '/home/mina/Downloads/vkitti_1.3.1_motgt/0001_clone.txt';
imageList = 342:399;
images = [];
for i=1:length(imageList)
    if i==1
        assignObjectID = 1;
        image = createImageObject(segmentationFile,detectionFile,i,imageList,assignObjectID);
    else
        assignObjectID = 0;
        image = createImageObject(segmentationFile,detectionFile,i,imageList,assignObjectID);
    end    
    images = [images, image];
end
imagesUniqueID = objectTracker(images);

%plot objetcs of currentImage & image
%         for j=1:currentImage.nObjects
%             rgb = currentImage.uniqueRGB(j,:);
%             mask = I(:,:,1)==rgb(1) & I(:,:,2)==rgb(2) & I(:,:,3)==rgb(3);
%             I_masked = I;
%             new_mask = repmat(mask, 1,1,3);
%             I_masked(new_mask) = 0;
%             add_mask = zeros(size(new_mask), 'uint8');
%             add_mask(:,:,1) = mask*255;
%             I_masked = I_masked + add_mask;
%             figure; imagesc(I_masked);
%             end

result = {};
for i=1:length(imagesUniqueID)
    for j=1:imagesUniqueID(i).nObjects
        classGT = imagesUniqueID(i).objects(j).classGT;
        id = imagesUniqueID(i).objects(j).id;
        if i==1
            result{j,1} = classGT;
            result{j,2} = id;
        else
            [exists,index] = ismember(classGT,result(:,1));
            if ~exists
                result{end+1,1} = classGT;
                result{end,2} = id;
            else
                result{index,2} = [result{index,2},id];
            end
        end
    end
end

