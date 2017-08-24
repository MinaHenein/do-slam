function [points,label] = extractLabelLandmarks(I,siftThreshold,rightLeftPixels)

points = sift(rgb2gray(I),'Threshold',siftThreshold);
figure
imshow(I);
hold on
plotsiftframe(points)
hold off
imH = size(I,1);
imW = size(I,2);
points(1,:) = round(points(1,:));
points(2,:) = round(points(2,:));
rightColor = -ones(size(points,2),1);
leftColor = -ones(size(points,2),1);
for j=1:size(points,2)
    right = I(points(2,j),min(points(1,j)+rightLeftPixels,imW),:);
    left = I(points(2,j),max(points(1,j)-rightLeftPixels,1),:);
    if right(1,1,1)-right(1,1,2)>50 && right(1,1,1)-right(1,1,3)>50 && right(1,1,1)>50
        rightColor(j) = 1;
    elseif right(1,1,2)-right(1,1,1)>50 && right(1,1,2)-right(1,1,3)>50 && right(1,1,2)>50
        rightColor(j) = 2;
    elseif right(1,1,3)-right(1,1,2)>50 && right(1,1,3)-right(1,1,1)>50 && right(1,1,3)>50
        rightColor(j) = 3;
    elseif abs(right(1,1,1)-right(1,1,2))<50 && abs(right(1,1,2)-right(1,1,3))<50 &&...
            abs(right(1,1,1)-right(1,1,3))<50
        rightColor(j) = 0;
    end
    
    if left(1,1,1)-left(1,1,2)>50 && left(1,1,1)-left(1,1,3)>50 && left(1,1,1)>50
        leftColor(j) = 1;
    elseif left(1,1,2)-left(1,1,1)>50 && left(1,1,2)-left(1,1,3)>50 && left(1,1,2)>50
        leftColor(j) = 2;
    elseif left(1,1,3)-left(1,1,2)>50 && left(1,1,3)-left(1,1,1)>50 && left(1,1,3)>50
        leftColor(j) = 3;
    elseif abs(left(1,1,1)-left(1,1,2))<50 && abs(left(1,1,2)-left(1,1,3))<50 &&...
            abs(left(1,1,1)-left(1,1,3))<50
        leftColor(j) = 0;
    end
end
label = zeros(size(points,2),1);
for k=1:size(points,2)
    if rightColor(k)==3 && leftColor(k)==1; label(k) = 1;
    elseif rightColor(k)==1 && leftColor(k)==0; label(k) = 2;
    elseif rightColor(k)==0 && leftColor(k)==3; label(k) = 3;
    elseif rightColor(k)==1 && leftColor(k)==3; label(k) = 4;
    elseif rightColor(k)==0 && leftColor(k)==1; label(k) = 5;
    elseif rightColor(k)==3 && leftColor(k)==0; label(k) = 6;
    end
end

for j=1:size(points,2)-1
    for k=j+1:size(points,2)
        if(points(1,j)==points(1,k) && points(2,k)==points(2,k))
            label(k) = 0;
        end
    end
end

points(:,label==0) = [];
label(~any(label,2),:) = [];

pointsCopy1 = points;
pointsCopy2 = points;
labelCopy = label;

toDelete=[];
for k=1:size(labelCopy,1)-1
    for m=k+1:size(labelCopy,1)
        if(labelCopy(k)==labelCopy(m))
            toDelete = [toDelete,k,m];
        end
    end
end

pointsCopy1(:,unique(toDelete))=[];
% calculate centroid of all other labeled points
av = mean(pointsCopy1,2);
centroid = av(1:2,1);
% get distance centroid to deleted points
uniqueToDelete=unique(toDelete);
distances=zeros(length(uniqueToDelete),1);
for j=1:length(uniqueToDelete)
    distances(j) = sqrt((centroid(1)-pointsCopy2(1,uniqueToDelete(j)))^2 +...
        (centroid(2)-pointsCopy2(2,uniqueToDelete(j)))^2);
end

toBeDeleted={};
toBeDeletedDistances={};
for k=1:length(uniqueToDelete)-1
    for m=k+1:length(uniqueToDelete)
        if(labelCopy(uniqueToDelete(k))==labelCopy(uniqueToDelete(m)))
            toBeDeleted{k,end+1} = uniqueToDelete(k);
            toBeDeleted{k,end+1} = uniqueToDelete(m);
            toBeDeletedDistances{k,end+1}=distances(k);
            toBeDeletedDistances{k,end+1}=distances(m);
        end
    end
end

deleteFromPoints = [];
for i=1:size(toBeDeleted,1)
    % delete furthest points from centroid
    [~,indx]= min(cell2mat(toBeDeletedDistances(i,:)));
    toBeDeletedRow = cell2mat(toBeDeleted(i,:));
    deleteFromPoints = [deleteFromPoints, ...
        toBeDeletedRow(toBeDeletedRow~=toBeDeletedRow(indx))];
end

points(:,deleteFromPoints)=[];
label(deleteFromPoints)=[];

end