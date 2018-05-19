function centroid = getObjectCentroid(objectMask)

[rows,cols] = find(objectMask);
centroidX = mean(cols);
centroidY = mean(rows);
centroid = [centroidX,centroidY];

end