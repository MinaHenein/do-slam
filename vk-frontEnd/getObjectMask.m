function objectMask = getObjectMask(l,t,r,b,maskI)

xCentroid = (l+r)/2;
yCentroid = (t+b)/2;

maskColour = maskI(round(yCentroid),round(xCentroid),:);
objectMask = maskI(:,:,1)==maskColour(1,1,1) & ...
    maskI(:,:,2)==maskColour(1,1,2) & maskI(:,:,3)==maskColour(1,1,3);

end