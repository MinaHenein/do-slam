function objectMask = getObjectMask(l,t,r,b,maskI,maskColour,settings)

if isempty(maskColour)
    xCentroid = (l+r)/2;
    yCentroid = (t+b)/2;
    maskColour = maskI(round(yCentroid),round(xCentroid),:);
end

if strcmp(settings.dataset,'kitti')
    if maskColour == 0 
        objectMask = [];
    else
        objectMask = maskI(:,:,:) == maskColour;
    end
elseif strcmp(settings.dataset,'vkitti')
    objectMask = maskI(:,:,1)==maskColour(1,1,1) & ...
        maskI(:,:,2)==maskColour(1,1,2) & maskI(:,:,3)==maskColour(1,1,3);
end



end