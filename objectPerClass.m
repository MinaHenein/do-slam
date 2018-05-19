function [nObjectPerClass, uniqueClasses] = objectPerClass(image)
%determine number of objects of each class in an image
nObjects = image.nObjects;
uniqueClasses = unique(image.objects.class);
nClasses =  length(uniqueClasses);
nObjectPerClass = zeros(nClasses,1);

for i=1:nObjects
    for j=1:nClasses
        if strcmp(image.objects(i).class,uniqueClasses(j))
            nObjectPerClass(j) = nObjectPerClass(j) + 1;
        end
    end
end

end