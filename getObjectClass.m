function [class, classGT] = getObjectClass(segmentationFile, objectRGB)

class = [];
%read file
fid = fopen(segmentationFile,'r');
tline = fgetl(fid);
i = 1;
while ischar(tline) % iterate through the segmented object text file
    tline = fgetl(fid);
    if tline ~= -1
        lineStruct = strsplit(tline,' ');
    end
    % find objectRGB in segmented object colour
    if objectRGB(1) == str2double(lineStruct{2}) && objectRGB(2) == str2double(lineStruct{3})...
            && objectRGB(3) == str2double(lineStruct{4})
        class = lineStruct{1};
        if length(class)>=4 && strcmp(class(1:length('Car:')),'Car:')
            class = 'Car';
        end
        classGT = lineStruct{1};
    end
    
    i = i + 1;
end

end