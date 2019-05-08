function objectPoints = vKITTI_pointObservability(filePath)

fid = fopen(filePath, 'r');
Data = textscan(fid,'%s','Delimiter','\n');
CStr = Data{1};
fclose(fid);

pointsDataAssociationIndex = find(contains(CStr, '2PointsDataAssociation')==1);
posesIndex = find(contains(CStr, 'VERTEX_POSE_R3_SO3')==1);

objectPoints = {};
seenObjects = [];

for i=1:length(pointsDataAssociationIndex)
    line = strsplit(CStr{pointsDataAssociationIndex(i),1},' ');
    pointId1 = str2double(line{1,2});
    pointId2 = str2double(line{1,3});
    objectId =  str2double(line{1,4});
    if pointId2 == 630
    end
    frame = length(find(posesIndex(posesIndex(:,1)<pointsDataAssociationIndex(i))));
    if ~ismember(objectId, seenObjects)
        seenObjects = [seenObjects, objectId];
        objectPoints{end+1,frame} = [pointId1 pointId2];
    else
        indx = find(seenObjects == objectId);
        iObjectPoints = [];
        for j = 1:size(objectPoints(indx,:),2)
            iObjectPoints = [iObjectPoints [objectPoints{indx,j}]];
            if  ~isempty(iObjectPoints)
                if pointId1 == iObjectPoints(end)
                    frame = j;
                    if ~ismember(pointId2,[objectPoints{indx,frame}])
                        objectPoints{indx,frame} = [objectPoints{indx,frame} pointId2];
                        break;
                    end
                elseif size(objectPoints(indx,:),2) < frame && j == size(objectPoints(indx,:),2)
                    objectPoints{indx,frame} = [pointId1 pointId2];
                    break;
                end
            end
        end
    end
end


end