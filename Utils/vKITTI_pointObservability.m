function objectPoints = vKITTI_pointObservability(gtFilePath,measFilePath)

fid = fopen(gtFilePath, 'r');
gtData = textscan(fid,'%s','Delimiter','\n');
gtCStr = gtData{1};
fclose(fid);

fid = fopen(measFilePath, 'r');
measData = textscan(fid,'%s','Delimiter','\n');
measCStr = measData{1};
fclose(fid);

pointsDataAssociationIndex = find(contains(measCStr, '2PointsDataAssociation')==1);
posesIndex = find(contains(gtCStr, 'VERTEX_POSE_R3_SO3')==1);

objectPoints = {};
seenObjects = [];

for i=1:length(pointsDataAssociationIndex)
    line = strsplit(measCStr{pointsDataAssociationIndex(i),1},' ');
    pointId1 = str2double(line{1,2});
    pointId2 = str2double(line{1,3});
    objectId =  str2double(line{1,4});
    frame = length(find(posesIndex(posesIndex(:,1)<pointsDataAssociationIndex(i))));
    if ~ismember(objectId, seenObjects)
        seenObjects = [seenObjects, objectId];
        indx = find(seenObjects == objectId);
        objectPoints{indx,frame-1} = pointId1;
        objectPoints{indx,frame} = pointId2;
    else
        indx = find(seenObjects == objectId);
        if frame >  length(objectPoints(indx,:))
            if frame-1 > length(objectPoints(indx,:))
                objectPoints{indx,frame-1} = pointId1;
            else
                objectPoints{indx,frame-1} = [objectPoints{indx,frame-1} pointId1];
            end
%             objectPoints{indx,frame-1} = [objectPoints{indx,frame-1} pointId1];
            objectPoints{indx,frame} = pointId2;
        else
        if ~ismember(pointId1,[objectPoints{indx,frame-1}])
            objectPoints{indx,frame-1} = [objectPoints{indx,frame-1} pointId1];
        end
        if ~ismember(pointId2,[objectPoints{indx,frame}])
            objectPoints{indx,frame} = [objectPoints{indx,frame} pointId2];
        end
        end
%         iObjectPoints = [];
%         for j = 1:size(objectPoints(indx,:),2)
%             iObjectPoints = [iObjectPoints [objectPoints{indx,j}]];
%             if  ~isempty(iObjectPoints)
%                 if pointId1 == iObjectPoints(end)
%                     frame = j;
%                     if ~ismember(pointId2,[objectPoints{indx,frame}])
%                         objectPoints{indx,frame} = [objectPoints{indx,frame} pointId2];
%                         break;
%                     end
%                 elseif size(objectPoints(indx,:),2) < frame && j == size(objectPoints(indx,:),2)
%                     objectPoints{indx,frame} = [pointId1 pointId2];
%                     break;
%                 else
%                     objectPoints{indx,frame} = [[objectPoints{indx,frame}] pointId1 pointId2];
%                 end
%             end
%         end
    end
end


end