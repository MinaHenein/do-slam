function [tx,ty,tz,ox,oy,oz,translation,rotation,tx_points,ty_points,tz_points,...
    translation_points,tx_pointsStatic,ty_pointsStatic,tz_pointsStatic,...
    translation_pointsStatic,tx_pointsDynamic,ty_pointsDynamic,tz_pointsDynamic,...
    translation_pointsDynamic] = computeDistanceAndRotation(gtFilePath, staticPointIndices, dynamicPointIndices)
% GT
fileID = fopen(strcat(pwd,'/Data/GraphFiles/',gtFilePath),'r');
Data = textscan(fileID, '%s', 'delimiter', '\n', 'whitespace', '');
CStr = Data{1};
fclose(fileID);

nPoses = 0;
for i=1:1:length(CStr)
    splitLine = strsplit(CStr{i,1},' ');
    label = splitLine{1,1};
    if strcmp(label(1:length('VERTEX_POSE')),'VERTEX_POSE')
        nPoses = nPoses+1;
        value = str2double(splitLine(3:end));
        poses(:,nPoses) = value'; 
    end
end

tx = 0; ty = 0; tz = 0;
ox = 0; oy = 0; oz = 0;
translation = 0;
rotation = 0;
deg_per_rad = 180/pi;

for i=1:nPoses-1
    relativePose = AbsoluteToRelativePoseR3xso3(poses(:,i),poses(:,i+1));
    translation = translation + norm(relativePose(1:3));
    rotation = rotation + wrapToPi(norm(relativePose(4:6))) * deg_per_rad;
    tx = tx + norm(relativePose(1));
    ty = ty + norm(relativePose(2));
    tz = tz + norm(relativePose(3));
    R = rot(relativePose(4:6));
    eulXYZ = rot2Eul(R,'XYZ');
    ox = ox + norm(eulXYZ(1));
    oy = oy + norm(eulXYZ(2));
    oz = oz + norm(eulXYZ(3));
end

fileID = fopen(strcat(pwd,'/Data/GraphFiles/',gtFilePath),'r');
Data = textscan(fileID, '%s', 'delimiter', '\n', 'whitespace', '');
CStr = Data{1};
fclose(fileID);

nPoints = 0;
nStaticPoints = 0;
nDynamicPoints = 0;
for i=1:1:length(CStr)
    splitLine = strsplit(CStr{i,1},' ');
    label = splitLine{1,1};
    if strcmp(label(1:length('VERTEX_POINT')),'VERTEX_POINT')
        nPoints = nPoints+1;
        value = str2double(splitLine(3:end));
        id = str2double(splitLine(2));
        points(:,nPoints) = value'; 
        if ismember(id,staticPointIndices)
            nStaticPoints = nStaticPoints +1;
            staticPoints(:,nStaticPoints) = value';
        elseif ismember(id,dynamicPointIndices)
            nDynamicPoints = nDynamicPoints +1;
            dynamicPoints(:,nDynamicPoints) = value';
        end
    end
end

tx_points = 0; ty_points = 0; tz_points = 0;
translation_points = 0;

for i=1:nPoints-1
    distance = points(:,i+1)-points(:,i);
    translation_points = translation_points + norm(distance);
    tx_points = tx_points + norm(distance(1));
    ty_points = ty_points + norm(distance(2));
    tz_points = tz_points + norm(distance(3));
end

% static
nStaticPoints = length(staticPointIndices);
tx_pointsStatic = 0; ty_pointsStatic = 0; tz_pointsStatic = 0;
translation_pointsStatic = 0;
for i=1:nStaticPoints-1
    distance = staticPoints(:,i+1)-staticPoints(:,i);
    translation_pointsStatic = translation_pointsStatic + norm(distance);
    tx_pointsStatic = tx_pointsStatic + norm(distance(1));
    ty_pointsStatic = ty_pointsStatic + norm(distance(2));
    tz_pointsStatic = tz_pointsStatic + norm(distance(3));
end

% dynamic
nDynamicPoints = length(dynamicPointIndices);
tx_pointsDynamic = 0; ty_pointsDynamic = 0; tz_pointsDynamic = 0;
translation_pointsDynamic = 0;
for i=1:nDynamicPoints-1
    distance = dynamicPoints(:,i+1)-dynamicPoints(:,i);
    translation_pointsDynamic = translation_pointsDynamic + norm(distance);
    tx_pointsDynamic = tx_pointsDynamic + norm(distance(1));
    ty_pointsDynamic = ty_pointsDynamic + norm(distance(2));
    tz_pointsDynamic = tz_pointsDynamic + norm(distance(3));
end
end