function GTPoses = getVICONGroundtruth(filePath,fileName)

fid = fopen(strcat(filePath,fileName));
tline = fgetl(fid);
position = 0;
orientation = 0;
nextLinePosition = 0;
nextLineOrientation = 0;
GTPoses = [];

while (~feof(fid))
    if length(tline)>14 && strcmp(tline(1:14),'  translation:')
        nextLinePosition = 1;
    end
    if length(tline)>11 && strcmp(tline(1:11),'  rotation:')
        nextLineOrientation = 1;
    end
    if length(tline)>7 && nextLinePosition && strcmp(tline(1:7),'    x: ')
        pos_x = str2double(tline(8:end));
    end
    if length(tline)>7 && nextLinePosition && strcmp(tline(1:7),'    y: ')
        pos_y = str2double(tline(8:end));
    end
    if length(tline)>7 && nextLinePosition && strcmp(tline(1:7),'    z: ')
        pos_z = str2double(tline(8:end));
        nextLinePosition = 0;
        position = 1;
    end
    if length(tline)>7 && nextLineOrientation && strcmp(tline(1:7),'    x: ')
        qx = str2double(tline(8:end));
    end
    if length(tline)>7 && nextLineOrientation && strcmp(tline(1:7),'    y: ')
        qy = str2double(tline(8:end));
    end
    if length(tline)>7 && nextLineOrientation && strcmp(tline(1:7),'    z: ')
        qz = str2double(tline(8:end));
    end
    if length(tline)>7 && nextLineOrientation && strcmp(tline(1:7),'    w: ')
        qw = str2double(tline(8:end));
        nextLineOrientation = 0;
        orientation = 1;
    end
    if position && orientation
        pose = [pos_x,pos_y,pos_z,q2a([qw,qx,qy,qz])];
        GTPoses = [GTPoses;pose];
        position = 0;
        orientation = 0;
        nextLinePosition = 0;
        nextLineOrientation = 0;
    end
    tline = fgetl(fid);    
end
fclose(fid);

end