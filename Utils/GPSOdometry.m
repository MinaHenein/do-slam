function [cameraPoses, odometry]  = GPSOdometry(gpsFile,syncedData)

gpsFileID = fopen(gpsFile,'r');
gpsData = textscan(gpsFileID,'%s','delimiter','\n','whitespace',' ');
gpsCStr = gpsData{1};
fclose(gpsFileID);

cameraPoses = zeros(6,size(syncedData,1));
odometry = zeros(6,size(syncedData,1)-1);
for i= 1:size(syncedData,1)-1
     splitLine1 = strsplit(gpsCStr{syncedData(i,2)+1},' ');
     lat1 = str2double(splitLine1(1));
     lon1 = str2double(splitLine1(2));
     alt1 = str2double(splitLine1(3))/1000;
     qwc1 = splitLine1(4);
     qwc1 = qwc1{1,:};
     qw1 = str2double(qwc1(2:end));
     qx1 = str2double(splitLine1(5));
     qy1 = str2double(splitLine1(6));
     qzc1 = splitLine1(7);
     qzc1 = qzc1{1,:};
     qz1 = str2double(qzc1(1:end-2));
     q1 = [qw1 qx1 qy1 qz1];
     r1 = 6371000 + alt1;
     x1 = r1*cosd(lat1)*cosd(lon1);
     y1 = r1*cosd(lat1)*sind(lon1);
     z1 = r1*sind(lat1);
     axisAngle1 = q2a(q1);
     %axisRot1 = axisAngle1/norm(axisAngle1);
     %angleRot1 = norm(axisAngle1);
     %xEast = axisRot1(1); yNorth = axisRot1(2); zUp = axisRot1(3);
     %spheroid = referenceSphere('Earth');
     %[X1,Y1,Z1] = enu2ecef(xEast,yNorth,zUp,lat1,lon1,alt1,spheroid);
     %pose1 = [x1 y1 z1 X1*angleRot1 Y1*angleRot1 Z1*angleRot1]';
     pose1 = [x1 y1 z1 axisAngle1(1) axisAngle1(2) axisAngle1(3)]';
     
     splitLine2 = strsplit(gpsCStr{syncedData(i+1,2)+1},' ');
     lat2 = str2double(splitLine2(1));
     lon2 = str2double(splitLine2(2));
     alt2 = str2double(splitLine2(3))/1000;
     qwc2 = splitLine2(4);
     qwc2 = qwc2{1,:};
     qw2 = str2double(qwc2(2:end));
     qx2 = str2double(splitLine2(5));
     qy2 = str2double(splitLine2(6));
     qzc2 = splitLine2(7);
     qzc2 = qzc2{1,:};
     qz2 = str2double(qzc2(1:end-2));
     q2 = [qw2 qx2 qy2 qz2];
     r2 = 6371000 + alt2;
     x2 = r2*cosd(lat2)*cosd(lon2);
     y2 = r2*cosd(lat2)*sind(lon2);
     z2 = r2*sind(lat2);
     axisAngle2 = q2a(q2);
     %axisRot2 = axisAngle2/norm(axisAngle2);
     %angleRot2 = norm(axisAngle2);
     %xEast = axisRot2(1); yNorth = axisRot2(2); zUp = axisRot2(3);
     %[X2,Y2,Z2] = enu2ecef(xEast,yNorth,zUp,lat2,lon2,alt2,spheroid);
     %pose2 = [x2 y2 z2 X2*angleRot2 Y2*angleRot2 Z2*angleRot2]';
     pose2 = [x2 y2 z2 axisAngle2(1) axisAngle2(2) axisAngle2(3)]';
     
     cameraPoses(:,i) = pose1;
     cameraPoses(:,i+1) = pose2;
     odometry(:,i) = AbsoluteToRelativePoseR3xso3(pose1, pose2);
end

end