function odometry  = GPSOdometry(gpsFile)

gpsFileID = fopen(gpsFile,'r');
gpsData = textscan(gpsFileID,'%s','delimiter','\n','whitespace',' ');
gpsCStr = gpsData{1};
fclose(gpsFileID);

odometry = zeros(6,size(gpsCStr,1)-1);
for i= 2:size(gpsCStr,1)-1
     splitLine1 = strsplit(gpsCStr{i},' ');
     lat1 = str2double(splitLine1(1));
     lon1 = str2double(splitLine1(2));
     alt1 = str2double(splitLine1(3))/1000;
     qwc1 = regexp(splitLine1(4),'\d+(\.)?(\d+)?','match');
     qw1 = str2double([qwc1{:}]);
     qx1 = str2double(splitLine1(5));
     qy1 = str2double(splitLine1(6));
     qzc1 = regexp(splitLine1(7),'\d+(\.)?(\d+)?','match');
     qz1 = str2double([qzc1{:}]);
     q1 = [qw1 qx1 qy1 qz1];
     r1 = 6371000 + alt1;
     x1 = r1*cosd(lat1)*cosd(lon1);
     y1 = r1*cosd(lat1)*sind(lon1);
     z1 = r1*sind(lat1);
     pose1 = [x1 y1 z1 q2a(q1)]';
     
     splitLine2 = strsplit(gpsCStr{i+1},' ');
     lat2 = str2double(splitLine2(1));
     lon2 = str2double(splitLine2(2));
     alt2 = str2double(splitLine2(3))/1000;
     qwc2 = regexp(splitLine2(4),'\d+(\.)?(\d+)?','match');
     qw2 = str2double([qwc2{:}]);
     qx2 = str2double(splitLine2(5));
     qy2 = str2double(splitLine2(6));
     qzc2 = regexp(splitLine2(7),'\d+(\.)?(\d+)?','match');
     qz2 = str2double([qzc2{:}]);
     q2 = [qw2 qx2 qy2 qz2];
     r2 = 6371000 + alt2;
     x2 = r2*cosd(lat2)*cosd(lon2);
     y2 = r2*cosd(lat2)*sind(lon2);
     z2 = r2*sind(lat2);
     pose2 = [x2 y2 z2 q2a(q2)]';
 
     odometry(:,i) = AbsoluteToRelativePoseR3xso3(pose1, pose2);
end

end