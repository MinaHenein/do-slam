function syncedData = synchroniseData(stereoTimeStampsFile, gpsTimeStampsFile)

% get stereo time stamps
stereoFileID = fopen(stereoTimeStampsFile,'r');
stereoData = textscan(stereoFileID,'%s','delimiter','\n','whitespace',' ');
stereoCStr = stereoData{1};
fclose(stereoFileID);
stereoTimeStamps = str2double(stereoCStr);

% get gps time stamps
gpsFileID = fopen(gpsTimeStampsFile,'r');
gpsData = textscan(gpsFileID,'%s','delimiter','\n','whitespace',' ');
gpsCStr = gpsData{1};
fclose(gpsFileID);

gpsTimeStamps = zeros(size(gpsCStr,1)-1,1);
for i= 2:size(gpsCStr,1)
     splitLine = strsplit(gpsCStr{i},' ');
     gpsTimeStamps(i,1) = str2double(splitLine(end));
end


% for every stereo image, find the closest time stamp in gps reading
nImages = size(stereoCStr,1);
syncedData = zeros(nImages,2);

for i=1:nImages
    stereoTimeStamp = stereoTimeStamps(i,1);
    gpsTimeGap = abs(bsxfun(@minus,gpsTimeStamps,stereoTimeStamp));
    [~,closestGPS] = min(gpsTimeGap);
    syncedData(i,:) = [i, closestGPS];
end


end