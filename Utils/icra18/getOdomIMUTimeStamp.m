filePath = '/home/mina/Downloads/icra18/';
odomFileID = fopen(strcat(filePath,'odom.txt'));
imuFileID = fopen(strcat(filePath,'imu.txt'));
odomLine = fgetl(odomFileID);
imuLine = fgetl(imuFileID);
odomTimeStamp =[];
imuTimeStamp =[];

while ischar(odomLine)
    if length(odomLine)>10 && strcmp(odomLine(1:10),'    secs: ')
        odomTime = str2double(odomLine(11:end));
        odomTimeStamp = [odomTimeStamp;odomTime];
    end
    if length(odomLine)>10 && strcmp(odomLine(1:10),'    secs: ')
        imuTime = str2double(odomLine(11:end));
        imuTimeStamp = [imuTimeStamp;imuTime];
    end
    odomLine = fgetl(odomFileID);
    imuLine = fgetl(imuFileID);
end
fclose(odomFileID);
fclose(imuFileID);

save('odomTimeStamp','odomTimeStamp');
save('imuTimeStamp','imuTimeStamp');