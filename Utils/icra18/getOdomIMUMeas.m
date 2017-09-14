function [odomMeas,imuMeas] = getOdomIMUMeas()

filePath = '/home/mina/Downloads/icra18/';
odomFileID = fopen(strcat(filePath,'odom.txt'));
imuFileID = fopen(strcat(filePath,'imu.txt'));
odomLine = fgetl(odomFileID);
imuLine = fgetl(imuFileID);
odomTimeStamped = 0;
imuTimeStamped = 0;
linearData = 0;
odomAngularData = 0;
angularData = 0;
nextLineLinearData = 0;
nextLineOdomAngularData = 0;
nextLineAngularData = 0; 
odomMeas = [];
imuMeas = [];

firstOdomTime = 1504686340;
previousOdomTime = firstOdomTime;

while ischar(odomLine)
    if length(odomLine)>10 && strcmp(odomLine(1:10),'    secs: ')
        odomTime = str2double(odomLine(11:end));
        odomTimeStamped = 1;
    end
    if length(odomLine)==12 && strcmp(odomLine(1:12),'    linear: ')
        nextLineLinearData = 1;
    end
    if length(odomLine)>9 && nextLineLinearData && strcmp(odomLine(1:9),'      x: ')
        vx = str2double(odomLine(10:end));
    end
    if length(odomLine)>9 && nextLineLinearData && strcmp(odomLine(1:9),'      y: ')
        vy = str2double(odomLine(10:end));
    end
    if length(odomLine)>9 && nextLineLinearData && strcmp(odomLine(1:9),'      z: ')
        vz = str2double(odomLine(10:end));
        nextLineLinearData = 0;
        linearData = 1;
    end
    
    if length(odomLine)==13 && strcmp(odomLine(1:13),'    angular: ')
        nextLineOdomAngularData = 1;
    end
    if length(odomLine)>9 && nextLineOdomAngularData && strcmp(odomLine(1:9),'      x: ')
        odomwx = str2double(odomLine(10:end));
    end
    if length(odomLine)>9 && nextLineOdomAngularData && strcmp(odomLine(1:9),'      y: ')
        odomwy = str2double(odomLine(10:end));
    end
    if length(odomLine)>9 && nextLineOdomAngularData && strcmp(odomLine(1:9),'      z: ')
        odomwz = str2double(odomLine(10:end));
        nextLineOdomAngularData = 0;
        odomAngularData = 1;
    end
    
    if odomTimeStamped && linearData && odomAngularData
        dt = odomTime - previousOdomTime;
        odomMeas = [odomMeas;[odomTime,vx*dt,vy*dt,vz*dt,odomwx*dt,odomwy*dt,odomwz*dt]];
        previousOdomTime = odomTime;
        odomTimeStamped = 0;
        linearData = 0;
        odomAngularData = 0;
        nextLineLinearData = 0;
        nextLineOdomAngularData = 0;
    end
    odomLine = fgetl(odomFileID);
end
fclose(odomFileID);

firstIMUTime = 1504686340;
previousIMUTime = firstIMUTime;

while ischar(imuLine)
    if length(imuLine)>10 && strcmp(imuLine(1:10),'    secs: ')
        imuTime = str2double(imuLine(11:end));
        imuTimeStamped = 1;
    end
    if length(imuLine)==18 && strcmp(imuLine(1:18),'angular_velocity: ')
        nextLineAngularData = 1;
    end
    if length(imuLine)>5 && nextLineAngularData && strcmp(imuLine(1:5),'  x: ')
        wx = str2double(imuLine(6:end));
    end
    if length(imuLine)>5 && nextLineAngularData && strcmp(imuLine(1:5),'  y: ')
        wy = str2double(imuLine(6:end));
    end
    if length(imuLine)>5 && nextLineAngularData && strcmp(imuLine(1:5),'  z: ')
        wz = str2double(imuLine(6:end));
        nextLineAngularData = 0;
        angularData = 1;
    end
    if imuTimeStamped && angularData
        dt = imuTime - previousIMUTime;
        imuMeas = [imuMeas;[imuTime,wx*dt,wy*dt,wz*dt]];
        previousIMUTime = imuTime;
        imuTimeStamped = 0;
        angularData = 0;
        nextLineAngularData = 0;
    end
    imuLine = fgetl(imuFileID);
end
fclose(imuFileID);