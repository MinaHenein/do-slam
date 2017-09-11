filePath = '/home/mina/Downloads/icra18/';
odomFileID = fopen(strcat(filePath,'odom.txt'));
imuFileID = fopen(strcat(filePath,'imu.txt'));
odomLine = fgetl(odomFileID);
imuLine = fgetl(imuFileID);
timeStamped = 0;
linearData = 0;
angularData = 0;
nextLineLinearData = 0;
nextLineAngularData = 0; 
pose1ID = 1;
covariance = [0.16 0.0 0.0 0.0 0.0 0.0 0.16 0.0 0.0 0.0 0.0 0.16 0.0 0.0...
    0.0 0.010966 0.0 0.0 0.010966 0.0 0.010966];
while ischar(odomLine)
    if length(odomLine)>10 && strcmp(odomLine(1:10),'    secs: ')
        time = str2double(odomLine(11:end));
        if pose1ID == 1
            firstTime = time;
            timeStamped = 1;
        elseif time-previousTime >= 0
            timeStamped = 1;
        end
    end
    if length(odomLine)==12 && strcmp(odomLine(1:12),'    linear: ')
        nextLineLinearData = 1;
    end
    if length(imuLine)==18 && strcmp(imuLine(1:18),'angular_velocity: ')
        nextLineAngularData = 1;
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
    if timeStamped && linearData && angularData
        pose2ID = pose1ID + 1;
        if pose1ID ==1
            dt = time - firstTime;
        else
            dt = time - previousTime;
        end
        odomMeas = [vx*dt,vy*dt,vz*dt,wx*dt,wy*dt,wz*dt];
        odomMeasCov =  covariance;
        filePath = '/home/mina/workspace/src/Git/do-slam/Utils/icra18/';
        fID = fopen(strcat(filePath,'odometryMeasGraphFile.txt'),'a');
        format = strcat('%6f %s %d %d ',repmat(' %6f',1,27));
        fprintf(fID,format,time,'EDGE_R3_SO3',pose1ID,pose2ID,odomMeas,odomMeasCov);
        fprintf(fID,'\n');
        fclose(fID);
        pose1ID = pose1ID + 1;
        previousTime = time;
        timeStamped = 0;
        linearData = 0;
        angularData = 0;
        nextLineLinearData = 0;
        nextLineAngularData = 0;
    end
    odomLine = fgetl(odomFileID);
    imuLine = fgetl(imuFileID);
end

fclose(odomFileID);
fclose(imuFileID);