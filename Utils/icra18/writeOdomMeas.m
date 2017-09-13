function writeOdomMeas(odomMeas, imuMeas, synchronisedData)

pose1ID = 1;
covariance = [0.16 0.0 0.0 0.0 0.0 0.0 0.16 0.0 0.0 0.0 0.0 0.16 0.0 0.0...
    0.0 0.00030625 0.0 0.0 0.00030625 0.0 0.00030625];

for i=2:size(synchronisedData,1)
    
    odomDt = odomMeas(synchronisedData(i,2),1) - odomMeas(synchronisedData(i-1,2),1); 
    imuDt =  imuMeas(synchronisedData(i,3),1) - imuMeas(synchronisedData(i-1,3),1);
    
    vx = odomMeas(synchronisedData(i-1,2),2);
    vy = odomMeas(synchronisedData(i-1,2),3);
    vz = odomMeas(synchronisedData(i-1,2),4);
    
    wx = imuMeas(synchronisedData(i-1,3),2);
    wy = imuMeas(synchronisedData(i-1,3),3);
    wz = imuMeas(synchronisedData(i-1,3),4);
    
    odom = [vx*odomDt,vy*odomDt,vz*odomDt,wx*imuDt,wy*imuDt,wz*imuDt];
    odomCov =  covariance;
    pose2ID = pose1ID +1;
    filePath = '/home/mina/workspace/src/Git/do-slam/Utils/icra18/';
    fID = fopen(strcat(filePath,'odometryMeasGraphFile.txt'),'a');
    format = strcat('%s %d %d ',repmat(' %6f',1,27));
    fprintf(fID,format,'EDGE_R3_SO3',pose1ID,pose2ID,odom,odomCov);
    fprintf(fID,'\n');
    fclose(fID);
    pose1ID = pose2ID;
end
end

