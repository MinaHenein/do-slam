function writeOdomMeas3(odomMeas, imuMeas, synchronisedData)

pose1ID = 1;
% 0.04 m, 1 deg = 0.0175 rad 
covariance = [0.0016 0.0 0.0 0.0 0.0 0.0 0.0016 0.0 0.0 0.0 0.0 0.0016 0.0 0.0...
    0.0 0.00030625 0.0 0.0 0.00030625 0.0 0.00030625];

for i=2:size(synchronisedData,1)
     
    dx = 0;
    dy = 0;
    dz = 0;
    ox = 0;
    oy = 0;
    oz = 0;
    
    odomDt = odomMeas(synchronisedData(i,2),1) - odomMeas(synchronisedData(i-1,2),1); 
    imuDt =  imuMeas(synchronisedData(i,3),1) - imuMeas(synchronisedData(i-1,3),1);
    
    
    for j=synchronisedData(i-1,2)+1:synchronisedData(i,2)
        
        %odomDt = odomMeas(j,1) - odomMeas(j-1,1);
        
        vx = odomMeas(j-1,2);
        vy = odomMeas(j-1,3);
        vz = odomMeas(j-1,4);
        
        dx = dx + vx*odomDt;
        dy = dy + vy*odomDt;
        dz = dz + vz*odomDt;
    end
    
    for k=synchronisedData(i-1,3)+1:synchronisedData(i,3)
        
        %imuDt = imuMeas(k,1) - imuMeas(k-1,1);
        
        wx = imuMeas(k-1,2);
        wy = imuMeas(k-1,3);
        wz = imuMeas(k-1,4);
        
        ox = ox + wx*imuDt;
        oy = oy + wy*imuDt;
        oz = oz + wz*imuDt;
        
    end
    
    odom = [dx,dy,dz,ox,oy,oz];
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

