function synchronisedData = synchroniseROSVICONData(rgbTimeStamp,odomTimeStamp,...
    imuTimeStamp,GTCameraPoseTimeStamp,GTObj1PoseTimeStamp,GTObj2PoseTimeStamp)

%--------------------------------------------------------------------------
% Author: Mina Hnenein - mina.henein@anu.edu.au - 11/09/17
% Contributors:
%--------------------------------------------------------------------------

synchronisedData = zeros(size(rgbTimeStamp,1),6);

for i=1:size(rgbTimeStamp,1)
    % find closest time stamped msg in odom and imu data
    rgbTime = rgbTimeStamp(i,1);
    odomTimeGap = abs(bsxfun(@minus,odomTimeStamp(:,1),rgbTime));
    imuTimeGap = abs(bsxfun(@minus,imuTimeStamp(:,1),rgbTime));
    cameraTimeGap = abs(bsxfun(@minus,GTCameraPoseTimeStamp(:,1),rgbTime));
    obj1TimeGap = abs(bsxfun(@minus,GTObj1PoseTimeStamp(:,1),rgbTime));
    obj2TimeGap = abs(bsxfun(@minus,GTObj2PoseTimeStamp(:,1),rgbTime));
    
    [~,closestOdom] = min(odomTimeGap);
    [~,closestIMU] = min(imuTimeGap);
    [~,closestCamera] = min(cameraTimeGap);
    [~,closestObj1] = min(obj1TimeGap);
    [~,closestObj2] = min(obj2TimeGap);
    
    synchronisedData(i,:) = [i,closestOdom,closestIMU,...
        closestCamera,closestObj1,closestObj2];

end
