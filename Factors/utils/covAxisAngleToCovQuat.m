
%https://math.stackexchange.com/questions/1996586/transform-se3-pose-covariance?rq=1
% this function expects to receive a poseAxisAngle = [ax,ay,az,tx,ty,tz];
% and a covariance of the poseAxisAngle in the same order
% & and outputs a poseQuat = [qx,qy,qz,qw,tx,ty,tz] 
% and a covariance of the poseQuat in the same order 

function [poseQuat, covPoseQuat] = covAxisAngleToCovQuat (poseAxisAngle, covPoseAxisAngle)

R = poseAxisAngle(1:3);
t = poseAxisAngle(4:6);

phi = norm(R);
alpha = sin(phi/2)/phi;

qx = R(1) * alpha;
qy = R(2) * alpha;
qz = R(3) * alpha;
qw = sin(phi/2);

poseQuat = [qx, qy, qz, qw, t']';

parQR = [((0.5*cos(phi/2)-alpha)/phi^2)*(R*R') + alpha*eye(3); -alpha/2*R'];
parAB = [parQR zeros(4,3); zeros(3) eye(3)];

covPoseQuat = parAB*covPoseAxisAngle*parAB';

% force non diagonal elements to be zeros
for i=1:size(covPoseQuat,1)
    for j = 1:size(covPoseQuat,2)
        if i~=j
            covPoseQuat(i,j) = 0;
        end
    end
end

end


