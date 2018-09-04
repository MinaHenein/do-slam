
%https://math.stackexchange.com/questions/1996586/transform-se3-pose-covariance?rq=1

function Cov7 = covAxisAngleToCovQuat (axisAngle, covAxisAngle)

R = axisAngle;
Cov6 = covAxisAngle;

phi = norm(R);
alpha = sin(phi/2)/phi;

parQR = [((0.5*cos(phi/2)-alpha)/phi^2)*R*R' + alpha*eye(3); -alpha/2*R'];
parAB = [parQR zeros(4,3); zeros(3) eye(3)];

Cov7 = parAB*Cov6*parAB';

end