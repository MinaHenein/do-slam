%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 08/05/2019
% Contributors:
%--------------------------------------------------------------------------
% Transforms object SE3 motion (m-degree/frame) to linear velocity (km/hr) for driving applications
%--------------------------------------------------------------------------

function objectVelocities = objectMotionToVelocity(objectMotions, fps)

objectVelocities = zeros(1,size(objectMotions,2));

for i =1:size(objectMotions,2)
    objectMotion = objectMotions(:,i);
    translationNorm = norm(objectMotion(1:3));
    objectVelocity = translationNorm * fps * 3.6;
    objectVelocities(1,i) = objectVelocity;
end
    
end