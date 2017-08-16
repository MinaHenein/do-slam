%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 10/08/17
% Contributors:
%--------------------------------------------------------------------------
% Testing the below equation assuming constantSE3 transformation applied to
% an object
% l^i_k = L_(k+1) * (_kH_k+1)^-1 * (L_(k+1))^-1 * l^i_(k+1)

nSteps  = 3;
%% set up object
objPtsRelative = {[0 1 0]',[1 -1 1]',[1 1 1]'};

% applies relative motion - rotation of pi/6 radians per time step about z
% axis and -pi/4 radians about y axis with linear translation of x = 1 and
% y = 2
objectPose = [5 0 0 0 0 0]'; % initial object pose - moved 5 forward on x axis
rotationMatrix = rot([pi/6;-pi/4;0]);
translationVector = [1;2;0];
objectRelativePose = [translationVector; arot(rotationMatrix)];
H = [rotationMatrix, translationVector; 0 0 0 1];

for i=2:nSteps
    objectPose(:,i) = RelativeToAbsolutePoseR3xso3(objectPose(:,i-1),...
        objectRelativePose);
end

objectPts = objPtsRelative;

for j=1:size(objectPts,2)
    objectPts{j} = RelativeToAbsolutePositionR3xso3(objectPose,...
        repmat(objectPts{j},1,nSteps));
end

Pt1DiffTimes = objectPts{1,1};
Pt2DiffTimes = objectPts{1,2};
Pt3DiffTimes = objectPts{1,3};

l11 = [Pt1DiffTimes(:,1);1];
l12 = [Pt1DiffTimes(:,2);1];
l13 = [Pt1DiffTimes(:,3);1];

l21 = [Pt2DiffTimes(:,1);1];
l22 = [Pt2DiffTimes(:,2);1];
l23 = [Pt2DiffTimes(:,3);1];

l31 = [Pt3DiffTimes(:,1);1];
l32 = [Pt3DiffTimes(:,2);1];
l33 = [Pt3DiffTimes(:,3);1];

objPose1 = [rot(objectPose(4:6,1)) objectPose(1:3,1); 0 0 0 1];
objPose2 = [rot(objectPose(4:6,2)) objectPose(1:3,2); 0 0 0 1];
objPose3 = [rot(objectPose(4:6,3)) objectPose(1:3,3); 0 0 0 1];

objPtsRel = [objPtsRelative{1,1},objPtsRelative{1,2},objPtsRelative{1,3}];
[R2,t2] = Kabsch(objPtsRel,[l12(1:3),l22(1:3),l32(1:3)]) ;
T2 = [R2 t2; 0 0 0 1]; % should be equivalen to objPose2

% get new values
shouldBel11 = objPose2*inv(H)*inv(objPose2)*l12;
shouldBel12 = objPose3*inv(H)*inv(objPose3)*l13;

shouldBel21 = objPose2*inv(H)*inv(objPose2)*l22;
shouldBel22 = objPose3*inv(H)*inv(objPose3)*l23; 

shouldBel31 = objPose2*inv(H)*inv(objPose2)*l32;
shouldBel32 = objPose3*inv(H)*inv(objPose3)*l33;

% Assertion
assert(abs(shouldBel11(1)-l11(1))<1e-10)
assert(abs(shouldBel11(2)-l11(2))<1e-10)
assert(abs(shouldBel11(3)-l11(3))<1e-10)

assert(abs(shouldBel12(1)-l12(1))<1e-10)
assert(abs(shouldBel12(2)-l12(2))<1e-10)
assert(abs(shouldBel12(3)-l12(3))<1e-10)

assert(abs(shouldBel21(1)-l21(1))<1e-10)
assert(abs(shouldBel21(2)-l21(2))<1e-10)
assert(abs(shouldBel21(3)-l21(3))<1e-10)

assert(abs(shouldBel22(1)-l22(1))<1e-10)
assert(abs(shouldBel22(2)-l22(2))<1e-10)
assert(abs(shouldBel22(3)-l22(3))<1e-10)

assert(abs(shouldBel31(1)-l31(1))<1e-10)
assert(abs(shouldBel31(2)-l31(2))<1e-10)
assert(abs(shouldBel31(3)-l31(3))<1e-10)

assert(abs(shouldBel32(1)-l32(1))<1e-10)
assert(abs(shouldBel32(2)-l32(2))<1e-10)
assert(abs(shouldBel32(3)-l32(3))<1e-10)