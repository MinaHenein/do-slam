p1 = [389 219;
397.2 219.6;
388.8 220.8;
394.56 228.96;
375.84 220.32;
401.76 220.32;
374.976 221.184;
400.205 217.728;
371.174 221.875;
390.666 221.461;
376.234 222.157;
372.651 211.408]';

p2 = [376.928 219.727;
384.976 220.327;
376.728 221.527;
382.488 229.687;
363.01 221.047;
389.688 221.047;
362.146 221.911;
387.981 218.455;
358.193 222.602;
378.746 222.187;
363.555 222.884;
359.366 212.134]';

X1 = inv(reshape([0.8532373 0.03525124 -0.5203302 -165.7739 -0.03277691,...
    0.9993652 0.01395724 119.548 0.520492 0.005145979 0.8538511 -190.1206 0 0 0 1],[4,4])');

X2 = inv(reshape([0.8532716 0.03510126 -0.520284 -165.8084 -0.03281838,...
    0.9993688 0.01360048 119.5852 0.520433 0.005469974 0.853885 -190.0639 0 0 0 1],[4,4])');


K = [725,  0, 620.5;
    0 725, 187; 
    0 0 1];

I1 = imread('00410.png');
l1 = zeros(3,size(p1,2));
for i=1:size(p1,2)
    depth = double(I1(round(p1(2,i)),round(p1(1,i))))/100; 
    lc =  K\[p1(:,i);1];
    lc = lc*depth;
    l = X1*[lc;1];
    l1(:,i) = l(1:3);
end

I2 = imread('00411.png');
l2 = zeros(3,size(p2,2));
for i=1:size(p2,2)
    depth = double(I2(round(p2(2,i)),round(p2(1,i))))/100; 
    lc =  K\[p2(:,i);1];
    lc = lc*depth;
    l = X2*[lc;1];
    l2(:,i) = l(1:3);
end

[R, t, rmse] = Kabsch(l1,l2);
T =  [R t; 0 0 0 1];

objPoseInCam = [-5.437333 1.763065 15.4934 1.792547 -0.01278046 0.03326242];

translation = objPoseInCam(1:3);
rotation = objPoseInCam(4:6);
Ry = eul2Rot([0, rotation(1) + pi/2, 0]);
Rx = eul2Rot([0, 0, rotation(2)]);
Rz = eul2Rot([rotation(3), 0, 0]);
objectRotationinCameraFrame =  Ry * Rx * Rz; 

x3d = translation(1);
y3d = translation(2);
z3d = translation(3);
objectTranslationinCameraFrame = [x3d;y3d;z3d];
objectPoseInCameraFrame = [objectRotationinCameraFrame, objectTranslationinCameraFrame; 0 0 0 1];


objectPoseInGlobalFrame = X1 * objectPoseInCameraFrame;
nextFrameObjectPoseGlobalFrame = T * objectPoseInGlobalFrame;


nextObjPoseInCam = [-5.494761 1.762879 14.8279 1.757522 -0.01189814 0.0334819];
translation = nextObjPoseInCam(1:3);
rotation = nextObjPoseInCam(4:6);
Ry = eul2Rot([0, rotation(1) + pi/2, 0]);
Rx = eul2Rot([0, 0, rotation(2)]);
Rz = eul2Rot([rotation(3) 0, 0]);
nextObjectRotationinCameraFrame =  Ry * Rx * Rz ;
x3d = translation(1);
y3d = translation(2);
z3d = translation(3);
nextObjectTranslationinCameraFrame = [x3d;y3d;z3d];
nextObjectPoseInCameraFrame = [nextObjectRotationinCameraFrame, nextObjectTranslationinCameraFrame; 0 0 0 1];
nextObjectPoseInGlobalFrameGT = X2 * nextObjectPoseInCameraFrame;

error = nextFrameObjectPoseGlobalFrame\nextObjectPoseInGlobalFrameGT;
T_error = norm(error(1:3,4));
R_error_deg = rad2deg(norm(arot(error(1:3,1:3))));

