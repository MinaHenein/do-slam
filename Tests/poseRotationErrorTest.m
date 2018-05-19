%testPoseRotationError
%vkitti GT poses
posesGT = [43.9271   44.8080   45.6034   46.3907   47.2525   48.0287   48.8009   49.6436   50.4081   51.2341
-226.6234 -227.1014 -227.5347 -227.9699 -228.4491 -228.8892 -229.3306 -229.8149 -230.2536 -230.7388
112.6500  112.6500  112.6500  112.6500  112.6500  112.6500  112.6500  112.6501  112.6500  112.6500
0.0210    0.0227    0.0227    0.0234    0.0224    0.0239    0.0249    0.0265    0.0278    0.0293
0.0069    0.0064    0.0054    0.0026    0.0028    0.0045    0.0066    0.0076    0.0069    0.0044
-0.4953   -0.5006   -0.5058   -0.5107   -0.5163   -0.5215   -0.5259   -0.5305   -0.5340   -0.5378];

poses =  zeros(6,size(posesGT,2));

for i=1:size(poses,2)
    GTRotation = rot(posesGT(4:6,i));
    noisyRotation = rot([0;0;deg2rad(10*i)])* GTRotation;
    poses(:,i) = [posesGT(1:3,i);arot(noisyRotation)];    
end

v_rel_pose = AbsoluteToRelativePoseR3xso3(posesGT(:,1),posesGT(:,1));
[ATE_translation_error,ATE_rotation_error,...
    ATE_squared_translation_error,ATE_squared_rotation_error,absRotError] = ...
    Compute_AbsoluteTrajectoryError(poses,posesGT,v_rel_pose);

n_delta = 1;
[RPE_translation_error,RPE_rotation_error,...
    RPE_squared_translation_error,RPE_squared_rotation_error,relRotError] = ...
    Compute_RelativePoseError(poses,posesGT,v_rel_pose,n_delta);

[AARPE_translation_error,AARPE_rotation_error,AARPE_squared_translation_error,...
    AARPE_squared_rotation_error] = ...
    Compute_RelativePoseError_AllToAll(poses,posesGT,v_rel_pose);

