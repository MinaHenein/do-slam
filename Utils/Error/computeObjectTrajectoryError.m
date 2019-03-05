function [r_f_translation_error, f_rot_error_degrees] = computeObjectTrajectoryError(GTObjectMotion, estimatedObjectMotion)

nObjects = size(GTObjectMotion,2); 
for i=1:nObjects

SE3MotionVertexValue = estimatedObjectMotion(:,i);
constantSE3ObjectMotion = GTObjectMotion(:,i);

v_error = AbsoluteToRelativePoseR3xso3(SE3MotionVertexValue, constantSE3ObjectMotion);
f_trans_error2 = norm(v_error(1:3))^2;
r_f_translation_error = sqrt(f_trans_error2);
fprintf('\nTranslational error of object %d:\n',i);
disp(r_f_translation_error)
f_rot_error_degrees = wrapToPi(norm(v_error(4:6))) * 180/pi;
fprintf('\nRotational error of object %d:\n',i);
disp(f_rot_error_degrees)
    
end

end