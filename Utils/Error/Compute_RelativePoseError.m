function [r_f_translation_error,r_f_rotation_error,...
    r_f_squared_translation_error,r_f_squared_rotation_error] = ...
    Compute_RelativePoseError(est_vertices,gt_vertices,v_rel_pose,n_delta)

assert(n_delta > 0); 
    
r_f_translation_error = 0;
r_f_rotation_error = 0;
r_f_squared_translation_error = 0;
r_f_squared_rotation_error = 0;

%number of degrees per one radian (for angular unit conversion)
f_deg_per_rad = 180/pi;


%gt_vertices_transformed;
assert(isequal(size(est_vertices),size(gt_vertices)));
n = size(est_vertices,2);
for i = 1:n
    
    v_gt_pose_cur = RelativeToAbsolutePoseR3xso3(gt_vertices(:,i),v_rel_pose);
    
    if(i > n_delta)
        % get the previous and current vertices
        v_gt_pose_prev = gt_vertices(:,i - n_delta); % one from n_delta vertices ago
        v_est_pose_prev = est_vertices(:,i - n_delta);
        v_est_pose_cur = est_vertices(:,i);
        
        % calculate edges between those vertices
        v_edge_est = AbsoluteToRelativePoseR3xso3(v_est_pose_prev,v_est_pose_cur);
        v_edge_gt = AbsoluteToRelativePoseR3xso3(v_gt_pose_prev,v_gt_pose_cur);
        %%
        
        % calculate the errors
        v_error = AbsoluteToRelativePoseR3xso3(v_edge_est, v_edge_gt);
        f_trans_error2 = norm(v_error(1:3))^2;
        r_f_translation_error = r_f_translation_error + sqrt(f_trans_error2);
        r_f_squared_translation_error = r_f_squared_translation_error + f_trans_error2;
        f_rot_error_degrees = wrapToPi(norm(v_error(4:6))) * f_deg_per_rad;
        r_f_rotation_error = r_f_rotation_error + f_rot_error_degrees;
        r_f_squared_rotation_error = r_f_squared_rotation_error + f_rot_error_degrees * f_rot_error_degrees;
        
    end
    
end

% normalsie by number of edges
r_f_translation_error = r_f_translation_error/(n-1);
r_f_rotation_error = r_f_rotation_error/(n-1);
r_f_squared_translation_error = r_f_squared_translation_error/(n-1); 
r_f_squared_rotation_error = r_f_squared_rotation_error/(n-1);

end
