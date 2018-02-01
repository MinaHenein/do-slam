function [r_f_translation_error,r_f_rotation_error,r_f_squared_translation_error,...
    r_f_squared_rotation_error] = Compute_RelativePoseError_AllToAll(est_vertices,gt_vertices,v_rel_pose)

r_f_translation_error = 0;
r_f_rotation_error = 0;
r_f_squared_translation_error = 0;
r_f_squared_rotation_error = 0;

assert(isequal(size(est_vertices),size(gt_vertices)));

% calculate RPE for all the possible deltas
n = size(est_vertices,2);
for n_delta = 1:n-1
    [f_translation_error, f_rotation_error, f_squared_translation_error,...
        f_squared_rotation_error] = Compute_RelativePoseError(est_vertices, gt_vertices, v_rel_pose, n_delta);
    r_f_translation_error = r_f_translation_error + f_translation_error;
    r_f_rotation_error = r_f_rotation_error + f_rotation_error;
    r_f_squared_translation_error = r_f_squared_translation_error + f_squared_translation_error;
    r_f_squared_rotation_error = r_f_squared_rotation_error + f_squared_rotation_error;
end


% divide by number of edges
r_f_translation_error = r_f_translation_error/(n/2);
r_f_rotation_error = r_f_rotation_error/(n/2);
r_f_squared_translation_error = r_f_squared_translation_error/(n/2);
r_f_squared_rotation_error =r_f_squared_rotation_error/(n/2);

end
		
        
