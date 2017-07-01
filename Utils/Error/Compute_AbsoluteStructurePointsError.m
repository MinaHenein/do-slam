function [r_f_translation_error,r_f_squared_translation_error] = ...
    Compute_AbsoluteStructurePointsError(est_vertices,gt_vertices)

r_f_translation_error = 0;
r_f_squared_translation_error = 0;

assert(isequal(size(est_vertices),size(gt_vertices)));
n = size(est_vertices,2);
for i = 1:n

    % calculate the errors
    v_error = gt_vertices(:,i) - est_vertices(:,i);
    f_trans_error2 = norm(v_error(1:3))^2;
    r_f_translation_error = r_f_translation_error + sqrt(f_trans_error2);
    r_f_squared_translation_error = r_f_squared_translation_error + f_trans_error2;
        
end

% normalise errors by number of vertices
r_f_translation_error = r_f_translation_error/n;
r_f_squared_translation_error = r_f_squared_translation_error/n;

end
