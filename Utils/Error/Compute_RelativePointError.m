function [r_f_translation_error, r_f_squared_translation_error] = ...
    Compute_RelativePointError(est_vertices,gt_vertices,n_delta)

assert(n_delta > 0); 
    
r_f_translation_error = 0;
r_f_squared_translation_error = 0;

%gt_vertices_transformed;
assert(isequal(size(est_vertices),size(gt_vertices)));
n = size(est_vertices,2);
for i = 1:n
    
    
    if(i > n_delta)
        % get the previous and current vertices
        v_gt_point_prev = gt_vertices(:,i - n_delta); % one from n_delta vertices ago
        v_gt_point_cur = gt_vertices(:,i);
        v_est_point_prev = est_vertices(:,i - n_delta);
        v_est_point_cur = est_vertices(:,i);
       
        
        % calculate edges between those vertices
        v_edge_est = v_est_point_cur - v_est_point_prev;
        v_edge_gt = v_gt_point_cur - v_gt_point_prev;
        
        % calculate the errors
        v_error = v_edge_gt - v_edge_est;
        f_trans_error2 = norm(v_error(1:3))^2;
        r_f_translation_error = r_f_translation_error + sqrt(f_trans_error2);
        r_f_squared_translation_error = r_f_squared_translation_error + f_trans_error2;
        
    end
  
end

% normalise errors by number of edges
r_f_translation_error = r_f_translation_error/(n-1); 
r_f_squared_translation_error = r_f_squared_translation_error/(n-1);

end
