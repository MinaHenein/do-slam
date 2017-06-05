function pointVisibility = checkOcclusion(point, meshes)
%Checks for occlusion of a point, given the relative coordinate of the
%point in the camera frame and set of triangles given in the same reference
%frame.

pointVisibility = 1; % assuming point is visible in the first place
for i=1:size(meshes,1)
    P1 = meshes(i,1:3)'; % three vertexes of the triangle
    P2 = meshes(i,4:6)';
    P3 = meshes(i,7:9)';
    N = cross(P2-P1,P3-P1); % Normal to the plane of the triangle
    P0 = dot(P1,N)/dot(point,N)*(point); % The point of intersection (if it exists)
    if dot(cross(P2-P1,P0-P1),N)>0 && dot(cross(P3-P2,P0-P2),N)>0 && dot(cross(P1-P3,P0-P3),N)>0
        if norm(P0)<norm(point) % if distance to point is larger than distance to intersection, not visible
            pointVisibility = 0;
            break
        end
    end
end

end

