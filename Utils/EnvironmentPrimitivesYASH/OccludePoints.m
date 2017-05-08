function [newPoints, indexes] = OccludePoints(Points,MeshTriangles)
%OCCLUDEPOINTS Takes a set of points and MeshTriangles, and conducts
%raytracing based occlusion to remove points that are not seen. Outputs the
%points that are occluded with optional secondary output of indexe
%correlations to the original.
nPoints = size(Points,1);
indexes = 1:nPoints;
indexes(2,:) = 1; % also stores the visibility in the second row

for i = 1:nPoints
    for j=1:size(MeshTriangles,1)
        P1 = MeshTriangles(j,1:3)'; % three vertexes of the triangle
        P2 = MeshTriangles(j,4:6)';
        P3 = MeshTriangles(j,7:9)';
        N = cross(P2-P1,P3-P1); % Normal to the plane of the triangle
        P0 = dot(P1,N)/dot(point,N)*(point); % The point of intersection (if it exists)
        if dot(cross(P2-P1,P0-P1),N)>0 && dot(cross(P3-P2,P0-P2),N)>0 && dot(cross(P1-P3,P0-P3),N)>0
            if norm(P0)<norm(point) % if distance to point is larger than distance to intersection, not visible
                indexes(2,i) = 0;
                break
            end
        end
    end
end

indexes = indexes(1,indexes(2,:)==1); % removes indexes for which visibiity is 0
newPoints = Points(indexes,:); % returns Points that aren't occluded

end

