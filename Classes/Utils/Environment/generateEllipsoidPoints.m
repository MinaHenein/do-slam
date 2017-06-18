%--------------------------------------------------------------------------
% Author: Yash Vyas - yjvyas@gmail.com - 18/06/17
% Contributors:
%--------------------------------------------------------------------------

function [positions, links] = generateEllipsoidPoints(radii, N)
%GENERATEELLIPSOIDPOINTS Generates all the points on an ellipsoid, and
%provides the Delaunay triangulation links between them. Does not do random
%sampling for these points, this is done in addEllipsoid in the
%Environment. Positions are in relative coordinates.

[x, y, z] = ellipsoid(0, 0, 0, radii(1), radii(2), radii(3), N);
x = reshape(x, [1 (N+1)*(N+1)]);
y = reshape(y, [1 (N+1)*(N+1)]);
z = reshape(z, [1 (N+1)*(N+1)]);

positions = [x; y; z];
positions = unique(positions','rows')';

tetrahedra = delaunay(positions(1,:),positions(2,:),positions(3,:));
links = [tetrahedra(:,[1 2 3]); tetrahedra(:,[1 2 4]); tetrahedra(:,[1 3 4]); tetrahedra(:, [2 3 4])];
links = sort(links, 2);
links = unique(links, 'rows');

end

