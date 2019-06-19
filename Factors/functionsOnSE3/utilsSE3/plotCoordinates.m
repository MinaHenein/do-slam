
function varargout = plotCoordinates(o, R, color) 
% plots coordinates at origin o and rotated with R

if nargin == 2
    color = 'rgb';
end

s = 1;

quiver3( o(1), o(2), o(3), s*R(1,1), s*R(2,1), s*R(3,1),color(1));hold on
quiver3( o(1), o(2), o(3), s*R(1,2), s*R(2,2), s*R(3,2),color(2));
quiver3( o(1), o(2), o(3), s*R(1,3), s*R(2,3), s*R(3,3),color(3));%hold off

end 