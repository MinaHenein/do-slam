
function varargout = plotCoordinates(o, R, color) 
% plots coordinates at origin o and rotated with R

if nargin == 2
    color = 'rgb';
end

quiver3( o(1), o(2), o(3), R(1,1), R(2,1), R(3,1),color(1));hold on
quiver3( o(1), o(2), o(3), R(1,2), R(2,2), R(3,2),color(2));
quiver3( o(1), o(2), o(3), R(1,3), R(2,3), R(3,3),color(3));%hold off

end 