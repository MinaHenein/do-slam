
function varargout = plotCoordinates(o, R) 
% plots coordinates at origin o and rotated with R

quiver3( o(1), o(2), o(3), R(1,1), R(2,1), R(3,1),'r');hold on
quiver3( o(1), o(2), o(3), R(1,2), R(2,2), R(3,2),'g');
quiver3( o(1), o(2), o(3), R(1,3), R(2,3), R(3,3),'b');

end 