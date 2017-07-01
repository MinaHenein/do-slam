function planeHandle = plotPlane(normal,distance,points,planeColour)
%PLOTPLANE Summary of this function goes here
%   Detailed explanation goes here


%plot plane
%choose 4 corner points
minPositions = min(points,[],2);
maxPositions = max(points,[],2);
rangePositions = range(points,2);
%dependent variable computed from min/max positions & plane parameters
dependentVariable = (rangePositions==min(rangePositions));
variables = {'x','y','z'};
dependentVariable = variables{dependentVariable};
%looks better if z is dependent? favour this
% if rangePositions(3)~=0
%     dependentVariable = 'z';
% end
switch dependentVariable
    case 'x'
        yCorners = [minPositions(2) minPositions(2) maxPositions(2) maxPositions(2)];
        zCorners = [minPositions(3) maxPositions(3) maxPositions(3) minPositions(3)];
        xCorners = (distance - normal(2)*yCorners - normal(3)*zCorners)/normal(1);
    case 'y'
        xCorners = [minPositions(1) minPositions(1) maxPositions(1) maxPositions(1)];
        zCorners = [minPositions(3) maxPositions(3) maxPositions(3) minPositions(3)];
        yCorners = (distance - normal(1)*xCorners - normal(3)*zCorners)/normal(2);
    case 'z'
        xCorners = [minPositions(1) minPositions(1) maxPositions(1) maxPositions(1)];
        yCorners = [minPositions(2) maxPositions(2) maxPositions(2) minPositions(2)];
        zCorners = (distance - normal(1)*xCorners - normal(2)*yCorners)/normal(3);
end            
planeHandle = patch('XData',xCorners,'YData',yCorners,'ZData',zCorners); 
set(planeHandle,'FaceColor',planeColour);
set(planeHandle,'FaceAlpha',0.25);

end

