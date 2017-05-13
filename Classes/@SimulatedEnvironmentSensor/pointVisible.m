function [visibility,relativePoint] = pointVisible(self,point,t)
%POINTVISIBLE Summary of this function goes here
%   Detailed explanation goes here

%GP_Point - relative position between camera, point at time t
relativePoint = point.get('trajectory').AbsoluteToRelativePoint(self.get('trajectory'),t);

%convert to spherical coords
S2xRRelativePosition = relativePoint.get('S2xRPosition');
%check if az,el,r within limits
if (S2xRRelativePosition(1) >= self.fieldOfView(1)) && (S2xRRelativePosition(1) <= self.fieldOfView(2)) &&...
   (S2xRRelativePosition(2) >= self.fieldOfView(3)) && (S2xRRelativePosition(2) <= self.fieldOfView(4)) &&...   
   (S2xRRelativePosition(3) >= self.fieldOfView(5)) && (S2xRRelativePosition(3) <= self.fieldOfView(6))
    visibility = 1;
else
    visibility = 0;
end


end
