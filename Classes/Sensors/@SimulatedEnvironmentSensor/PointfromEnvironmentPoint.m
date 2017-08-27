%--------------------------------------------------------------------------
% Author: Montiel Abello - montiel.abello@gmail.com - 23/05/17
% Contributors: Yash Vyas - yjvyas@gmail.com - 15/07/18
%--------------------------------------------------------------------------

function point = PointfromEnvironmentPoint(environmentPoint)
%POINTFROMENVIRONMENTPOINT Creates a sensor point from environment Point.

point = Point();
point.set('index',environmentPoint.get('index'));
point.set('trajectory',environmentPoint.get('trajectory'));
if ~isempty(environmentPoint.get('primitiveIndexes'))
    point.set('objectIndexes',environmentPoint.get('primitiveIndexes'));
end

end

