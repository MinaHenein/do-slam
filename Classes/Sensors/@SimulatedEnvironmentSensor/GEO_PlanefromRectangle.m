%--------------------------------------------------------------------------
% Author: Montiel Abello - montiel.abello@gmail.com - 23/05/17
% Contributors:
%--------------------------------------------------------------------------

function object = GEO_PlanefromRectangle(rectangle)
%GEO_PLANEFROMRECTANGLE Creates a GEO_Plane object from an environment
%rectangle.

object = GEO_Plane();
object.set('index',rectangle.get('index'));
object.set('pointIndexes',rectangle.get('pointIndexes'));
object.set('rectangleTrajectory',rectangle.get('trajectory')); 

end

