%--------------------------------------------------------------------------
% Author: Yash Vyas - yjvyas@gmail.com - 18/06/18
% Contributors:
%--------------------------------------------------------------------------

function object = RBOfromEP(EP_Default)
%RBOFROMEP Creates a Rigid Body Object from an Environment Primitive

object = RigidBodyObject();
object.set('trajectory', EP_Default.get('trajectory'));
object.set('index', EP_Default.get('index'));
object.set('pointIndexes', EP_Default.get('pointIndexes'));
end

