%--------------------------------------------------------------------------
% Author: Yash Vyas - yjvyas@gmail.com - 18/06/18
% Contributors:
%--------------------------------------------------------------------------

function object = RBOfromEP(environmentPrimitive)
%RBOFROMEP Creates a Rigid Body Object from an Environment Primitive

object = RigidBodyObject();
self.set('trajectory', environmentPrimitive.get('trajectory'));
self.set('index', environmentPrimitive.get('index'));
self.set('pointIndexes', environmentPrimitive.get('pointIndexes'));
end

