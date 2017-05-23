%--------------------------------------------------------------------------
% Author: Montiel Abello - montiel.abello@gmail.com - 23/05/17
% Contributors:
%--------------------------------------------------------------------------

classdef RigidBodyObject < Object
    %RIGIDBODYOBJECT class instances are used by Sensor class instances to 
    %create a representation of rigid body objects in the environment.
    %   Rigid body objects have trajectory (ie a car) - will likely be
    %   composed of other attached objects fixed to the rigid body
    
     %% 1. Properties
    properties(GetAccess = 'protected', SetAccess = 'protected')
        trajectory
    end
    
    %% 2. Methods
    
end

