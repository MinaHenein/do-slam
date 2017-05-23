%--------------------------------------------------------------------------
% Author: Montiel Abello - montiel.abello@gmail.com - 23/05/17
% Contributors:
%--------------------------------------------------------------------------

classdef DeformableObject < Object
    %DEFORMABLEOBJECT class instances are used by Sensor class instances to 
    %create a representation of rigid body objects in the environment.
    %   Deformable objects have trajectory (ie a person) - will likely be
    %   composed of other attached objects with their own trajectories
    %   relative to the trajectory of the deformable object
    
     %% 1. Properties
    properties(GetAccess = 'protected', SetAccess = 'protected')
        trajectory
    end
    
    %% 2. Methods    
    
end

