%--------------------------------------------------------------------------
% Author: Montiel Abello - montiel.abello@gmail.com - 23/05/17
% Contributors: Yash Vyas - yjvyas@gmail.com - 30/05/17
%--------------------------------------------------------------------------

classdef RigidBodyObject < SensorObject
    %RIGIDBODYOBJECT class instances are used by Sensor class instances to 
    %create a representation of rigid body objects in the environment.
    %   Rigid body objects have trajectory (ie a car) - will likely be
    %   composed of other attached objects fixed to the rigid body
    
     %% 1. Properties
    properties(GetAccess = 'protected', SetAccess = 'protected')
        trajectory
        mesh
    end
    
    %% 2. Methods
    methods
        function out = getSwitch(self,property,varargin)
            out = self.(property); 
        end
        
        function self = setSwitch(self,property,value,varargin)
        	self.(property) = value;
        end    
    end
end

