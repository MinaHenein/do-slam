classdef DeformableObject < BaseObject
    %DEFORMABLEOBJECT class instances are used by Sensor class instances to 
    %create a representation of rigid body objects in the environment.
    %   Deformable objects have trajectory (ie a person) - will likely be
    %   composed of other attached objects with their own trajectories
    %   relative to the trajectory of the deformable object
    
     %% 1. Properties
    properties
        trajectory
    end
    
    %% 2. Methods
    methods(Access = private)
        function out = getSwitch(self,property,varargin)
            %output depends on varargin
            switch property
                case 'index'
                    out = self.index;
                case 'trajectory'
                    out = self.trajectory;
            end
        end
        
        function self = setSwitch(self,property,value,varargin)
            %depends on varargin
            switch property
                case 'index'
                    self.index = value;
                case 'trajectory'
                    self.trajectory = value;
            end
        end
    end
    
    
end

