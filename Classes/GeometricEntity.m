classdef GeometricEntity < BaseObject
    %GEOMETRICENTITY class instances are used by Sensor class instances to 
    %create a representation of geometric entities in the environment.
    %   Geometric entities are represented by a mathematical function ie
    %   plane is modelled by normal and distance
    
    %% 1. Properties
    properties
        parameters
    end
    
    %% 2. Methods
    methods(Access = public)
        function out = get(self,property)
            %output depends on varargin
            switch property
                case 'index'
                    out = self.index;
                case 'parameters'
                    out = self.parameters;
                case 'trajectory'
                    out = self.trajectory;
            end
        end
        
        function self = set(self,property,value)
            %depends on varargin
            switch property
                case 'index'
                    self.index = value;
                case 'parameters'
                    self.parameters = value;
                case 'trajectory'
                    self.trajectory = value;
            end
        end
    end
    
end

