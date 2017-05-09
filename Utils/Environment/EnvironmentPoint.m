classdef EnvironmentPoint < ArrayGetSet & matlab.mixin.Heterogeneous
    %EnvironmentPoint Summary of this class goes here
    %   Detailed explanation goes here
    
    %% 1. Properties
    properties(GetAccess = 'protected', SetAccess = 'protected')
        index
        trajectory
        primitiveIndexes
    end
    
    %% 2. Methods
    % Constructor
    methods(Access = public)
        function self = EnvironmentPoint(index,trajectory)
            switch nargin
                case 0
                otherwise
                    self.trajectory = trajectory;
                    self.index      = index;
            end
        end
        
    end
    
    % Getter & Setter
    methods(Access = public) %set to protected later??
        function out = getSwitch(self,property,varargin)
            switch property
                case {'GP_Point','R3Position'}
                    out = self.trajectory.get(property,varargin{1});
                case 'static'
                    out = self.trajectory.get(property);
                otherwise
                    out = self.(property);
            end
        	
        end
        
        function self = setSwitch(self,property,value)
        	self.(property) = value;
        end
    end
end

