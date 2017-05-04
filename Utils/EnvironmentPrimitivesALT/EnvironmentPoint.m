classdef EnvironmentPoint
    %EP_POINT Summary of this class goes here
    %   Detailed explanation goes here
    
    %% 1. Properties
    properties(GetAccess = 'private', SetAccess = 'private')
        index
        trajectory
    end
    
    %% 2. Methods
    % Getter & Setter
    methods(Access = public) %set to protected later??
        function out = get(self,property)
        	out = [self.(property)];
        end
        
        function self = set(self,property,value)
        	self.(property) = value;
        end
    end
    
    % Constructor
    methods(Access = public)
        function self = EnvironmentPoint(trajectory,index)
            switch nargin
                case 0
                case 2
                    self.trajectory = trajectory;
                    self.index      = index;
            end
        end
        
    end
    
end

