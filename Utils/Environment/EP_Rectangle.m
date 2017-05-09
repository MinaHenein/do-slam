classdef EP_Rectangle < EnvironmentPrimitive
    %EP_RECTANGLE Summary of this class goes here
    %   Detailed explanation goes here
    
    %% 1. Properties
    properties
        sideLengths
    end
    
    %% 2. Methods
    % Constructor
    methods(Access = public)
        function self = EP_Rectangle(sideLengths,trajectory)
            self.sideLengths = sideLengths;
            self.trajectory  = trajectory;
        end
        
    end
    
    % Get & Set
    methods(Access = public)
        function out = getSwitch(self,property,varargin)
            switch property
                case {'GP_Pose','R3xso3Pose','logSE3Pose','R3xso3Position','logSE3Position','axisAngle','R'}
                    out = self.trajectory.get(property,varargin{1});
                otherwise
                    out = self.(property);
            end
        	
        end
        
        function self = setSwitch(self,property,value,varargin)
        	self.(property) = value;
        end
        
    end
    
end

