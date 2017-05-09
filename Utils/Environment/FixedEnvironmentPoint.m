classdef FixedEnvironmentPoint < EnvironmentPoint
    %EP_POINT Summary of this class goes here
    %   Detailed explanation goes here
    
    %% 1. Properties
    properties(GetAccess = 'protected', SetAccess = 'protected')
        referencePrimitiveIndex
    end
    
    %% 2. Methods
    % Constructor
    methods(Access = public)
        function self = FixedEnvironmentPoint(index,trajectory,referencePrimitiveIndex)
            switch nargin
                case 0
                otherwise
                    self.index                   = index;
                    self.trajectory              = trajectory;
                    self.referencePrimitiveIndex = referencePrimitiveIndex;

            end
        end
        
    end
    
    % Getter & Setter
    methods(Access = public) %set to protected later??
        function out = getSwitch(self,property,varargin)
            switch property
                case {'GP_Point','R3Position'}
                    out = self.trajectory.get(property,varargin{1});
                otherwise
                    out = self.(property);
            end
        	
        end
        
        function self = setSwitch(self,property,value)
        	self.(property) = value;
        end
    end
    
    
    
end

