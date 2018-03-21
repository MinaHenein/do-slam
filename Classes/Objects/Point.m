%--------------------------------------------------------------------------
% Author: Montiel Abello - montiel.abello@gmail.com - 23/05/17
% Contributors:
%--------------------------------------------------------------------------

classdef Point < ArrayGetSet
    %POINT class instances are used by Sensor class instances to create a
    %representation of points in the environment.
    %   Points are created and used to generate measurements
    
    %% 1. Properties
    properties(GetAccess = 'protected', SetAccess = 'protected')
        index
        trajectory
        objectIndexes
        vertexIndex
        vertexIndexTime
    end
    
    
    %% 2. Methods
    
    % Getter & Setter
    methods(Access = public) %set to protected later??
        function out = getSwitch(self,property,varargin)
            switch property
                case {'GP_Point','R3Position','S2xRPosition'}
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
        
        function self = clearIndex(self)
            self.vertexIndex = [];
            self.vertexIndexTime = [];
        end
    end
    
end

