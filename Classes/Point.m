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
    end
    
    
    %% 2. Methods
    % Constructor
    methods(Access = public) %set to private later??
        function self = Point(environmentPoint)
            switch nargin
                case 0 %preallocate
                otherwise
                    self.index         = environmentPoint.get('index');
                    self.trajectory    = environmentPoint.get('trajectory');
                    self.objectIndexes = environmentPoint.get('primitiveIndexes');
            end
        end
        
    end
    
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
    end
    
end

