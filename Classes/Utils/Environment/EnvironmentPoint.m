classdef EnvironmentPoint < ArrayGetSet & matlab.mixin.Heterogeneous
    %EnvironmentPoint represents point in environment
    %   Environment points are typically initialised by creating points on
    %   an EnvironmentPrimitive. The trajectories of these points are
    %   generated as relative to the trajectory of the primitive
    %   The primitiveIndexes stores indexes of primitives the point is to
    %   be associated with. *NOTE: By default, this will contain the index 
    %   of the primitive used to generate the point, but it does not have
    %   to. primitiveIndexes can contain indexes of any primitives that the
    %   point should be constrained to when generating a graph file
    
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

