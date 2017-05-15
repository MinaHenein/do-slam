classdef EnvironmentPrimitive < handle & matlab.mixin.Copyable
    %ENVIRONMENTPRIMITIVE Contains a set of Points and
    %Trajectory, used to create sensor observation selfects.
    % 
    
    properties (GetAccess=protected, SetAccess=protected)
        index
        Points
        MeshTriangles
        Trajectory
    end
    
    methods
        function self = EnvironmentPrimitive(varargin)
            % Constructor, default is empty. Can
            % provide input arguments Points, MeshTriangles and Trajectory.
            if numel(varargin)==1
                self.index = varargin{1}; % Must provide ID.
            else
                error('Incorrect ID input.')
            end
            if numel(varargin)>1
                self.set('Points',varargin{2});
            end
            if numel(varargin)>2
                self.set('MeshTriangles',varargin{3});
            end
            if numel(varargin)>3
                self.set('Trajectory',varargin{4});
            end
        end
              
        varargout = get(self, field, varargin);
        self = set(self, field, varargin);    
        
    end
    
end

