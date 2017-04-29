classdef EnvironmentPrimitive < handle & matlab.mixin.Copyable
    %GEOMETRICPRIMITIVE Contains a set of Points and
    %Trajectory, used to create sensor observation objects.
    % 
    
    properties (GetAccess=protected, SetAccess=protected)
        ID
        Points
        MeshTriangles
        Trajectory
    end
    
    methods
        function obj = EnvironmentPrimitive(varargin)
            % Constructor, default is empty. Can
            % provide input arguments Points, MeshTriangles and Trajectory.
            if numel(varargin)==1
                obj.ID = varargin{1}; % Must provide ID.
            else
                error('Incorrect ID input.')
            end
            if numel(varargin)>1
                obj.set('Points',varargin{2});
            end
            if numel(varargin)>2
                obj.set('MeshTriangles',varargin{3});
            end
            if numel(varargin)>3
                obj.set('Trajectory',varargin{4});
            end
        end
              
        varargout = get(obj, field, varargin);
        obj = set(obj, field, varargin);    
        
    end
    
end

