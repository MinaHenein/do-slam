classdef GeometricPrimitive < handle & matlab.mixin.Copyable
    %GEOMETRICPRIMITIVE Contains a mathematical representation for each
    %geometric primitive i.e parameters needed to create such a geometric
    %primitive
    
    properties(GetAccess='public',SetAccess='public')% to change later to private
        type
        params
        pose
    end
    
    methods
        
        function obj = GeometricPrimitive(varargin)
            % Constructor, default is empty. Can
            % provide input arguments type, and pose.
            if nargin >= 1
                obj.type = varargin{1};
            end
            if nargin == 2
                obj.set('pose',varargin{2});
            end
            if nargin > 2 
                error('Initializing a geometric primitive with too many input arguments');
            end
        end
        
        varargout = get(obj, field, varargin);
        obj = set(obj, field, varargin);  
    end
    
end

