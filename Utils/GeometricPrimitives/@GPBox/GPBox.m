classdef GPBox < GeometricPrimitive
    %GPBox Contains a mathematical representation for a box
    %geometric primitive i.e parameters needed to create a box geometric
    %primitive (length, width, height)
    
    properties(GetAccess='public',SetAccess='public')% to change later to private
        length
        width
        height
    end
    
    methods
        
        function obj = GPBox(varargin)
            % Constructor, default is empty. Can
            % provide input arguments type, and pose.
            obj.type = 'box';
            if nargin >= 0
                disp('Initializing a GPBox with no parameters')
            end
            if nargin == 1
                params = varargin{1};
                if length(params) == 1
                    obj.set('length', params(1))
                    obj.set('width',  params(1))
                    obj.set('height', params(1))
                    disp('Initializing a GPBox with 1 parameter only; a CUBE')
                elseif length(params) == 2
                    obj.set('length', params(1))
                    obj.set('width',  params(1))
                    obj.set('height', params(2))
                    
                    disp(['Warning! Initializing a GPBox with 2 parameters only'...
                    'width and length will be considered the same'])
                elseif length(params) == 3
                obj.set('length', params(1))
                obj.set('width',  params(2))
                obj.set('height', params(3))
                else
                    error('Trying to initialize a GPBox with too many input parameters')
                end
            end
                obj.params = [obj.length, obj.width, obj.height];
            if nargin == 2
                obj.set('pose',varargin{2});
            elseif nargin > 2
                error('Initializing a geometric primitive box with too many input arguments');
            end
            
        end
        
        varargout = get(obj, field, varargin);
        obj = set(obj, field, varargin);  
    end
    
end