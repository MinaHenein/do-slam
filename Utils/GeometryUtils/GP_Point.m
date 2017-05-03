classdef GP_Point < GeometricPrimitive
    %GP_POINT represents a position in 3D space
    %   currently only R3 position implemented
    
    %% 1. Properties
    properties(GetAccess = 'public', SetAccess = 'public')
        R3Position
    end
    
    %% 2. Methods
    % Getter & Setter
    methods(Access = public)
        function out = get(self,property,varargin)
            if (nargin==2)
                out = [self.(property)];
            elseif (nargin==3) %specific location required
                out = [self(varargin{1}).(property)];
            end
            
        end
        
        function self = set(self,property,values,varargin)
            if (nargin==3)
                self.(property) = value;
            elseif (nargin==4) %set specific locations
                locations = varargin{1};
                for i = 1:numel(locations)
                    self(locations(i)).(property) = values(:,i);
                end
            end
            
        end
    end
    
end

