classdef EPEllipsoid < GeometricPrimitive
    %GPELLIPSOID Class to implement an Ellipsoid in an environment, points
    %are generated along a mesh representation.
    
    properties
    end
    
    methods
        function obj = GPEllipsoid(varargin)
            if nargin==0
                params = [1 1 2 10];
            elseif nargin>=2
                if size(varargin{1},1)==4
                    params = varargin{1};
                else
                    error('First input argument for GPEllipsoid must have 4 parameters.')
                end
            else
                error('Incorrect input arguments to create Ellipsoid')
            end
            % get ellipsoid points and set them here
            obj.addFeatures(generateEllipsoidPoints(params));
            if varargin{2}
                if isa(varargin{2},'Trajectory')
                    obj.Trajectory = Trajectory;
                else
                    error('Second input must be a Trajectory.')
                end
            end
            
            % add bit for generating Mesh according to Delaunay Triangles
        end
                    
    end
    
end

