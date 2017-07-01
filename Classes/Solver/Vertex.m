classdef Vertex
    %VERTEX represents vertex in graph of system
    %   type can be...
    
    properties
        value
        covariance
        type 
        iEdges
        index
        colour
    end
    
    
    methods
        %% constructor
        function obj = Vertex(varargin)
            switch nargin
                case 0 %initialise empty - for preallocation of object array
                case 5
                    obj.value      = varargin{1};
                    obj.covariance = varargin{2};
                    obj.type       = varargin{3};
                    obj.iEdges     = varargin{4};
                    obj.index      = varargin{5};

            end
        end
        
    end    
    
end

