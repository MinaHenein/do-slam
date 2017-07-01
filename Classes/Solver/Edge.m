classdef Edge
    %EDGE represents an edge in the graph of the system
    %   type can be...
    
    properties
        value
        covariance
        jacobians
        type 
        iVertices
        index
        active
    end
        
    methods
        %% constructor
        function obj = Edge(varargin)
            switch nargin
                case 0 %preallocation
                case 6
                    obj.value        = varargin{1};
                    obj.covariance   = varargin{2};
                    obj.jacobians    = varargin{3};
                    obj.type         = varargin{4};
                    obj.iVertices    = varargin{5};
                    obj.index        = varargin{6};
                    %default active
                    obj.active = 1;
            end
        end
        

    end
    
end

