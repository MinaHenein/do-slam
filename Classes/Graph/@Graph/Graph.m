classdef Graph
    %GRAPH represents system with vertices and edges
    %   Vertices represent linearisation points of system.
    %   Edges store edge value, jacobians, covariances and associated
    %   observation.
    %   Linear system, represented by System class is  constructed directly
    %   from graph
    %   For more information, see computeVertices and computeEdges methods
    
    properties
        vertices
        edges
        
        %store indexes for easy access. add desired properties and adjust
        %identifyTypes method to store indexes. Do this for indexes that
        %will need to be accessed frequently. Otherwise, just use
        %identifyVertices or identifyEdges methods
        iPoseVertices
        iPointVertices
        iPosePoseEdges
        iPosePointEdges
    end
    
    properties (Dependent)
        nVertices
        nEdges
    end
    
    methods    
        %% constructor
        function obj = Graph(varargin)
            switch nargin
                case 0 %incremental
                    obj.vertices = Vertex();
                    obj.edges    = Edge();
                case 2 %build from graph file cell array
                    config = varargin{1};
                    obj = obj.graphFileToGraph(config,varargin{2});
                case 3 %batch
                    camera = varargin{1};
                    measurements = varargin{2};
                    tree = varargin{3};
                    %construct linearisation points and edges
                    obj = obj.constructVertices(camera,measurements,tree);
                    obj = obj.constructEdges(measurements,tree);
                    obj = obj.identifyTypes();
            end
            
        end
                
        %% sizes
        function nVertices = get.nVertices(obj)
            nVertices = numel(obj.vertices);
            if nVertices==1 && isempty(obj.vertices(1).index)
                nVertices = 0;
            end
        end
        function nEdges = get.nEdges(obj)
            nEdges = numel(obj.edges);
            if nEdges==1 && isempty(obj.edges(1).index)
                nEdges = 0;
            end
        end
        
        %% declare
        solver = process(obj,camera,measurements,tree,processing);
        obj = obj.buildGraph(map,camera,tree);
        obj.saveGraphFile(fileName);
    end
    
end

