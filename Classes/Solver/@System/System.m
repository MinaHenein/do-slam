classdef System
    %SYSTEM Class represents linearised system constructed from graph
    %   properties:
    %       A - jacobian matrix
    %       b - residuals
    %       edgeHeights - vector giving dimension of each edge
    %       vertexWidths - vector giving dimension of each vertex
    %       iActiveEdges - indexes of active edges
    %
    %   see constructLinearSystem method for more information
    
    properties
        A
        b
        L
        d
        covariance
        covSqrtInv
        %block indexing properties
        edgeHeights
        vertexWidths
        iActiveEdges
        Hk
        ck
        kPerp
%         H
%         c
    end
    
    properties (Dependent)
         H
         c
        chiSquaredError
    end
    
    methods
        %% constructor
        function obj = System(config,graph,measurementsCell)
            obj = constructLinearSystem(obj,config,graph,measurementsCell);
        end
        
        %% useful transformations
        function H = get.H(obj)
            H = (obj.covSqrtInv*obj.A)'*(obj.covSqrtInv*obj.A);
        end
        
        function c = get.c(obj)
            c = (obj.covSqrtInv*obj.A)'*(obj.covSqrtInv*obj.b);
        end
        
        function chiSquaredError = get.chiSquaredError(obj)
            nEdges = size(obj.edgeHeights,1);
            chiSquared = ((obj.b)'/obj.covariance)*(obj.b);
            %normalise
            chiSquaredError =  full(chiSquared/nEdges);
        end
             
        %% declare
        obj = addEdgeUpdateHessian(obj,config,measurementsCell,edgeCell)
        obj = addEdgeUpdateCholesky(obj,config,measurementsCell,edgeCell)
    end
    
end

