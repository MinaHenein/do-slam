%--------------------------------------------------------------------------
% Author: Mina Henein - mina.henein@anu.edu.au - 30/06/17
% Contributors:
%--------------------------------------------------------------------------

classdef SolverConfig < Config
    %SOLVERCONFIG 
    %   This subclass used to add properties specific to the solver
    %   that should not be in base Config class
    %% 1. Properties
    properties(GetAccess = 'public', SetAccess = 'private')
        %dimensions
        dimPose
        dimPoint
        
        %plane parameterisation
        planeNormalParameterisation
        
        %constraints
        applyAngleConstraints
        automaticAngleConstraints
        
        %first linearisation point
        startPose
        
        %static assumption
        staticAssumption
        
        %solver settings
        sortVertices
        sortEdges
        processing
        nVerticesThreshold
        nEdgesThreshold
        solveRate
        solverType
        threshold
        maxNormDX
        maxIterations
    end
    
    %% 2. Methods
    % Constructor
    methods(Access = public) 
        function self = SolverConfig()
            self.initPath();
        end
    end
    
    % Get & Set
    methods(Access = public) 
    	function out = getSwitch(self,property)
        	out = self.(property);
        end
        
        function self = setSwitch(self,property,value)
        	self.(property) = value;
        end
    end
end

