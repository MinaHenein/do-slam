%--------------------------------------------------------------------------
% Author: Montiel Abello - montiel.abello@gmail.com - 23/05/17
% Contributors:
%               Mina Henein -- aded solver related parameters
%--------------------------------------------------------------------------

classdef Config < ArrayGetSet
    %CONFIG is used to store user settings
    %   Properties must be set with the 'set' method
    %   Properties can be accessed with '.' notation instead of the 'get'
    %   method for ease of use.
    %
    %   *Suggestion: rather than each user adding properties only relevant
    %   to their application, subclasses should be created with
    %   non-fundamental properties for specific applications
    
    %% 1. Properties
    properties(GetAccess = 'public', SetAccess = 'protected')
        %array of time values when measurements are made
        t
        
        % rng seed
        rngSeed
        
        % noise model
        noiseModel
        
        %measurement std dev
        stdPosePrior
        stdPointPrior
        stdPosePose
        stdPosePoint
        stdPointPlane
        
        %R3xso3 or logSE3
        poseParameterisation
        
        absoluteToRelativePoseHandle 
        absoluteToRelativePointHandle
        relativeToAbsolutePoseHandle
        relativeToAbsolutePointHandle
        
        %% TODO: both shouldnt be here!!
        cameraPointParameterisation
        cameraControlInput
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
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
        
        %graph file labels
        poseVertexLabel
        pointVertexLabel
        planeVertexLabel
        posePoseEdgeLabel
        posePointEdgeLabel
        pointPlaneEdgeLabel
        posePriorEdgeLabel
        
        %files
        sep
        folderPath
        graphFileFolderName
        groundTruthFileName
        measurementsFileName
        
    end
    
    properties (Dependent)
        %measurement covariances
        covPosePrior
        covPointPrior
        covPosePose
        covPosePoint
        covPointPlane
    end
    
    %% 2. Methods
    % Constructor
    methods(Access = public)
        function self = Config()
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
    
    % Dependent properties
    methods
        function covPosePrior = get.covPosePrior(obj)
            covPosePrior = stdToCovariance(obj.stdPosePrior);
        end
        function covPointPrior = get.covPointPrior(obj)
            covPointPrior = stdToCovariance(obj.stdPointPrior);
        end
        function covPosePose = get.covPosePose(obj)
            covPosePose = stdToCovariance(obj.stdPosePose);
        end
        function covPosePoint = get.covPosePoint(obj)
            covPosePoint = stdToCovariance(obj.stdPosePoint);
        end
        function covPointPlane = get.covPointPlane(obj)
            covPointPlane = stdToCovariance(obj.stdPointPlane);
        end
    end
    
    % initialise file stuff
    methods(Access = protected)
        function self = initPath(self)
            if ispc
                self.sep = '\';
            elseif isunix || ismac
                self.sep = '/';
            end
            self.folderPath = pwd;
        end
    end
end