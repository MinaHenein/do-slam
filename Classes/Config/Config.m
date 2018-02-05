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
        nSteps
        
        % rng seed
        rngSeed
        
        % noise model
        noiseModel
        
        % motion model
        motionModel
        
        % if motionModel is constantSE3Motion
        constantSE3Motion
        % if motionModel is constantAccelerationSE3Motion
        constantAccelerationSE3Motion
        
        %measurement std dev
        stdPosePrior
        stdPointPrior
        stdPosePose
        stdPosePoint
        stdPointPoint
        std3Points
        std2PointsVelocity
        stdPointPlane
        std2PointsSE3
        std2PointsSE3Motion
        
        %R3xso3 or logSE3
        poseParameterisation
        
        % measurement type for point motion - can be point2Edge, point3Edge,
        % or velocity - ONLY affects the pre-measurements file stage.
        pointMotionMeasurement
        
        % function handles moved to public temporarily
        
        %% TODO: both shouldnt be here!!
        cameraPointParameterisation
        cameraControlInput
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %dimensions
        dimPose
        dimPoint
        
        %plane parameterisation
        planeNormalParameterisation
        
        %SE3 Motion Vertex initialization
        SE3MotionVertexInitialization
        
        %constraints
        applyAngleConstraints
        automaticAngleConstraints
        
        %first linearisation point
        startPose
        
        %static assumption
        staticAssumption
        
        staticDataAssociation % this determines whether points are 
        % always data associated or whether only between 2 time steps
        % settings are 'On' meaning global data association or or 'Off'
        objectAssociation % same as above but for objects
        
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
        pointPointEdgeLabel
        point3EdgeLabel
        velocityVertexLabel
        SE3MotionVertexLabel
        pointVelocityEdgeLabel
        pointSE3MotionEdgeLabel
        pointDataAssociationLabel
        pointPointEdgeSE3Label
        pointPlaneEdgeLabel
        posePriorEdgeLabel
        angleVertexLabel
        angleEdgeLabel
        distanceVertexLabel
        distanceEdgeLabel
        pointRGBVertexLabel
        pointPointRGBEdgeLabel
        fixedAngleEdgeLabel
        fixedDistanceEdgeLabel
        
        % general settings
        displayProgress
        plotPlanes
        plotIncremental
        displaySPPARMS
        
        %files
        sep
        folderPath
        savePath
        graphFileFolderName
        groundTruthFileName
        measurementsFileName
        
    end
    
    properties(GetAccess = 'public', SetAccess = 'public')
        absoluteToRelativePoseHandle 
        absoluteToRelativePointHandle
        relativeToAbsolutePoseHandle
        relativeToAbsolutePointHandle
    end
    
    properties (Dependent)
        %measurement covariances
        covPosePrior
        covPointPrior
        covPosePose
        covPosePoint
        covPointPoint
        cov3Points
        cov2PointsVelocity
        cov2PointsSE3Motion
        cov2PointsSE3
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
        function covPointPoint = get.covPointPoint(obj)
            covPointPoint = stdToCovariance(obj.stdPointPoint);
        end
        function cov3Points = get.cov3Points(obj)
            cov3Points = stdToCovariance(obj.std3Points);
        end
        function cov2PointsVelocity = get.cov2PointsVelocity(obj)
            cov2PointsVelocity = stdToCovariance(obj.std2PointsVelocity);
        end
        function cov2PointsSE3Motion = get.cov2PointsSE3Motion(obj)
            cov2PointsSE3Motion = stdToCovariance(obj.std2PointsSE3Motion);
        end
        function cov2PointsSE3 = get.cov2PointsSE3(obj)
            cov2PointsSE3 = stdToCovariance(obj.std2PointsSE3);
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