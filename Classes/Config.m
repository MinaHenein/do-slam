classdef Config < ArrayGetSet
    %CONFIG Summary of this class goes here
    %   Detailed explanation goes here
    
    %% 1. Properties
    properties(GetAccess = 'public', SetAccess = 'private')
        %array of time values when measurements are made
        t 
        
        %measurement std dev
        stdPosePrior
        stdPointPrior
        stdPosePose   
        stdPosePoint 
        stdPointPlane
        
        %R3xso3 or logSE3
        poseParameterisation
        
        %graph file labels
        poseVertexLabel
        pointVertexLabel
        planeVertexLabel
        posePoseEdgeLabel
        posePointEdgeLabel
        pointPlaneEdgeLabel     
        
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
            if ispc
                self.sep = '\';
            elseif isunix || ismac
                self.sep = '/';
            end
            self.folderPath = pwd;
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
    
end

