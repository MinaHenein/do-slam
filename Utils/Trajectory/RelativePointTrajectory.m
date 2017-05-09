classdef RelativePointTrajectory < PointTrajectory
    %RELATIVEPOINTTRAJECTORY Summary of this class goes here
    %   Detailed explanation goes here
    
    %% 1. Properties
    properties(GetAccess = 'protected', SetAccess = 'protected')
        referenceTrajectory
        relativePoint
    end
    
    %% 2. Methods
    % Constructor
    methods(Access = public)
        function self = RelativePointTrajectory(referenceTrajectory,relativePoint)
            self.referenceTrajectory = referenceTrajectory;
            self.relativePoint       = relativePoint;
        end
    end
    
    % Get & Set
    methods(Access = public)
        function value = getSwitch(self,property,varargin)
            switch property
                case 'GP_Point'
                    t             = varargin{1};
                    referencePose = self.referenceTrajectory.get('GP_Pose',t);
                    value         = self.relativePoint.RelativeToAbsolutePoint(referencePose);
                case {'R3Position'}
                    t             = varargin{1};
                    referencePose = self.referenceTrajectory.get('GP_Pose',t);
                    pose          = self.relativePoint.RelativeToAbsolutePoint(referencePose);
                    value         = pose.get(property);
                case 'static'
                    value = self.referenceTrajectory.get('static');
                otherwise 
                    error('Error: invalid property')
            end
        end
        
        function self = setSwitch(self,property,value,varargin)
            self.(property) = value;
        end
        
    end
    
end

