classdef PositionModelPoseTrajectory < PoseTrajectory
    %PositionModelPoseTrajectory represents a dynamic trajectory that is
    %instantiated from 3D position waypoints
    %   a position model is fitted to the waypoints and stored as a
    %   property.
    %   when the pose @ time t is requested through the get->getSwitch
    %   methods, the model is used to compute the position.
    %   the orientation is chosen by differentiating the position model at
    %   time t to get the velocity. The orientation is chosen such that x
    %   faces the direction of motion and the z axis lies in the vertical
    %   plane containing the direction of motion    
    
    %% 1. Properties
    properties(GetAccess = 'protected', SetAccess = 'protected')
    end
    
    properties(Hidden)
        xModel 
        yModel
        zModel
    end
    
    %% 2. Methods    
    % Constructor
    methods(Access = public)
        function self = PositionModelPoseTrajectory(waypoints,parameterisation,fitType)
            assert(strcmp(parameterisation,'R3'),'Error: Only R3 waypoints implemented.')
            %fit x,y,z to waypoints
            self.xModel = fit(waypoints(1,:)',waypoints(2,:)',fitType);
            self.yModel = fit(waypoints(1,:)',waypoints(3,:)',fitType);
            self.zModel = fit(waypoints(1,:)',waypoints(4,:)',fitType);
        end
        
    end
    
    % Get & Set
    methods(Access = public)
        function value = getSwitch(self,property,varargin)
            switch property
                case 'GP_Pose'
                    t     = varargin{1};
                    pose  = computePose(self,t);
                    value = pose;
                case {'R3xso3Pose','logSE3Pose','R3xso3Position','logSE3Position','axisAngle','R'}
                    t     = varargin{1};
                    pose  = computePose(self,t);
                    value = pose.get(property);
                otherwise 
                    error('Error: invalid property')
            end
        end
        
        function self = setSwitch(self,property,value,varargin)
            self.(property) = value;
        end
        
    end
    
    % Computes pose @ times t
    methods(Access = private)
        function poses = computePose(self,t)
            nPoses = numel(t);
            %compute position from x,y,z models
            position = [self.xModel(t)';
                        self.yModel(t)';
                        self.zModel(t)'];
            %differentiate models at time t to get velocity
            velocity = [differentiate(self.xModel,t)';
                        differentiate(self.yModel,t)';
                        differentiate(self.zModel,t)'];
            %assign orientation from velocity
            orientation = zeros(3,nPoses);
            for i = 1:nPoses
                orientation(:,i) = assignOrientation(velocity(:,i));
            end
            %output is GP_Pose
            poses(nPoses) = GP_Pose();
            for i = 1:nPoses
                poses(i) = GP_Pose([position(:,i); orientation(:,i)]); 
            end
        end
        
    end
    
end

