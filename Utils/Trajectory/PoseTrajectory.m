classdef PoseTrajectory < Trajectory
    %PoseTrajectory Summary of this class goes here
    %   Detailed explanation goes here
    
    %% 1. Properties
    properties(GetAccess = 'protected', SetAccess = 'protected')
        poses
    end
    
    %% 2. Methods
    % Getter & Setter
    methods(Access = public) %set to protected later??
        function out = get(self,property,varargin)
            if (nargin==2)
                out = [self.(property)];
            elseif (nargin==4) && strcmp(property,'poses')
                switch varargin{1}
                    case 'timeStep'
                        %get poses of each trajectory for time step varargin{2}
                        assert(numel(varargin{2})==1,'Unsafe to get poses for multiple trajectories at multiple times')
                        nTrajectories = numel(self);
                        out(nTrajectories) = GP_Pose();
                        for i = 1:nTrajectories
                            out(i) = self(i).poses(varargin{2});
                        end
                    case 'time'
                        %get poses of each trajectory for time varargin{2}
                        nTrajectories = numel(self);
                        out(nTrajectories) = GP_Pose();
                        for i = 1:nTrajectories
                            poseLogical = (self(i).t==varargin{2});
                            assert(sum(poseLogical)==1,'Exactly 1 pose must exist at input time for each trajectory')
                            out(i) = self(i).poses(poseLogical);
                        end
                end
            end
            
        end
        
        function self = set(self,property,value)
        	self.(property) = value;
        end
    end
    
    % Constructor
    methods(Access = public) %set to private later??
        function self = PoseTrajectory(mode,varargin)
            switch nargin
                case 0
                    %allows pre-allocation
                otherwise
                    switch mode
                        case 'waypoints'
                            parameterisation = varargin{1};
                            assert(strcmp(parameterisation,'R3'),'Error: Only R3 waypoints implemented.')
                            waypoints = varargin{2};
                            tFit      = varargin{3};
                            fitType   = varargin{4};
                            self.fitTrajectory(waypoints,tFit,fitType);
                        case 'discrete'
                            parameterisation = varargin{1};
                            assert(any(strcmp(parameterisation,{'logSE3','R3xso3'})),'Error: only logSE3 and R3xso3 pose parameterisation implemented.')
                            dataPoints = varargin{2};
                            self.t = dataPoints(1,:);
                            nPoses = numel(self.t);
                            GPPoses(nPoses) = GP_Pose;
                            GPPoses.set(strcat(parameterisation,'Pose'),dataPoints(2:7,:),[1:nPoses]);
                            self.poses = GPPoses;
                        case 'stationary'
                            parameterisation = varargin{1};
                            assert(any(strcmp(parameterisation,{'logSE3','R3xso3'})),'Error: only logSE3 and R3xso3 pose parameterisation implemented.')
                            self.t = varargin{2};
                            nPoses = numel(self.t);
                            dataPoints = repmat(varargin{3},1,nPoses);
                            nPoses = numel(self.t);
                            GPPoses(nPoses) = GP_Pose;
                            GPPoses.set(strcat(parameterisation,'Pose'),dataPoints,[1:nPoses]);
                            self.poses = GPPoses;
                        case 'continuous'
                            self.model = varargin{1};
                    end
            end
        end
        
    end
    
    % Fitting
    methods(Access = private)
        function self = fitTrajectory(self,waypoints,tFit,fitType)
            nPoses = numel(tFit);
            
            %models
            fX = fit(waypoints(1,:)',waypoints(2,:)',fitType);
            fY = fit(waypoints(1,:)',waypoints(3,:)',fitType);
            fZ = fit(waypoints(1,:)',waypoints(4,:)',fitType);
            
            %use model to get positions
            poses = zeros(6,numel(tFit));
            poses(1,:) = fX(tFit)';
            poses(2,:) = fY(tFit)';
            poses(3,:) = fZ(tFit)';

            %X-axis forward
            forward = [1,0,0]';
            up      = [0,0,1]';
            %compute orientation to face direction of motion
            for i = 1:numel(tFit)-1
                v = poses(1:3,i+1) - poses(1:3,i);
                poses(4:6,i) = assignOrientation(v);
                
            end
            poses(4:6,end) = poses(4:6,end-1); %LAST POSE?
            
            %dataPoints
            self.t = tFit;
            GPPoses(nPoses) = GP_Pose;
            GPPoses.set('R3xso3Pose',poses,[1:nPoses]);
            self.poses = GPPoses;
        end
    end
    
    % Plotting
    methods(Access = public)
        function plot(self,varargin)
            poses = self.poses.get('R3xso3Pose');
            
            %plot positions
            plot3(poses(1,:),poses(2,:),poses(3,:),'k.')
            
            %plot axes
            for i = 1:size(poses,2)
                scale = 0.5;
                plotCoordinates(poses(1:3,i),scale*rot(poses(4:6,i)))
            end
        end
    end
    
end

