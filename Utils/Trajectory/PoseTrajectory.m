classdef PoseTrajectory < Trajectory
    %PoseTrajectory Summary of this class goes here
    %   Detailed explanation goes here
    
    %% 1. Properties
    properties
        %TODO
        %poses property - GP_Pose class array
        %get.dataPoints method - get pose data from GP_Pose array +
        %varargin for options
    end
    
    %% 2. Methods
    % Constructor
    methods(Access = public) %set to private later??
        function self = PoseTrajectory(parameterisation,mode,varargin)
            switch nargin
                case 0
                    %allows pre-allocation
                otherwise
                    assert(any(strcmp(parameterisation,{'R3xSO3','SE3'})),'Error: Only R3xSO3 and SE3 trajectories implemented.')
                    self.parameterisation = parameterisation;
                    switch mode
                        case 'waypoints'
                            waypoints = varargin{1};
                            tFit      = varargin{2};
                            fitType   = varargin{3};
                            self.fitTrajectory(waypoints,tFit,fitType);
                        case 'discrete'
                            self.dataPoints = varargin{1};
                        case 'continuous'
                            self.model = varargin{1};
                    end
            end
        end
        
    end
    
    % Fitting
    methods(Access = private)
        function self = fitTrajectory(self,waypoints,tFit,fitType)
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
            
            %convert to SE3 if needed
            if strcmp(self.parameterisation,'SE3')
                for i = 1:size(poses,2)
                    poses(:,i) = R3xso3_LogSE3(poses(:,i));
                end
            end
            
            %dataPoints
            self.dataPoints = [tFit; poses];
        end
    end
    
    % Plotting
    methods(Access = public)
        function plot(self,varargin)
            poses = self.dataPoints(2:7,:);
            if strcmp(self.parameterisation,'SE3')
                for i = 1:size(poses,2)
                    poses(:,i) = LogSE3_Rxt(poses(:,i));
                end
            end
            
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

