classdef PointTrajectory < Trajectory
    %PoseTrajectory Summary of this class goes here
    %   Detailed explanation goes here
    
    %% 1. Properties
    properties(GetAccess = 'protected', SetAccess = 'protected')
        points
    end
    
    %% 2. Methods
    % Getter & Setter
    methods(Access = public) %set to protected later??
        function out = get(self,property,varargin)
            if (nargin==2)
                out = [self.(property)];
            elseif (nargin==3) && strcmp(property,'poses')
                out = [self.points(varargin{1})];
            end
            
        end
        
        function self = set(self,property,value)
        	self.(property) = value;
        end
    end
    
    % Constructor
    methods(Access = public) %set to private later??
        function self = PointTrajectory(mode,varargin)
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
                            assert(strcmp(parameterisation,'R3'),'Error: Only R3 positions implemented.')
                            dataPoints = varargin{2};
                            self.t = dataPoints(1,:);
                            nPositions = numel(self.t);
                            GPPoints(nPositions) = GP_Point;
                            GPPoints.set(strcat(parameterisation,'Position'),dataPoints(2:4,:),[1:nPositions]);
                            self.points = GPPoints;
                        case 'continuous'
                            self.model = varargin{1};
                    end
            end
        end
        
    end
    
    % Fitting
    methods(Access = private)
        function self = fitTrajectory(self,waypoints,tFit,fitType)
            nPositions = numel(tFit);
            
            %models
            fX = fit(waypoints(1,:)',waypoints(2,:)',fitType);
            fY = fit(waypoints(1,:)',waypoints(3,:)',fitType);
            fZ = fit(waypoints(1,:)',waypoints(4,:)',fitType);
            
            %use model to get positions
            positions = zeros(3,numel(tFit));
            positions(1,:) = fX(tFit)';
            positions(2,:) = fY(tFit)';
            positions(3,:) = fZ(tFit)';

            %dataPoints
            self.t = tFit;
            GPPositions(nPositions) = GP_Point;
            GPPositions.set('R3Position',positions,[1:nPositions]);
            self.points = GPPositions;
        end
    end
    
    % Plotting
    methods(Access = public)
        function plot(self,varargin)
            positions = self.points.get('R3Position');
            
            %plot positions
            plot3(positions(1,:),positions(2,:),positions(3,:),'k.')
            
        end
    end
    
end

