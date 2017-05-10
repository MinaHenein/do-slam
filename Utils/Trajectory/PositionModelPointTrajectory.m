classdef PositionModelPointTrajectory < PointTrajectory
    %PositionModelPoseTrajectory represents a dynamic trajectory that is
    %instantiated from 3D position waypoints
    %   a position model is fitted to the waypoints and stored as a
    %   property.
    %   when the pose @ time t is requested through the get->getSwitch
    %   methods, the model is used to compute the position.
    
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
        function self = PositionModelPointTrajectory(waypoints,parameterisation,fitType)
            switch nargin
                case 0 %allow preallocation
                otherwise
                    assert(strcmp(parameterisation,'R3'),'Error: Only R3 waypoints implemented.')
                    %fit x,y,z to waypoints
                    self.xModel = fit(waypoints(1,:)',waypoints(2,:)',fitType);
                    self.yModel = fit(waypoints(1,:)',waypoints(3,:)',fitType);
                    self.zModel = fit(waypoints(1,:)',waypoints(4,:)',fitType);
            end
                    
        end
        
    end
    
    % Get & Set
    methods(Access = public)
        function value = getSwitch(self,property,varargin)
            switch property
                case 'GP_Point'
                    t = varargin{1};
                    point = computePoint(self,t);
                    value = point;
                case {'R3Position','S2xRPosition'}
                    t = varargin{1};
                    point = computePoint(self,t);
                    value = point.get(property);
                case 'static'
                    value = 0;
                otherwise 
                    error('Error: invalid property')
            end
        end
        
        function self = setSwitch(self,property,value,varargin)
            self.(property) = value;
        end
    end
    
    %use x,y,z models to compute position at times t
    methods(Access = private)
        function points = computePoint(self,t)
            nPoints = numel(t);
            position = [self.xModel(t)';
                        self.yModel(t)';
                        self.zModel(t)'];
            %output is GP_Point
            points(nPoints) = GP_Point();
            for i = 1:nPoints
                points(i) = GP_Point(position(:,i));  
            end
        end
    end
    
end

