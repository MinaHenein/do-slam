classdef DiscretePointTrajectory < PointTrajectory
    %DISCRETEPOSETRAJECTORY is trajectory represented only by GP_Point
    %waypoints
    %   This class will typically be used to represent trajectories
    %   generated from real data that cannot/should not be represented with
    %   a simplified model
    %   Only points at times 't' can be requested with the 'get' method
    
    %% 1. Properties
    properties(Hidden)
        points
        t
    end
    
    %% 2. Methods
    % Constructor
    methods(Access = public)
        function self = DiscretePointTrajectory(dataPoints,t,varargin)
            switch nargin
                case 0 %allow preallocation
                otherwise
                    % assert t unique
                    assert(numel(t)==numel(unique(t)),'Error: t cannot contain duplicates')
                    nPoints = numel(t);
                    % convert poses to GP_Point
                    if ~strcmp(class(dataPoints),'GP_Point')
                        points(nPoints) = GP_Point();
                        if isempty(varargin)
                            for i = 1:nPoints
                                points(i) = GP_Point(dataPoints(:,i));
                            end
                        else %pass parameterisation to GP_Point constructor
                            for i = 1:nPoints
                                points(i) = GP_Point(dataPoints(:,i),varargin{:});
                            end
                        end
                    % dataPoints already GP_Points    
                    else
                        points = dataPoints;
                    end
                    
                    % check sizes
                    assert(numel(points)==numel(t),'Error: number of dataPoints and times must be equal')
                    
                    % set properties
                    self.points = points;
                    self.t     = t;
            end
                    
        end
        
    end
    
    % Get & Set
    methods(Access = public)
        function value = getSwitch(self,property,varargin)
            switch property
                case 'GP_Point'
                    t     = varargin{1};
                    value = self.getPoints(t);
                case {'R3Position','S2xRPosition'}
                    t      = varargin{1};
                    points = self.getPoints(t);
                    value  = points.get(property);
                case 'points'
                    value = self.points;
                case 't'
                    value = self.t;
                case 'static'
                    value = 0;
                otherwise 
                    error('Error: invalid property')
            end
        end
        
        function self = setSwitch(self,property,value,varargin)
            switch property
                case 'GP_Pose'
                    t = varargin{1};
                    nPoints = numel(t);
                    for i = 1:nPoints
                        iLogical = (self.t==t(i));
                        self.points(iLogical) = value(i);
                    end
                case{'R3Position','S2xRPosition'}
                    t = varargin{1};
                    nPoints = numel(t);
                    for i = 1:nPoints
                        iLogical = (self.t==t(i));
                        self.points(iLogical) = self.points(iLogical).set(property,value(:,i));
                    end
            end
        end
        
    end
    
    % Get points
    methods(Access = private)
        function points = getPoints(self,t)
            assert(~any(~ismember(t,self.t)),'Error: can only request points for times in self.t')
            nPoints = numel(t);
            points(nPoints) = GP_Point();
            for i = 1:nPoints
                iLogical = (self.t==t(i));
                points(i) = self.points(iLogical);
            end
        end
    end
    
    % Add/delete dataPoints
    methods(Access = public)
        function self = addPoses(self,dataPoints,t,varargin)
            % add dataPoints to points
            nPoints = numel(t);
            assert(nPoints==numel(unique(t)),'Error: t cannot contain duplicates')
            assert(~any(ismember(t,self.t)),'Error: cannot redefine dataPoints with this method')
            assert(nPoints==numel(unique(t)),'Error: t cannot contain duplicates')
            
            % convert points to GP_Points
            if ~strcmp(class(dataPoints),'GP_Pose')
                points(nPoints) = GP_Point();
                if isempty(varargin)
                    for i = 1:nPoints
                        points(i) = GP_Point(dataPoints(:,i));
                    end
                else %pass parameterisation to GP_Pose constructor
                    for i = 1:nPoints
                        points(i) = GP_Point(dataPoints(:,i),varargin{:});
                    end
                end
            % dataPoints already GP_Poses    
            else
                points = dataPoints;
            end

            % check sizes
            assert(numel(points)==numel(t),'Error: number of dataPoints and times must be equal')
            
            % add points,t to self
            self.points = [self.points points];
            self.t     = [self.t t];
            
            % reorder
            [tSorted,order] = sort(self.t);
            self.points = self.points(order);
            self.t      = tSorted;
        end
        
        function self = deletePoints(self,t)
            %delete dataPoints with times in t
            nPoints = numel(t);
            assert(nPoints==numel(unique(t)),'Error: t cannot contain duplicates')
            assert(~any(~ismember(t,self.t)),'Error: can only delete points for times in self.t')
            keepLogical = ~ismember(self.t,t);
            self.points = self.points(keepLogical);
            self.t     = self.t(keepLogical);
        end
    end
    
end

