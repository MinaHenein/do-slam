%--------------------------------------------------------------------------
% Author: Montiel Abello - montiel.abello@gmail.com - 23/05/17
% Contributors:
%--------------------------------------------------------------------------

classdef DiscretePoseTrajectory < PoseTrajectory
    %DISCRETEPOSETRAJECTORY is trajectory represented only by GP_Pose
    %waypoints
    %   This class will typically be used to represent trajectories
    %   generated from real data that cannot/should not be represented with
    %   a simplified model
    %   Only poses at times 't' can be requested with the 'get' method
    
    %% 1. Properties
    properties(Hidden)
        poses
        t
    end
    
    %% 2. Methods
    % Constructor
    methods(Access = public)
        function self = DiscretePoseTrajectory(dataPoints,t,varargin)
            switch nargin
                case 0 %allow preallocation
                otherwise
                    % assert t unique
                    assert(numel(t)==numel(unique(t)),'Error: t cannot contain duplicates')
                    nPoses = numel(t);
                    % convert poses to GP_Pose
                    if ~strcmp(class(dataPoints),'GP_Pose')
                        poses(nPoses) = GP_Pose();
                        if isempty(varargin)
                            for i = 1:nPoses
                                poses(i) = GP_Pose(dataPoints(:,i));
                            end
                        else %pass parameterisation to GP_Pose constructor
                            for i = 1:nPoses
                                poses(i) = GP_Pose(dataPoints(:,i),varargin{:});
                            end
                        end
                    % dataPoints already GP_Poses    
                    else
                        poses = dataPoints;
                    end
                    
                    % check sizes
                    assert(numel(poses)==numel(t),'Error: number of dataPoints and times must be equal')
                    
                    % set properties
                    self.poses = poses;
                    self.t     = t;
            end
                    
        end
        
    end
    
    % Get & Set
    methods(Access = public)
        function value = getSwitch(self,property,varargin)
            switch property
                case 'GP_Pose'
                    t     = varargin{1};
                    value = self.getPoses(t);
                case {'R3xso3Pose','logSE3Pose','R3xso3Position','logSE3Position','axisAngle','R'}
                    t     = varargin{1};
                    poses = self.getPoses(t);
                    value = poses.get(property);
                case 'poses'
                    value = self.poses;
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
                    nPoses = numel(t);
                    for i = 1:nPoses
                        iLogical = (self.t==t(i));
                        self.poses(iLogical) = value(i);
                    end
                case {'R3xso3Pose','logSE3Pose','R3xso3Position','logSE3Position','axisAngle'}
                    t = varargin{1};
                    nPoses = numel(t);
                    for i = 1:nPoses
                        iLogical = (self.t==t(i));
                        self.poses(iLogical) = self.poses(iLogical).set(property,value(:,i));
                    end
                case {'R'}
                    t = varargin{1};
                    nPoses = numel(t);
                    for i = 1:nPoses
                        iLogical = (self.t==t(i));
                        self.poses(iLogical) = self.poses(iLogical).set(property,value(:,mapping(i,3)));
                    end
            end
        end
        
    end
    
    % Get poses
    methods(Access = private)
        function poses = getPoses(self,t)
            assert(~any(~ismember(t,self.t)),'Error: can only request poses for times in self.t')
            nPoses = numel(t);
            poses(nPoses) = GP_Pose();
            for i = 1:nPoses
                iLogical = (self.t==t(i));
                poses(i) = self.poses(iLogical);
            end
        end
    end
    
    % Add/delete datapoints
    methods(Access = public)
        function self = addPoses(self,dataPoints,t,varargin)
            % add dataPoints to poses
            nPoses = numel(t);
            assert(nPoses==numel(unique(t)),'Error: t cannot contain duplicates')
            assert(~any(ismember(t,self.t)),'Error: cannot redefine dataPoints with this method')
            assert(nPoses==numel(unique(t)),'Error: t cannot contain duplicates')
            
            % convert poses to GP_Pose
            if ~strcmp(class(dataPoints),'GP_Pose')
                poses(nPoses) = GP_Pose();
                if isempty(varargin)
                    for i = 1:nPoses
                        poses(i) = GP_Pose(dataPoints(:,i));
                    end
                else %pass parameterisation to GP_Pose constructor
                    for i = 1:nPoses
                        poses(i) = GP_Pose(dataPoints(:,i),varargin{:});
                    end
                end
            % dataPoints already GP_Poses    
            else
                poses = dataPoints;
            end

            % check sizes
            assert(numel(poses)==numel(t),'Error: number of dataPoints and times must be equal')
            
            % add poses,t to self
            self.poses = [self.poses poses];
            self.t     = [self.t t];
            
            % reorder
            [tSorted,order] = sort(self.t);
            self.poses = self.poses(order);
            self.t     = tSorted;
        end
        
        function self = deletePoses(self,t)
            %delete dataPoints with times in t
            nPoses = numel(t);
            assert(nPoses==numel(unique(t)),'Error: t cannot contain duplicates')
            assert(~any(~ismember(t,self.t)),'Error: can only delete poses for times in self.t')
            keepLogical = ~ismember(self.t,t);
            self.poses = self.poses(keepLogical);
            self.t     = self.t(keepLogical);
        end
    end
    
end

