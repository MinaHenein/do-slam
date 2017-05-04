classdef GP_Pose < GeometricPrimitive
    %GP_POSE Summary of this class goes here
    %   Detailed explanation goes here
    %   *Dependent properties will be recomputed whenever R3xso3Pose changes
    %   and whenever they are accessed, they are computed from R3xso3Pose
    %   - this might be slow if R3xso3Pose changes is changed a lot
    %   - options: remove dependent properties if they are not needed a
    %   lot, OR remove dependencies and build this functionality into the
    %   set method manually - whenever any property is changed, change the
    %   rest
    
    %% 1. Properties
    properties(GetAccess = 'public', SetAccess = 'public')
        R3xso3Pose
    end
    
    properties(Dependent)
        logSE3Pose
        R3xso3Position
        logSE3Position
        axisAngle
        R
    end
    
    %% 2. Methods
    % Dependent property assignment - this might be slow *
    methods
        function logSE3Pose = get.logSE3Pose(self)
            logSE3Pose = R3xso3_LogSE3(self.R3xso3Pose);
        end
        
        function logSE3Position = get.logSE3Position(self)
            logSE3Position = self.logSE3Pose(1:3);
        end
        
        function R3xso3Position = get.R3xso3Position(self)
            R3xso3Position = self.R3xso3Pose(1:3);
        end
        
        function axisAngle = get.axisAngle(self)
            axisAngle = self.R3xso3Pose(4:6);
        end
        
        function R = get.R(self)
            R = rot(self.R3xso3Pose(4:6));
        end
    end
        
    % Getter & Setter
    methods(Access = public)
        function out = get(self,property,varargin)
            if (nargin==2)
                out = [self.(property)];
            elseif (nargin==3) %specific poses
                out = [self(varargin{1}).(property)];
            end
            
        end
        
        function self = set(self,property,values,varargin)
            if (nargin==3)
                switch property
                    case 'R3xso3Pose'
                        self.R3xso3Pose = values;
                    case 'logSE3Pose'
                        self.R3xso3Pose = LogSE3_Rxt(values);
                    case 'R3xso3Position'
                        self.R3xso3Pose(1:3) = values;
                    case 'logSE3Position'
                        self.R3xso3Pose = LogSE3_Rxt([values; self.logSE3Pose(4:6)]);
                    case 'axisAngle'
                        self.R3xso3Pose(4:6) = values;
                    case 'R'
                        self.R3xso3Pose(4:6) = arot(values);
                end
            elseif (nargin==4)
                locations = varargin{1};
                for i = 1:numel(locations)
                    switch property
                        case 'R3xso3Pose'
                            self(locations(i)).R3xso3Pose = values(:,i);
                        case 'logSE3Pose'
                            self(locations(i)).R3xso3Pose = LogSE3_Rxt(values(:,i));
                        case 'R3xso3Position'
                            self(locations(i)).R3xso3Pose(1:3) = values(:,i);
                        case 'logSE3Position'
                            self(locations(i)).R3xso3Pose = LogSE3_Rxt([values(:,i); self(locations(i)).logSE3Pose(4:6)]);
                        case 'axisAngle'
                            self(locations(i)).R3xso3Pose(4:6) = values(:,i);
                        case 'R'
                            self(locations(i)).R3xso3Pose(4:6) = arot(values(:,mapping(i,3)));
                    end
                end
            end
            
        end
    end
    
    % Transformations
    methods(Access = public)
        function poseRelative = AbsoluteToRelativePose(self,poseReference)
            poseRelative(numel(self)) = GP_Pose;
            if (numel(self)==numel(poseReference))
                for i = 1:numel(self)
                    poseRelative(i).set('R3xso3Pose',AbsoluteToRelativePoseR3xso3(self(i).get('R3xso3Pose'),poseReference(i).get('R3xso3Pose')));
                end
            elseif (numel(self)>1) && ((numel(poseReference)==1))
                for i = 1:numel(self)
                    poseRelative(i).set('R3xso3Pose',AbsoluteToRelativePoseR3xso3(self(i).get('R3xso3Pose'),poseReference.get('R3xso3Pose')));
                end
            else
                error('Error: inconsistent sizes')
            end
        end
        
        function poseAbsolute = RelativeToAbsolutePose(self,poseReference)
            poseAbsolute(numel(self)) = GP_Pose;
            if (numel(self)==numel(poseReference))
                for i = 1:numel(self)
                    poseAbsolute(i).set('R3xso3Pose',RelativeToAbsolutePoseR3xso3(poseReference(i).get('R3xso3Pose'),self(i).get('R3xso3Pose')));
                end
            elseif (numel(self)>1) && ((numel(poseReference)==1))
                for i = 1:numel(self)
                    poseAbsolute(i).set('R3xso3Pose',RelativeToAbsolutePoseR3xso3(poseReference.get('R3xso3Pose'),self(i).get('R3xso3Pose')));
                end
            else
                error('Error: inconsistent sizes')
            end
        end
    end
    
end

