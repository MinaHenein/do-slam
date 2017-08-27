%--------------------------------------------------------------------------
% Author: Montiel Abello - montiel.abello@gmail.com - 23/05/17
% Contributors: Yash Vyas - yjvyas@gmail.com - 27/08/17
%--------------------------------------------------------------------------

classdef GP_Pose < GeometricPrimitive
    %GP_POSE represents 6D pose geometry
    %   R3xso3 pose parameterisation is stored
    %   When other parameterisations/properties are requested, they are 
    %   computed from the stored R3xso3 pose value
    %   When other parameterisations/properties are set, the new R3xso3
    %   pose is computed and set
    
    %% 1. Properties
    properties(GetAccess = 'private', SetAccess = 'private')
        R3xso3Pose
    end
    
    %% 2. Methods
    % Constructor
    methods(Access = public)
        function self = GP_Pose(pose,varargin)
            switch nargin
                case 0
                case 1
                    self.R3xso3Pose = pose;
                otherwise
                    switch varargin{1}
                        case 'R3xso3'
                            self.R3xso3Pose = pose;
                        case 'logSE3'
                            self.R3xso3Pose = LogSE3_Rxt(pose);
                        otherwise
                            error('Error: invalid parameterisation')
                    end
            end
%             assert(isequal([6,1],size(self.R3xso3Pose)),'Error: pose must be 6x1')
        end
    end
    
    % Get & Set
    methods(Access = public)
        % Can get values that are not properties
        % Compute from R3xso3Pose property
        function value = getSwitch(self,property,varargin)
            switch property
                case 'R3xso3Pose'
                    value = self.R3xso3Pose;
                case 'logSE3Pose'
                    value = R3xso3_LogSE3(self.R3xso3Pose);
                case 'expSE3Pose'
                    logSE3Pose = R3xso3_LogSE3(self.R3xso3Pose);
                    value = ExpSE3(logSE3Pose);
                case 'R3xso3Position'
                    value = self.R3xso3Pose(1:3);
                case 'logSE3Position'
                    value = R3xso3_LogSE3(self.R3xso3Pose);
                    value = value(1:3);
                case 'axisAngle'
                    value = self.R3xso3Pose(4:6);
                case 'R'
                    value = rot(self.R3xso3Pose(4:6));
                otherwise 
                    error('Error: invalid property')
            end
        end
        
        % Can set values that are not properties
        % Compute R3xso3Pose property
        function self = setSwitch(self,property,value,varargin)
            switch property
                case {'pose','R3xso3Pose'}
                    self.R3xso3Pose = value;
                case 'logSE3Pose'
                    self.R3xso3Pose = LogSE3_Rxt(value);
                case 'R3xso3Position'
                    self.R3xso3Pose = [value; self.R3xso3Pose(4:6)];
                case 'logSE3Position'
                    logSE3Pose = self.getSwitch('logSE3Pose');
                    logSE3Pose(1:3) = value;
                    self.R3xso3Pose = logSE3_Rxt(logSE3Pose);
                case 'axisAngle'
                    self.R3xso3Pose = [self.R3xso3Pose(1:3); value];
                case 'R'
                    self.R3xso3Pose = [self.R3xso3Pose(1:3); arot(value)];
                otherwise 
                    error('Error: invalid property')
            end
        end
    end
    
    % Transformations
    methods(Access = public)
        function poseRelative = AbsoluteToRelativePose(self,poseReference)
            %preallocate
            poseRelative(numel(self)) = GP_Pose;
            %indexing depends on size of input and output
            if (numel(self)==numel(poseReference))
                for i = 1:numel(self)
                    poseRelative(i) = GP_Pose();
                    poseRelative(i).set('R3xso3Pose',AbsoluteToRelativePoseR3xso3(poseReference(i).get('R3xso3Pose'),self(i).get('R3xso3Pose')));
                end
            elseif (numel(self)>1) && ((numel(poseReference)==1))
                for i = 1:numel(self)
                    poseRelative(i) = GP_Pose();
                    poseRelative(i).set('R3xso3Pose',AbsoluteToRelativePoseR3xso3(poseReference.get('R3xso3Pose'),self(i).get('R3xso3Pose')));
                end
            elseif (numel(self)==1) && ((numel(poseReference)>1))
                poseRelative(numel(poseReference)) = GP_Pose();
                for i = 1:numel(self)
                    poseRelative(i) = GP_Pose();
                    poseRelative(i).set('R3xso3Pose',AbsoluteToRelativePoseR3xso3(poseReference(i).get('R3xso3Pose'),self.get('R3xso3Pose')));
                end
            else
                error('Error: inconsistent sizes')
            end
        end
        
        function poseAbsolute = RelativeToAbsolutePose(self,poseReference)
            %preallocate
            poseAbsolute(numel(self)) = GP_Pose;
            %indexing depends on size of input and output
            if (numel(self)==numel(poseReference))
                for i = 1:numel(self)
                    poseAbsolute(i) = GP_Pose();
                    poseAbsolute(i).set('R3xso3Pose',RelativeToAbsolutePoseR3xso3(poseReference(i).get('R3xso3Pose'),self(i).get('R3xso3Pose')));
                end
            elseif (numel(self)>1) && ((numel(poseReference)==1))
                for i = 1:numel(self)
                    poseAbsolute(i) = GP_Pose();
                    poseAbsolute(i).set('R3xso3Pose',RelativeToAbsolutePoseR3xso3(poseReference.get('R3xso3Pose'),self(i).get('R3xso3Pose')));
                end
            elseif (numel(self)==1) && ((numel(poseReference)>1))
                poseAbsolute(numel(poseReference)) = GP_Pose();
                for i = 1:numel(poseAbsolute)
                    poseAbsolute(i) = GP_Pose();
                    poseAbsolute(i).set('R3xso3Pose',RelativeToAbsolutePoseR3xso3(poseReference(i).get('R3xso3Pose'),self.get('R3xso3Pose')));
                end
            else
                error('Error: inconsistent sizes')
            end
        end
    end
    
    % Add Noise
    methods(Access = public)
        function poseNoisy = addNoise(self,noiseModel,varargin)
            switch noiseModel
                case 'Gaussian'
                    mean   = varargin{1};
                    stdDev = varargin{2};
                    noise = normrnd(mean,stdDev);
                    poseNoisy = self.RelativeToAbsolutePose(GP_Pose(noise));
                case 'Off'
                    poseNoisy = self;
                otherwise
                    error('Error: invalid noise model')
            end
        end
    end
end

