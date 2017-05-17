classdef GP_Point < GeometricPrimitive
    %GP_POINT represents a position in 3D space
    %   R3 & S2xR point parameterisations implemented
    %   transformation methods allow points to be converted between
    %   absolute/relative position given a GP_Pose input
    
    %% 1. Properties
    properties(GetAccess = 'private', SetAccess = 'private')
        R3Position
    end
    
        %% 2. Methods
    % Constructor
    methods(Access = public)
        function self = GP_Point(position,varargin)
            switch nargin
                case 0
                case 1
                    self.R3Position = position;
                otherwise
                    %check parameterisation
                    assert(strcmp(varargin{1},'R3'),'Error: only R3 parameterisation implemented')
                    self.R3Position = position;
            end
%             assert(isequal([3,1],size(self.R3Position)),'Error: position must be 3x1')
        end
    end
    
    % Get & Set
    methods(Access = public)
        % Can get values that are not properties
        % Compute from R3Position property
        function value = getSwitch(self,property,varargin)
            switch property
                case 'R3Position'
                    value = self.R3Position;
                case 'S2xRPosition'
                    value = R3_S2xR(self.R3Position);
                otherwise 
                    error('Error: invalid property')
            end
        end
        
        % Can set values that are not properties
        % Compute R3xso3Pose property
        function self = setSwitch(self,property,value,varargin)
            switch property
                case 'R3Position'
                    self.R3Position = value;
                case 'S2xRPosition'
                    self.R3Position = S2xR_R3(value);
                otherwise 
                    error('Error: invalid property')
            end
        end
    end
    
    % Transformations
    methods(Access = public)
        function positionRelative = AbsoluteToRelativePoint(self,poseReference,varargin)
            %preallocate
            positionRelative(numel(self)) = GP_Point;
            % Default - transformations use R3xso3 pose parameterisation
            if isempty(varargin) || strcmp(varargin{1},'R3xso3')
                %indexing depends on size of input and output
                if (numel(self)==numel(poseReference))
                    for i = 1:numel(self)
                        positionRelative(i) = GP_Point();
                        positionRelative(i).set('R3Position',AbsoluteToRelativePositionR3xso3(poseReference(i).get('R3xso3Pose'),self(i).get('R3Position')));
                    end
                elseif (numel(self)>1) && ((numel(poseReference)==1))
                    for i = 1:numel(self)
                        positionRelative(i) = GP_Point();
                        positionRelative(i).set('R3Position',AbsoluteToRelativePositionR3xso3(poseReference.get('R3xso3Pose'),self(i).get('R3Position')));
                    end
                elseif (numel(self)==1) && ((numel(poseReference)>1))
                    positionRelative(numel(poseReference)) = GP_Point;
                    for i = 1:numel(self)
                        positionRelative(i) = GP_Point();
                        positionRelative(i).set('R3Position',AbsoluteToRelativePositionR3xso3(poseReference(i).get('R3xso3Pose'),self.get('R3Position')));
                    end
                else
                    error('Error: inconsistent sizes')
                end
            % if logSE3 pose parameterisation desired
            elseif strcmp(varargin{1},'logSE3')
                %indexing depends on size of input and output
                if (numel(self)==numel(poseReference))
                    for i = 1:numel(self)
                        positionRelative(i) = GP_Point();
                        positionRelative(i).set('R3Position',AbsolutePoint2RelativePoint3D(poseReference(i).get('logSE3Pose'),self(i).get('R3Position')));
                    end
                elseif (numel(self)>1) && ((numel(poseReference)==1))
                    for i = 1:numel(self)
                        positionRelative(i) = GP_Point();
                        positionRelative(i).set('R3Position',AbsolutePoint2RelativePoint3D(poseReference.get('logSE3Pose'),self(i).get('R3Position')));
                    end
                elseif (numel(self)==1) && ((numel(poseReference)>1))
                    positionRelative(numel(poseReference)) = GP_Point;
                    for i = 1:numel(self)
                        positionRelative(i) = GP_Point();
                        positionRelative(i).set('R3Position',AbsolutePoint2RelativePoint3D(poseReference(i).get('logSE3Pose'),self.get('R3Position')));
                    end
                else
                    error('Error: inconsistent sizes')
                end
            else
                error('Error: invalid pose parameterisation')
            end
                
        end
        
        function positionAbsolute = RelativeToAbsolutePoint(self,poseReference,varargin)
            %preallocate
            positionAbsolute(numel(self)) = GP_Point;
            % Default - transformations use R3xso3 pose parameterisation
            if isempty(varargin) || strcmp(varargin{1},'R3xso3')
                %indexing depends on size of input and output
                if (numel(self)==numel(poseReference))
                    for i = 1:numel(self)
                        positionAbsolute(i) = GP_Point();
                        positionAbsolute(i).set('R3Position',RelativeToAbsolutePositionR3xso3(poseReference(i).get('R3xso3Pose'),self(i).get('R3Position')));
                    end
                elseif (numel(self)>1) && ((numel(poseReference)==1))
                    for i = 1:numel(self)
                        positionAbsolute(i) = GP_Point();
                        positionAbsolute(i).set('R3Position',RelativeToAbsolutePositionR3xso3(poseReference.get('R3xso3Pose'),self(i).get('R3Position')));
                    end
                elseif (numel(self)==1) && ((numel(poseReference)>1))
                    positionAbsolute(numel(poseReference)) = GP_Point;
                    for i = 1:numel(self)
                        positionAbsolute(i) = GP_Point();
                        positionAbsolute(i).set('R3Position',RelativeToAbsolutePositionR3xso3(poseReference(i).get('R3xso3Pose'),self.get('R3Position')));
                    end
                else
                    error('Error: inconsistent sizes')
                end
            % if logSE3 pose parameterisation desired
            elseif strcmp(varargin{1},'logSE3')
                %indexing depends on size of input and output
                if (numel(self)==numel(poseReference))
                    for i = 1:numel(self)
                        positionAbsolute(i) = GP_Point();
                        positionAbsolute(i).set('R3Position',RelativePointToAbsolutePoint3D(poseReference(i).get('logSE3Pose'),self(i).get('R3Position')));
                    end
                elseif (numel(self)>1) && ((numel(poseReference)==1))
                    for i = 1:numel(self)
                        positionAbsolute(i) = GP_Point();
                        positionAbsolute(i).set('R3Position',RelativePointToAbsolutePoint3D(poseReference.get('logSE3Pose'),self(i).get('R3Position')));
                    end
                elseif (numel(self)==1) && ((numel(poseReference)>1))
                    positionAbsolute(numel(poseReference)) = GP_Point;
                    for i = 1:numel(self)
                        positionAbsolute(i) = GP_Point();
                        positionAbsolute(i).set('R3Position',RelativePointToAbsolutePoint3D(poseReference(i).get('logSE3Pose'),self.get('R3Position')));
                    end
                else
                    error('Error: inconsistent sizes')
                end
            else
                error('Error: invalid pose parameterisation')
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
                    poseNoisy = GP_Point(self.get('R3Position') + noise);
                case 'Off'
                    poseNoisy = self;
                otherwise
                    error('Error: invalid noise model')
            end
        end
    end
end

