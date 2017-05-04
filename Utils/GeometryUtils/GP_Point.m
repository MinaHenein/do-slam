classdef GP_Point < GeometricPrimitive
    %GP_POINT represents a position in 3D space
    %   currently only R3 position implemented
    
    %% 1. Properties
    properties(GetAccess = 'private', SetAccess = 'private')
        R3Position
    end
    
    %% 2. Methods
    % Getter & Setter
    methods(Access = public)
        function out = get(self,property,varargin)
            if (nargin==2)
                out = [self.(property)];
            elseif (nargin==3) %specific location required
                out = [self(varargin{1}).(property)];
            end
            
        end
        
        function self = set(self,property,values,varargin)
            if (nargin==3)
                self.(property) = values;
            elseif (nargin==4) %set specific locations
                locations = varargin{1};
                for i = 1:numel(locations)
                    self(locations(i)).(property) = values(:,i);
                end
            end
            
        end
    end
    
    % Constructor
    methods(Access = public)
        function self = GP_Point(position,varargin)
            switch nargin
                case 0
                case 1
                    self.set('R3Position',position);
                otherwise
                	assert(strcmp('R3',varargin{1}),'Error: only R3 position parameterisation implemented.')
            end
        end
    end
    
    % Transformations
    methods(Access = public)
        function positionRelative = AbsoluteToRelativePoint(self,poseReference)
            positionRelative(numel(self)) = GP_Point;
            if (numel(self)==numel(poseReference))
                for i = 1:numel(self)
                    positionRelative(i).set('R3Position',AbsoluteToRelativePositionR3xso3(poseReference(i).get('R3xso3Pose'),self(i).get('R3Position')));
                end
            elseif (numel(self)>1) && ((numel(poseReference)==1))
                for i = 1:numel(self)
                    positionRelative(i).set('R3Position',AbsoluteToRelativePositionR3xso3(poseReference.get('R3xso3Pose'),self(i).get('R3Position')));
                end
            elseif (numel(self)==1) && ((numel(poseReference)>1))
                positionRelative(numel(poseReference)) = GP_Point;
                for i = 1:numel(self)
                    positionRelative(i).set('R3Position',AbsoluteToRelativePositionR3xso3(poseReference(i).get('R3xso3Pose'),self.get('R3Position')));
                end
            else
                error('Error: inconsistent sizes')
            end
        end
        
        function positionAbsolute = RelativeToAbsolutePoint(self,poseReference)
            positionAbsolute(numel(self)) = GP_Point;
            if (numel(self)==numel(poseReference))
                for i = 1:numel(self)
                    positionAbsolute(i).set('R3Position',RelativeToAbsolutePositionR3xso3(poseReference(i).get('R3xso3Pose'),self(i).get('R3Position')));
                end
            elseif (numel(self)>1) && ((numel(poseReference)==1))
                for i = 1:numel(self)
                    positionAbsolute(i).set('R3Position',RelativeToAbsolutePositionR3xso3(poseReference.get('R3xso3Pose'),self(i).get('R3Position')));
                end
            elseif (numel(self)==1) && ((numel(poseReference)>1))
                positionAbsolute(numel(poseReference)) = GP_Point;
                for i = 1:numel(self)
                    positionAbsolute(i).set('R3Position',RelativeToAbsolutePositionR3xso3(poseReference(i).get('R3xso3Pose'),self.get('R3Position')));
                end
            else
                error('Error: inconsistent sizes')
            end
        end
    end
    
end

