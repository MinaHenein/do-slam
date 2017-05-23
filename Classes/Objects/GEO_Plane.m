%--------------------------------------------------------------------------
% Author: Montiel Abello - montiel.abello@gmail.com - 23/05/17
% Contributors:
%--------------------------------------------------------------------------

classdef GEO_Plane < GeometricEntityObject
    %GEO_PLANE represents a geometric plane parameterised by normal
    %(perpendicular to origin) and distance (distance to origin)
    %   GEO_Plane is formed from an EP_Rectangle
    %   trajectory of the rectangle is stored
    %   When parameters @ t are requested, normal and distance are computed
    %   from the rectangleTrajectory
    %   Parameters are constrained so the distance >= 0 and normal is an
    %   element of S2
    
    %% 1. Properties
    properties(GetAccess = 'protected', SetAccess = 'protected')
        rectangleTrajectory
    end
    
    
    %% 2. Methods   
    % Constructor
    methods(Access = public)
        function self = GEO_Plane(rectangle)
            switch nargin
                case 0
                otherwise
                    self.index        = rectangle.get('index');
                    self.pointIndexes = rectangle.get('pointIndexes');
                    self.rectangleTrajectory = rectangle.get('trajectory');                      
            end
        end
        
    end
    
    % Getter & Setter
    methods(Access = public) %set to protected later??
        function out = getSwitch(self,property,varargin)
            switch property 
                case 'parameters'
                    t = varargin{1};
                    out = self.computePlaneParameters(t);
                case 'normal'
                    t = varargin{1};
                    parameters = self.computePlaneParameters(t);
                    out = parameters(1:3,:);
                case 'distance'
                    t = varargin{1};
                    parameters = self.computePlaneParameters(t);
                    out = parameters(4,:);
                otherwise
                    out = self.(property);
            end
        end
        
        function self = setSwitch(self,property,value)
        	self.(property) = value;
        end
    end
    
    % Compute parameters from pose
    methods(Access = public)
        function [parameters] = computePlaneParameters(self,t)
            pose = self.rectangleTrajectory.get('GP_Pose',t);
            position = pose.get('R3xso3Position');
            R = pose.get('R');
            %compute parameters
            normal = R(:,3:3:end);
            distance = dot(position,normal);
            %constrain distance >= 0
            negativeLogical = (distance <= 0);
            normal(:,negativeLogical) = -normal(:,negativeLogical);
            distance(negativeLogical) = -distance(negativeLogical);
            parameters = [normal; distance];
        end
    end
    
end

