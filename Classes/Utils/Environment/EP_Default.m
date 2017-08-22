%--------------------------------------------------------------------------
% Author: Montiel Abello - montiel.abello@gmail.com - 23/05/17 
%         Yash Vyas - yjvyas@gmail.com - 02/06/17
% Contributors: 
%--------------------------------------------------------------------------

classdef EP_Default < EnvironmentPrimitive
    %EP_DEFAULT represents any geometric primitive formed from a set of
    %relative points
    %   EP_Default will likely be used to construct a RigidBodyObject
    
    %% 1. Properties
    properties(GetAccess = 'protected', SetAccess = 'protected')
        meshPoints % points representing the mesh, these are GP_Point
        meshLinks % set of triangle links between meshPoints representing the triangulation
    end
    
    %% 2. Methods   
    % constructor
    methods(Access = public)
        function self = EP_Default()
        end
    end
    
    % Get & Set
    methods(Access = public)
        function out = getSwitch(self,property,varargin)
            switch property
                case {'GP_Pose','R3xso3Pose','logSE3Pose','R3xso3Position','logSE3Position','axisAngle','R'}
                    out = self.trajectory.get(property,varargin{1});
                case 'static'
                    out = self.trajectory.get(property);
                case 'meshPointsAbsolute'
                    t = varargin{1};
                    objectPose = self.get('GP_Pose',t);
                    pointsAbsolute = self.meshPoints.RelativeToAbsolutePoint(objectPose);
                    out = pointsAbsolute;
                case 'meshRelative'
                    pointsRelative = self.meshPoints.get('R3Position');
                    links = self.meshLinks;
                    out = [pointsRelative(:,links(:,1))' pointsRelative(:,links(:,2))' pointsRelative(:,links(:,3))'];
                case 'meshAbsolute'
                    t = varargin{1};
                    pointsAbsolute = self.get('meshPointsAbsolute',t).get('R3Position');
                    links = self.meshLinks;
                    out = [pointsAbsolute(:,links(:,1))' pointsAbsolute(:,links(:,2))' pointsAbsolute(:,links(:,3))'];
                otherwise
                    out = self.(property);
            end
        	
        end
        
        function self = setSwitch(self,property,value,varargin)
        	self.(property) = value;
        end
        
    end
    
end

