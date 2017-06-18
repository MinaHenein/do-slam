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
        meshPoints % points representing the mesh
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
                    objectPose = self.get('GP_Pose',t).get('R3xso3Pose');
                    points = self.meshPoints;
                    for p=1:size(points,2)
                        points(:,p) = RelativeToAbsolutePositionR3xso3(objectPose,points(:,p));
                    end
                    out = points;                    
                case 'meshRelative'
                    points = self.meshPoints;
                    links = self.meshLinks;
                    out = [points(:,links(:,1))' points(:,links(:,2))' points(:,links(:,3))'];
                case 'meshAbsolute'
                    t = varargin{1};
                    points = self.get('meshPointsAbsolute',t);
                    links = self.meshLinks;
                    out = [points(:,links(:,1))' points(:,links(:,2))' points(:,links(:,3))'];
                otherwise
                    out = self.(property);
            end
        	
        end
        
        function self = setSwitch(self,property,value,varargin)
        	self.(property) = value;
        end
        
    end
    
end

