%--------------------------------------------------------------------------
% Author: Montiel Abello - montiel.abello@gmail.com - 23/05/17
% Contributors:
%--------------------------------------------------------------------------

classdef StaticPointTrajectory < PointTrajectory
    %StaticPointTrajectory represents a static point trajectory
    %   This class essentially wraps a GP_Point object with the methods of
    %   the PointTrajectory class, allowing it to be treated to same as
    %   dynamic trajectory objects
    
    %% 1. Properties
    properties(GetAccess = 'protected', SetAccess = 'protected')
    end
    
    properties(Hidden)
        GP_Point
    end
    
    %% 2. Methods    
    % Constructor
    methods(Access = public) %set to private later??
        function self = StaticPointTrajectory(point,varargin)
            switch nargin
                case 0 %allow preallocation
                otherwise
                    if ~strcmp(class(point),'GP_Point')
                        point = GP_Point(point,varargin{:});
                    end
                    self.GP_Point = point;
            end
            
        end
        
    end
    
    % Get & Set
    methods(Access = public)
        function value = getSwitch(self,property,varargin)
            if numel(varargin) > 0
                nPoints = numel(varargin{1});
            else
                nPoints = 1;
            end
            switch property
                case 'GP_Point'
                    value(nPoints) = GP_Point();
                    for i = 1:nPoints
                        value(i) = self.GP_Point.copy();
                    end
                case {'R3Position','S2xRPosition'}
                    value = self.GP_Point.get(property);
                    value = repmat(value,1,nPoints);
                case 'static'
                    value = 1;
                otherwise 
                    error('Error: invalid property')
            end
        end
        
        function self = setSwitch(self,property,value,varargin)
            switch property
                case 'GP_Point'
                    self.GP_Point = value;
                case {'R3Position'}
                    self.GP_Point.set(property,value);
                otherwise 
                    error('Error: invalid property')
            end
        end
    end
    
        % Plotting
    methods(Access = public)
        function plot(self,varargin)
            % could put for loop to accommodate object arrays - not sure if
            % useful or wise though...
            assert(numel(self)==1,'Error: This function not designed for object arrays')
            
            %store position for plotting
            position = self.GP_Point.get('R3Position');
            
            %plot positions
            plot3(position(1,:),position(2,:),position(3,:),'k.')

        end
    end
    
end

