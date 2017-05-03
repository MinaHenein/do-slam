classdef Trajectory < handle
    %TRAJECTORY class represents trajectories of moving/stationary
    %components (robots, objects, points )
    %   Both dynamic and static objects will use trajectory class. Only
    %   difference is model used.
    
    %% 1. Properties
    properties(GetAccess = 'public', SetAccess = 'public')
        parameterisation %string
    end
    properties(GetAccess = 'protected', SetAccess = 'protected')
        dataPoints %mxn array of dataPoints, m depends on parameterisation
                   %first row is always timestamps
        model      %function handle of trajectory model
    end
    
    
    %% 2. Methods
    % Constructor
    methods(Access = public) %set to private later??
        function self = Trajectory(parameterisation,mode,varargin)
            switch nargin
                case 0 %allows pre-allocation
            end
        end
        
    end
       
    % Getter & Setter
    methods(Access = public) %set to protected later??
        function out = get(self,property,varargin)
            if (nargin==2)
                out = self.(property);
            elseif (nargin>2)
                %do something with varargin depending on property
                out = self.(property);
            end
            
        end
        
        function self = set(self,property,value,varargin)
            if (nargin==3)
                self.(property) = value;
            elseif (nargin>3)
                %do something with varargin depending on property
                self.(property) = value;
            end
            
        end
    end
    
    
    % Get trajectory @ input times
    methods(Access = public) %set to private later??
%         function dataPoints = getDataPoints(self,t,fitType)
%             %inputs
%             %   t - 1xn array of times for which trajectory desired
%             %   fitting method - string selecting how to get poses from
%             %   existing datapoints
%             %output
%             %   dataPoints - trajectory value at times in t
%             
%             %preallocate dataPoints
%             dataPoints = zeros(size(self.dataPoints,1),numel(t));
%             
%             %check which t match with timeStamps?
%             
%             
%             
%         end
    end
end

