classdef Trajectory
    %TRAJECTORY class represents trajectories of moving/stationary
    %components (robots, objects, points )
    %   Both dynamic and static objects will use trajectory class. Only
    %   difference is model used.
    
    %% 1. Properties
    properties(GetAccess = 'public', SetAccess = 'public')
        parameterisation %string
    end
    properties(GetAccess = 'protected', SetAccess = 'protected')
        timeStamps %1xn array of time values of each dataPoint
        dataPoints %mxn array of dataPoints, m depends on parameterisation
        model      %function handle of trajectory model
    end
    
    
    %% 2. Methods
    % Constructor
    methods(Access = public) %set to private later??
        function self = Trajectory(parameterisation,mode,varargin)
            switch nargin
                case 0
                    %allows pre-allocation
                otherwise
                    self.parameterisation = parameterisation;
                    switch mode
                        case 'discrete'
                            self.timeStamps = varargin{1};
                            self.dataPoints = varargin{2};
                        case 'continuous'
                            self.model = varargin{1};
                    end
            end
        end
        
    end
    
    % Getter & Setter
end

