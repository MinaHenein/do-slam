classdef SimulatedEnvironmentSensor < Sensor
    %SimulatedEnvironmentSensor Summary of this class goes here
    %   Detailed explanation goes here
    
    %% 1. Properties
    properties(GetAccess = 'protected', SetAccess = 'protected')
        fieldOfView
    end
    
    %% 2. Methods
    % Constructor
    methods(Access = public)
        function self = SimulatedEnvironmentSensor(fieldOfView,trajectory)
            self.fieldOfView = fieldOfView;
            self.trajectory  = trajectory;
        end
    end
    
    % Getter & Setter
    methods(Access = public) %set to protected later??
        function out = getSwitch(self,property,varargin)
            switch property
                case {'GP_Pose','R3xso3Pose','logSE3Pose','R3xso3Position','logSE3Position','axisAngle','R'}
                    out = self.trajectory.get(property,varargin{1});
                case 'static'
                    out = self.trajectory.get(property);
                otherwise
                    out = self.(property);
            end
        	
        end
        
        function self = setSwitch(self,property,value)
        	self.(property) = value;
        end
    end
    
    %Declare external methods
    methods(Access = public)
        % Measurements
        generateMeasurements(self,config,sensorEnvironment)
    end
    
end

