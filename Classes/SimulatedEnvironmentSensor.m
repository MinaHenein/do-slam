classdef SimulatedEnvironmentSensor < Sensor
    %SimulatedEnvironmentSensor Summary of this class goes here
    %   Detailed explanation goes here
    
    %% 1. Properties
    properties(GetAccess = 'protected', SetAccess = 'protected')
        fieldOfView
        maxRange
    end
    
    %% 2. Methods
    % Constructor
    methods(Access = public)
        function self = SimulatedEnvironmentSensor(fieldOfView,maxRange,trajectory)
            self.fieldOfView = fieldOfView;
            self.maxRange    = maxRange;
            self.trajectory  = trajectory;
        end
    end
    
end

