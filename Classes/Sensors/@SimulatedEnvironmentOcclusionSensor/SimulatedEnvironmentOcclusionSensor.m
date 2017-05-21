classdef SimulatedEnvironmentOcclusionSensor < SimulatedEnvironmentSensor
    %SimulatedEnvironmentSensor represents a sensor used to generate
    %measurements of a primitives and points in the Environment class
    %   -SimulatedEnvironmentSensor converts the environment primitives and
    %    environment points to objects and points
    %   -This intermediate object representation allows more freedom in the
    %    kinds of observations and constraints which can be generated from
    %    environment primitives and points
    %   -ie different SimulatedEnvironmentSensor subclasses can rerepresent
    %    the same Environment class in different ways (ie planes vs
    %    rectangles)
    %   -Measurements of these points are generated and stored in a graph
    %    file, along with a ground truth graph file.
    
    %% 1. Properties
    properties
    end
    
    %% 2. Methods
  
    % Add environment
    methods(Access = public)
    end
    
    %Declare external methods
    methods(Access = public)
        % point visibility
        [visibility,relativePoint] = pointOcclusion(self,point,t)
        % Measurements
        generateMeasurements(self,config)
        
    end
    
end

