classdef Environment < handle
    %ENVIRONMENT Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        GeometricPrimitives
        Features
    end
    
    methods
        function obj = addGeometricPrimitive(obj,GeometricPrimitive)
            obj.GeometricPrimitives(end+1) = GeometricPrimitive;
        end
        
        function obj = addFeature(obj,Feature)
            obj.Feature = Feature;
        end
        
        function plotEnvironment(obj)
            % adapt code from plotMap, get this to plot the Map (with
            % Meshes)
        end
    end
    
end

